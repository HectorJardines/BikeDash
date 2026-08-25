#include "../../include/app/statistics.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/device.h>

LOG_MODULE_REGISTER(stats_mod, LOG_LEVEL_DBG);

/********************
 * STRUCTS/MACROS
 *******************/
#define WHEEL_CIRC              (28U) // measured in CM
#define EMA_ALPHA_AVG           (0.5f)
#define CSC_EVT_PERIOD          (1U) // measured in seconds
#define CSC_XEVT_PERIOD         (.000976f) // measured in useconds 1/1024 seconds
#define ELEVATION_GAIN_THRSH    (2U) // measured in meters

#define NVS_PARITION            storage_partition
#define NVS_PARTITION_DEVICE    PARTITION_DEVICE(NVS_PARITION)
#define NVS_PARTITION_OFFSET    PARTITION_OFFSET(NVS_PARITION)

#define TOTAL_DIST_ID       (1U)
#define TOTAL_ELEV_ID       (2U)
#define PREV_RIDE_STATS_ID  (3U)

struct dist_info {
    float total_distance;
    float curr_ride_distance;
    float sample;
};

struct speed_info {
    float current_speed;
    float avg_speed;
    struct csc_sensor_samples prev_wevt;
    struct csc_sensor_samples curr_wevt;
};


struct cadence_info {
    uint32_t current_cadence;
    uint32_t avg_cadence;
    struct csc_sensor_samples prev_cevt;
    struct csc_sensor_samples curr_cevt;
};


struct env_info {
    float ref_elevation;     /* STARTING POINT BEGINNING OF RIDE */
    float total_gain;
    float ride_gain;
    float curr_gain;

    uint16_t curr_temp;

    struct baro_alt_samples curr;
    struct baro_alt_samples prev;
};


/*************************
 * STATIC DECLARATIONS
 *************************/
static void stats_update_speed(void);
static void stats_update_distance(void);
static void stats_update_elevation_gain(void);
static void stats_update_cadence(void);
static void stats_update_temperature(void);
static void stats_thread_fn(void *p1, void *p2, void *p3);
static int stats_load_nvs(float *total_gain, float *total_dist);
static int stats_write_nvs(float *total_gain, float *total_dist);


static struct speed_info speed;
static struct cadence_info cadence;
static struct dist_info distance;
static struct env_info environment;

static struct nvs_fs fs;
static struct flash_pages_info pg_inf;

K_THREAD_DEFINE(stat_thread, 1024, stats_thread_fn, 
                NULL, NULL, NULL, 7, 0, 0);
K_MSGQ_DEFINE(stat_thread_q, sizeof(struct generic_sample), 5, 1);
/***********************
 * PUBLIC APIs
 ***********************/
/**
 * @brief Initialize cycle sensors
 * 
 * This function handles initialization of submodules 
 * like CSC client, baro-altimeter, gps (in the future).
 * 
 */
int32_t cycle_stats_init(void) {
    int32_t ret = 0;
    memset((void *)&speed, 0, sizeof(speed));
    memset((void *)&cadence, 0, sizeof(cadence));
    memset((void *)&distance, 0, sizeof(distance));
    memset((void *)&environment, 0, sizeof(environment));

    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("NVS PARITION DEVICE NOT READY\n\r");
        return 1;
    }

    fs.offset = NVS_PARTITION_OFFSET;
    ret = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &pg_inf);
    if (ret) {
        LOG_ERR("COULD NOT RETRIEVE PAGE INFORMATION: %d\n\r", ret);
        return 1;
    }

    fs.sector_size = pg_inf.size;
    fs.sector_count = 3U; // could be less 

    ret = nvs_mount(&fs);
    if (ret) {
        LOG_ERR("COULD NOT MOUNT NVS: %d\n\r", ret);
        return 1;
    }

    ret = stats_load_nvs(&environment.total_gain, &distance.total_distance);
    if (ret <= 0)
        LOG_WRN("FAILED TO LOAD STATS FROM NVS: %d\n\r", ret);
    return ret;
}


/**
 * @brief This function is called to post samples to the statistics thread
 * 
 * This statistics thread is responsible for taking sensor samples
 * and computing relevant statistics. E.g. instantaneous speed, 
 * total dist traveled, cycling cadence, altitude, etc.
 * 
 */
int32_t cycle_stats_post(struct generic_sample *sample) {
    int32_t ret = k_msgq_put(&stat_thread_q, (const void *)sample, K_NO_WAIT);
    if (ret)
        LOG_DBG("FAILED TO POST TO QUEUE: %d\n\r", ret);
    return ret;
}



/*********************
 * STATIC DECLARATIONS
 *********************/

static void stats_thread_fn(void *p1, void *p2, void *p3) {
    struct generic_sample sample;
    for (;;) {
        int ret = k_msgq_get(&stat_thread_q, (void *)&sample, K_FOREVER);
        if (!ret) {
            switch (sample.type) {
            case SAMPLE_CSC_CADENCE:
                cadence.curr_cevt.revs = sample.csc.revs;
                cadence.curr_cevt.evt_time = sample.csc.evt_time;
                stats_update_cadence();
                stats_update_distance();
                break;
            case SAMPLE_CSC_SPEED:
                speed.curr_wevt.revs = sample.csc.revs;
                speed.curr_wevt.evt_time = sample.csc.evt_time;
                stats_update_speed();
                stats_update_distance();
                break;
            case SAMPLE_PRESS_TEMP:
                environment.curr.altitude = sample.b_alt.altitude;
                environment.curr.temp = sample.b_alt.temp;
                environment.curr.evt_time = sample.b_alt.evt_time;
                stats_update_elevation_gain();
                stats_update_temperature();
                break;
            default:
                break;
            }
        }
        ret = stats_write_nvs(&environment.total_gain, &distance.total_distance);
    }
}


/**
 * @brief Given a current and previous wheel event compute speed
 * 
 * 
 * This function calculates speed as a function of total
 * wheel revolutions, time, and circumference of the 
 * wheel. This should be calculated once per second as 
 * data is received from the CSC sensors.
 * 
 */
static void stats_update_speed(void) {
    float evt_time_delta = (speed.curr_wevt.evt_time - speed.prev_wevt.evt_time) * CSC_XEVT_PERIOD;
    distance.sample = (speed.curr_wevt.revs * WHEEL_CIRC); // dist in CM
    distance.sample = distance.sample / evt_time_delta; // distance in kM
    float spd = distance.sample * 0.036f; // speed in km/h
    
    speed.prev_wevt.evt_time = speed.curr_wevt.evt_time;
    speed.prev_wevt.revs = speed.curr_wevt.revs;

    speed.current_speed = spd;
    // EMA AVG SPEED
    speed.avg_speed = (spd * EMA_ALPHA_AVG) + (speed.avg_speed * (1 - EMA_ALPHA_AVG));
}


/**
 * @brief Updates the current and total ride distance
 * 
 * This function should always be called AFTER stats_update_speed
 * the speed function computes the distance in CM and stores it
 * in the distance struct to be used here.
 * 
 */
static void stats_update_distance(void) {
    float dist_sample_km = distance.sample / 100000;
    distance.curr_ride_distance += dist_sample_km;
    distance.total_distance += dist_sample_km;
}


/**
 * @brief Calculates the elevation curr and total ride elevation gain
 * 
 * This function calculates the ride elevation. It filters out
 * any elevation gain under 2 meters. 
 * 
 */
static void stats_update_elevation_gain(void) {
    float elevation_delta = environment.curr.altitude - environment.prev.altitude;
    if (elevation_delta < 0)
        environment.curr_gain = 0;
    else
        environment.curr_gain += elevation_delta;

    if (environment.curr_gain > ELEVATION_GAIN_THRSH) {
        environment.ride_gain += elevation_delta;
        environment.total_gain += elevation_delta;
    }

    environment.prev.altitude = environment.curr.altitude;
    environment.prev.evt_time = environment.curr.evt_time;
}


/**
 * @brief Calculates the cadence in RPM
 * 
 * This function calculates instantaneous and avg
 * cadence (EMA). The function is called roughly once 
 * per second as BLE data is received from the CSC 
 * sensor.
 * 
 */
static void stats_update_cadence(void) {
    float evt_time_delta = (cadence.curr_cevt.evt_time - cadence.prev_cevt.evt_time) * CSC_XEVT_PERIOD;
    float cad =  cadence.curr_cevt.revs / evt_time_delta; // revs/s
    cad = cad / 60; // RPM
    cadence.current_cadence = cad;
    cadence.avg_cadence = (cad * EMA_ALPHA_AVG) + (cadence.avg_cadence * (1 - EMA_ALPHA_AVG));
    
    cadence.prev_cevt.evt_time = cadence.curr_cevt.evt_time;
    cadence.prev_cevt.revs = cadence.curr_cevt.revs;
}


/**
 * @brief Updates the current tempertature
 * 
 * This function should be called less often than 
 * the others as temperature should not expect
 * frequent fluctuation. Likely we will disable
 * the temperature samples for the BMP280 until 
 * we require them.
 * 
 */
static void stats_update_temperature(void) {
    environment.curr_temp = environment.curr.temp;
}


/**
 * @brief Loads total distance and elevation gain from NVS
 * 
 * This function should be called on system boot when the
 * statistic subsystem is initialized. This function will
 * retrieve the cumulative elevation gain and distance. 
 * This will likely be expaded to include previous ride stats
 * as well.
 * 
 */
static int stats_load_nvs(float *total_gain, float *total_dist) {
    int32_t ret = nvs_read(&fs, TOTAL_DIST_ID, (void *)total_dist, sizeof(*total_dist));
    if (ret <= 0)
        LOG_WRN("TOTAL DISTANCE VALUE NOT STORED IN NVS\n\r");
    ret = nvs_read(&fs, TOTAL_ELEV_ID, (void *)total_gain, sizeof(*total_gain));
    if (ret <= 0)
        LOG_WRN("TOTAL E-GAIN VALUE NOT STORED IN NVS\n\r");
    return ret;
}


/**
 * @brief Writes total distance and elevation gain to NVS
 * 
 * This function should be called periodically. Best time to 
 * call this function would be each time that an event is posted
 * to the statistic thread, if that event results in a change of 
 * either value.
 * 
 */
static int stats_write_nvs(float *total_gain, float *total_dist) {
    int32_t ret = nvs_write(&fs, TOTAL_ELEV_ID, (void *)total_gain, sizeof(*total_gain));
    if (ret <= 0)
        LOG_WRN("FAILED TO WRITE GAIN VALUE TO NVS: %d\n\r", ret);
    ret = nvs_write(&fs, TOTAL_DIST_ID, (void *)total_dist, sizeof(*total_dist));
    if (ret <= 0) 
        LOG_WRN("FAILED TO WRITE DIST VALUE TO NVS: %d\n\r", ret);
    return ret;
}

