#include "../../include/app/statistics.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(stats_mod, LOG_LEVEL_DBG);

/********************
 * STRUCTS/MACROS
 *******************/
#define WHEEL_CIRC              (28U) // measured in CM
#define EMA_ALPHA_AVG           (0.5f)
#define CSC_EVT_PERIOD          (1U) // measured in seconds
#define CSC_XEVT_PERIOD         (.000976f) // measured in useconds 1/1024 seconds
#define ELEVATION_GAIN_THRSH    (2U) // measured in meters

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


static struct speed_info speed;
static struct cadence_info cadence;
static struct dist_info distance;
static struct env_info environment;

K_THREAD_DEFINE(stat_thread, 1024, stats_thread_fn, 
                NULL, NULL, NULL, 7, 0, 0);
K_MSGQ_DEFINE(stat_thread_q, sizeof(struct generic_sample), 5, 4);
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
    memset((void *)&speed, 0, sizeof(speed));
    memset((void *)&cadence, 0, sizeof(cadence));
    memset((void *)&distance, 0, sizeof(distance));
    memset((void *)&environment, 0, sizeof(environment));
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
                break;
            case SAMPLE_CSC_SPEED:
                speed.curr_wevt.revs = sample.csc.revs;
                speed.curr_wevt.evt_time = sample.csc.evt_time;
                stats_update_speed();
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
    float spd = distance.sample * 0.036; // speed in km/h
    
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

