#ifndef _STATS_H
#define _STATS_H

#include <stdint.h>
#include "../../include/drivers/barometric_alt.h"

/********************
 * MACROS
 ******************/
#define SAMPLE_CSC_CADENCE  (0U)
#define SAMPLE_CSC_SPEED    (1U)
#define SAMPLE_PRESS_TEMP   (2U)




/******************
 * STRUCTS/ENUMS
 *******************/
struct csc_sensor_samples {
    uint32_t revs;
    uint16_t evt_time;
}__packed;

struct generic_sample {
    uint8_t type;
    union {
        struct csc_sensor_samples csc;     /* CYCLING SPEED/CADENCE SENSORS */
        struct baro_alt_samples b_alt;     /* BAROMETRIC ALTIMETER */
    };
};

struct display_stats {
    float speed;
    float avg_speed;
    float elevation;
    float total_elevation;
    float distance;
    float total_dist;
    float cadence;
    float avg_cadence;
    float temp;
};

/********************
 * PUBLIC APIs
 ********************/


/**
 * @brief Initialize cycle sensors
 * 
 * This function handles initialization of submodules 
 * like CSC client, baro-altimeter, gps (in the future).
 * 
 */
int32_t cycle_stats_init(void);


/**
 * @brief This function is called to post samples to the statistics thread
 * 
 * This statistics thread is responsible for taking sensor samples
 * and computing relevant statistics. E.g. instantaneous speed, 
 * total dist traveled, cycling cadence, altitude, etc.
 * 
 */
int32_t cycle_stats_post(struct generic_sample *sample);


#endif

