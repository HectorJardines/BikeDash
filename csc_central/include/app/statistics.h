#ifndef _STATS_H
#define _STATS_H

#include <stdint.h>

/********************
 * MACROS
 ******************/





/******************
 * STRUCTS/ENUMS
 *******************/
struct baro_alt_samples {
    uint32_t temp;
    uint32_t pressure;
};

struct csc_sensor_samples {
    uint32_t revs;
    uint16_t evt_time;
};

struct generic_sample {
    uint8_t type;
    union {
        csc_sensor_samples csc;     /* CYCLING SPEED/CADENCE SENSORS */
        baro_alt_samples b_alt;     /* BAROMETRIC ALTIMETER */
    };
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

