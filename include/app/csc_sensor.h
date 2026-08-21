#ifndef _CSC_SENSOR_H
#define _CSC_SENSOR_H

#include "../drivers/accel.h"
#include "../drivers/csc_ble.h"
/**************************
 * TYPEDEFS/ENUMS/STRUCTS
 *************************/
enum sensor_state {
    SENSOR_STATE_IDLE,
    SENSOR_STATE_ADV,
    SENSOR_STATE_CONNECTED,
};


/*******************
 * PUBLIC APIs
 ********************/


/**
 * @brief Initialize BLE stack and accelerometer submodule
 * 
 * This functions initializes the underlying BLE stack
 * and the accelerometer submodule. It also allows for 
 * switching between Cadence and Speed modes.
 * 
 */
int csc_sens_init(void);

#endif

