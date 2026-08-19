#ifndef _ACCEL_H
#define _ACCEL_H


#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

/*********************
 * MACROS/TYPEDEFS
 *********************/
typedef enum {
    ACCEL_MODE_CADENCE,
    ACCEL_MODE_SPEED
} accel_mode_e;



/**********************
 * PUBLIC APIs
 *********************/


/**
 * @brief Initialize and configure the BMI160 accelerometer
 * 
 * This API can configure the accelerometer either for cadence
 * measurement or speed measurement.
 * 
 * @param config_mode config mode for accel (cadence/speed)
 */
void accel_init(accel_mode_e);



/**
 * @brief 
 */


#endif /* ACCEL_H */