#ifndef _ACCEL_H
#define _ACCEL_H


#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

/*********************
 * MACROS/TYPEDEFS
 *********************/
 enum accel_ret {
    ACCEL_OK,
    ACCEL_ERR
};


enum accel_mode_e{
    ACCEL_MODE_CADENCE,
    ACCEL_MODE_SPEED
};


enum accel_quadrant {
    QUADRANT_0,
    QUADRANT_1,
    QUADRANT_2,
    QUADRANT_3
};


struct wheel_rev_info {
    uint32_t cwr;       /* CUMULATIVE WHEEL REV UPDATED EVERY WHEEL REV (CAN BE SET BY CENTRAL) */
};


struct crank_rev_info {
    uint32_t ccr;       /* CUMULATIVE CRANK COUNT UPDATED EVERY CRANK REV (CANNOT BE SET BY CENTRAL)*/
};

struct acc_rev_handle {
    uint8_t initialized;
    uint8_t accel_mode;
    uint8_t curr_quadrant;
    uint8_t last_quadrant;
    uint8_t bound_crossed_cnt;

    union {
        struct wheel_rev_info wr_inf;
        struct crank_rev_info cr_inf;
    };
};

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
uint8_t accel_init(void);

/**
 * @brief Retrieves the Cumulative Wheel/Crank Revolutions
 * 
 */
uint8_t accel_get_cxr(uint32_t *cxr);


/**
 * @brief Sets the accelerometer mode
 * 
 * This functions sets the accelerometer mode 
 * to either cadence mode or speed mode. Currently
 * they are more or less the same, may make some 
 * adjustments to configurations
 * 
 */
uint8_t accel_toggle_mode(void);

/**
 * @brief 
 */


#endif /* ACCEL_H */