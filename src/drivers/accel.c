#include "../../include/drivers/accel.h"


/************************
 * STATIC DECLARATIONS
 ************************/




/**********************
 * USER APIs
 **********************/

 
/**
 * @brief Initialize and configure the BMI160 accelerometer
 * 
 * This API can configure the accelerometer either for cadence
 * measurement or speed measurement.
 * 
 * @param config_mode config mode for accel (cadence/speed)
 */
void accel_init(accel_mode_e);



/*********************
 * STATIC DEFS
 *********************/



