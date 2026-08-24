#ifndef _BARO_ALT_H
#define _BARO_ALT_H

#include <stdint.h>

/**************
 * MACROS
 **************/
#define SEA_LVL_PRESS   (1013.25f) // hPa
#define SEA_LVL_TEMP    (288.15f)    // K
#define DRY_AIR_CONST   (287.05f)  // J/(kg * K)
#define G_ACCEL         (9.81f)    // m/s^2
#define TEMP_LAPSE_RATE (0.0065f) // K/m

/********************
 * STRUCTS/ENUMS
 ********************/
struct baro_alt_samples {
    uint32_t temp;
    float altitude;
};


/******************
 * PUBLIC APIs
 *******************/

/**
 * @brief Initialize devices configurations
 * 
 * 
 */
int bmp280_init(void);



/**
 * @brief Retrieve sensor samples 
 * 
 * 
 */
int bmp280_get_samples(struct baro_alt_samples *);





#endif
