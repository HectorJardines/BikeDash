#include "../../include/drivers/barometric_alt.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/drivers/i2c.h"
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <math.h>


LOG_MODULE_REGISTER(barometer_mod, LOG_LEVEL_DBG);

/**********************
 * STATIC DECLARATIONS
 ***********************/
static float bmp280_pressure_to_altitude(struct sensor_value *press, struct sensor_value *temp);

static const struct device *bmp280 = DEVICE_DT_GET_ANY(bosch_bme280);
/**********************
 * PUBLIC APIs
 ***********************/


/**
 * @brief Initialize devices configurations
 * 
 * 
 */
int bmp280_init(void) {
    int32_t ret = 1;

    if (!device_is_ready(bmp280)) {
        LOG_ERR("BMP280 SENSOR NOT READY\n\r");
        return ret;
    }
    ret = 0;
    return ret;
}


/**
 * @brief This function performs a one-shot sample of the BMP280 
 * 
 * The function reads the pressure and ambient temperature 
 * values from the BMP280 sensor. Addtionally, this 
 * function internally converts the pressure sample into 
 * absolute altitude.
 * 
 * 
 * @note This function will be called every second (1Hz SR)
 * this is ideal as this is the connection notify period
 * for the BLE connection as well.
 * 
 */
int bmp280_get_samples(struct baro_alt_samples *samples) {
    int ret = sensor_sample_fetch(bmp280);
    if (ret) {
        LOG_WRN("FAILED TO READ BMP280 SENSOR SAMPLES\n\r");
    }
    
    struct sensor_value press;
    struct sensor_value temp;

    ret = sensor_channel_get(bmp280, SENSOR_CHAN_PRESS, &press);
    if (ret) LOG_WRN("FAILED TO GET PRESSURE VALUE\n\r");
    ret = sensor_channel_get(bmp280, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    if (ret) LOG_WRN("FAILED TO GET AMB TEMP VALUE\n\r");

    samples->altitude = bmp280_pressure_to_altitude(&press, &temp);
    // we only care about integer part of ambient temp
    samples->temp = temp.val1;
}

/*********************
 * STATIC DEFS
 *********************/

static float bmp280_pressure_to_altitude(struct sensor_value *press, struct sensor_value *temp) {
    float altitude_sample = 0;
    // value returned as kPA, need hPa for our altitude conversion
    float press_raw = (press->val1 + (press->val2 * powf(10, -6)) * 10);

    float press_ratio = press_raw / SEA_LVL_PRESS;
    press_ratio = powf(press_ratio, (DRY_AIR_CONST * TEMP_LAPSE_RATE) / G_ACCEL);
    altitude_sample = (SEA_LVL_TEMP / TEMP_LAPSE_RATE) * (1 - press_ratio);
    return altitude_sample;
}


