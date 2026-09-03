#include "../../include/drivers/accel.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(acc_logging, LOG_LEVEL_DBG);


/********************
 * MACROS
 ******************/
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB
#define NOISE_MARGIN (0.05)
#define BOUNDARIES_PER_REV  (4U)

#define IS_IN_QUADRANT0(A, B) ((A) > NOISE_MARGIN && (B) < -NOISE_MARGIN)
#define IS_IN_QUADRANT1(A, B) ((A) < -NOISE_MARGIN && (B) < -NOISE_MARGIN)
#define IS_IN_QUADRANT2(A, B) ((A) < -NOISE_MARGIN && (B) > NOISE_MARGIN)
#define IS_IN_QUADRANT3(A, B) ((A) > NOISE_MARGIN && (B) > NOISE_MARGIN)


/************************
 * STATIC DECLARATIONS
 ************************/
static void accel_data_ready_cb(const struct device *dev,
					 const struct sensor_trigger *trigger);
static void accel_data_to_revs(uint32_t axis_a, uint32_t axis_b);

// static struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(bmi160), SPIOP);
static const struct device *dev_bmi160 = DEVICE_DT_GET_ANY(bosch_bmi160);
static const struct sensor_trigger bmi160_trig = {.type = SENSOR_TRIG_DATA_READY, .chan = SENSOR_CHAN_ACCEL_XYZ};

static struct acc_rev_handle h_rev;
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
uint8_t accel_init(void) {
    uint8_t ret = ACCEL_OK;

    if (dev_bmi160 == NULL) {
        LOG_ERR("FAILED TO RETRIEVE DEVICE: BMI160\n\r");
        return ACCEL_ERR;
    }

    // Kernel sets LPM, US bit, 4G range, 50 Hz ODR, does NOT set undersampling # of samples
    if (!device_is_ready(dev_bmi160)) {
        LOG_ERR("BMI160 DEV NOT READY\n\r");
        return ACCEL_ERR;
    }

    ret = sensor_trigger_set(dev_bmi160, &bmi160_trig, accel_data_ready_cb);
    if (ret) {
        LOG_ERR("FAILED TO SET BMI160 INTERRUPT TRIGGER\n\r");
        return ret;
    }

    
    h_rev.initialized = 1;

    LOG_DBG("ACCEL CONFIGURED SUCCESSFULLY\n\r");
    return ret;
}



/**
 * @brief Sets the accelerometer mode
 * 
 * This functions sets the accelerometer mode 
 * to either cadence mode or speed mode. Currently
 * they are more or less the same, may make some 
 * adjustments to configurations
 * 
 */
uint8_t accel_toggle_mode(void) {
    if (h_rev.accel_mode == ACCEL_MODE_CADENCE) {
        h_rev.accel_mode = ACCEL_MODE_SPEED;
        h_rev.wr_inf.cwr = 0;
    }
    else {
        h_rev.accel_mode = ACCEL_MODE_CADENCE;
        h_rev.cr_inf.ccr = 0;
    }
    h_rev.bound_crossed_cnt = 0;

    return h_rev.accel_mode;
}


/**
 * @brief Returns the configured accelerometer mode
 */
uint8_t accel_get_mode(void) {
    return h_rev.accel_mode;
}


/**
 * @brief Retrieves the Cumulative Wheel/Crank Revolutions
 * 
 */
uint8_t accel_get_cxr(uint32_t *cxr) {
    if (!h_rev.initialized) {
        LOG_WRN("ACCEL NOT INITIALIZED, CANNOT RETRIEVE CXR\n\r");
        return 1;
    }

    if (h_rev.accel_mode == ACCEL_MODE_CADENCE)
        *cxr = h_rev.cr_inf.ccr;
    else
        *cxr = h_rev.wr_inf.cwr;

    return 0;
}



/*********************
 * STATIC DEFS
 *********************/


/**
 * @brief Retrieves accel XYZ sensor samples 
 * 
 * 
 * 
 * 
 * @note this callback is called from the zephyr 
 * workqueue thread
 */
static void accel_data_ready_cb(const struct device *dev,
					 const struct sensor_trigger *trigger) 
{
    struct sensor_value sample_x, sample_y, sample_z;
    uint8_t ret = sensor_sample_fetch(dev);
    if (ret) goto cleanup;

    ret = sensor_channel_get(dev_bmi160, SENSOR_CHAN_ACCEL_X, &sample_x);
    if (ret) goto cleanup;
    float acc_x = sensor_value_to_double(&sample_x);
    ret = sensor_channel_get(dev_bmi160, SENSOR_CHAN_ACCEL_Y, &sample_y);
    if (ret) goto cleanup;
    float acc_y = sensor_value_to_double(&sample_y);
    ret = sensor_channel_get(dev_bmi160, SENSOR_CHAN_ACCEL_Z, &sample_z);
    if (ret) goto cleanup;
    float acc_z = sensor_value_to_double(&sample_z);

    LOG_DBG("ACC X: %.4f, ACC Y: %.4f, ACC Z: %.4f", acc_x, acc_y, acc_z);
    accel_data_to_revs(acc_x, acc_z); // prob have to change axes for cadence/speed sensors
    (void)acc_z;
    return;
cleanup:
    LOG_ERR("ERROR IN READING SENSOR SAMPLES: %d\n\r", ret);
    return;
}


static void accel_data_to_revs(uint32_t axis_a, uint32_t axis_b) {
    if (IS_IN_QUADRANT0(axis_a, axis_b)) {
        h_rev.curr_quadrant = QUADRANT_0;
    } else if (IS_IN_QUADRANT1(axis_a, axis_b)) {
        h_rev.curr_quadrant = QUADRANT_1;
    } else if (IS_IN_QUADRANT2(axis_a, axis_b)) {
        h_rev.curr_quadrant = QUADRANT_2;
    } else if (IS_IN_QUADRANT3(axis_a, axis_b)) {
        h_rev.curr_quadrant = QUADRANT_3;
    }

    if (h_rev.curr_quadrant != h_rev.last_quadrant) {
        h_rev.bound_crossed_cnt++;
        h_rev.last_quadrant = h_rev.curr_quadrant;
        if (h_rev.bound_crossed_cnt == BOUNDARIES_PER_REV) {
            switch (h_rev.accel_mode) {
            case (ACCEL_MODE_CADENCE):
                h_rev.cr_inf.ccr++;
                break;
            case (ACCEL_MODE_SPEED):
                h_rev.wr_inf.cwr++;
                break;
            default:
                break;
            }
            h_rev.bound_crossed_cnt = 0;
        }
    }
}

