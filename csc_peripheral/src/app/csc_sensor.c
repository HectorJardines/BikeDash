#include "../../include/app/csc_sensor.h"
#include "../../include/common/defines.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(csc_sens, LOG_LEVEL_DBG);
#define SENSOR_STK_SIZE     (1024U)
#define SENSOR_THRD_PRIO    (7U)
#define RTC_NODE DT_NODELABEL(rtc2)

/************************
 * STATIC DECLARATIONS
 ************************/
static void sens_thread_fn(void *a1, void *a2, void *a3);
static void sens_tim_cb(struct k_timer *timer_id);
static int get_sensor_evt_time(uint16_t *evt_ticks);
static void gpio_configure_ints(void);
static void adv_button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void mode_button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

K_THREAD_DEFINE(csc_sensor_thread, SENSOR_STK_SIZE, sens_thread_fn, NULL, NULL, NULL, SENSOR_THRD_PRIO, 0x00, 0);
K_EVENT_DEFINE(sens_event);
K_TIMER_DEFINE(sens_data_tim, sens_tim_cb, NULL);


static enum sensor_state curr_state = SENSOR_STATE_IDLE;
static enum accel_mode_e curr_mode = ACCEL_MODE_CADENCE;

static const struct device *rtc_dev = DEVICE_DT_GET(RTC_NODE);
static const struct gpio_dt_spec mode_btn = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_mode), gpios);
static const struct gpio_dt_spec adv_btn = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_adv), gpios);
static struct gpio_callback mode_btn_cb;
static struct gpio_callback adv_btn_cb;
/********************
 * PUBLIC APIs
 ********************/


int csc_sens_init(void) {


    int err = accel_init();
    if (err) {
        LOG_ERR("FAILED TO INITIALIZED ACCEL SUBSYTEM\n\r");
        return 1;
    }
    err = csc_ble_init();
    if (err) {
        LOG_ERR("FAILED TO INITIALIZE BLE SUBSYSTEM\n\r");
        return 1;
    }

    if (!gpio_is_ready_dt(&mode_btn) || !gpio_is_ready_dt(&adv_btn)) {
        LOG_ERR("GPIO PERIPHERAL NOT READY\n\r");
        return 1;
    }

    if (!device_is_ready(rtc_dev)) {
        LOG_ERR("RTC1 NOT READY\n\r");
        return 1;
    }

    gpio_configure_ints();
    
    LOG_DBG("SUCCESSFULLY INITIALIZED CSC SENSOR\n\r");
    return 0;
}



/***********************
 * STATIC DEFINITIONS
 ***********************/
/**
 * @brief CSC Sensor thread function
 * 
 * 
 */
static void sens_thread_fn(void *a1, void *a2, void *a3) {
    int ret = 0;
    uint32_t evts = 0x00;
    uint32_t rev_cnt = 0;
    uint16_t evt_time = 0;
    while (1) {
        evts = k_event_wait(&sens_event, EVT_BLE_CONN_ADV_START | EVT_BLE_CONN_EST | EVT_BLE_CONN_TERM, false, K_FOREVER);
        if (evts & EVT_BLE_CONN_ADV_START) {
            ret = csc_ble_start_adv();
            if (!ret)
                curr_state = SENSOR_STATE_ADV;
            k_event_clear(&sens_event, EVT_BLE_CONN_ADV_START);
        } else if (evts & EVT_BLE_CONN_EST) {
            curr_state = SENSOR_STATE_CONNECTED;
            k_timer_start(&sens_data_tim, K_SECONDS(1), K_SECONDS(1));
            k_event_clear(&sens_event, EVT_BLE_CONN_EST);
        } else if (evts & EVT_BLE_SEND_DATA) {
            accel_get_cxr(&rev_cnt);
            get_sensor_evt_time(&evt_time);
            if (curr_mode == ACCEL_MODE_CADENCE)
                csc_ble_measurement_notify(0, 0, rev_cnt, evt_time);
            else
                csc_ble_measurement_notify(rev_cnt, evt_time, 0, 0);
            k_event_clear(&sens_event, EVT_BLE_SEND_DATA);
        } else if (evts & EVT_BLE_CONN_TERM) {
            curr_state = SENSOR_STATE_IDLE;
            k_timer_stop(&sens_data_tim);
            k_event_clear(&sens_event, EVT_BLE_CONN_TERM);
        }
    }
}


/**
 * @brief This function configures mode and adv_start button interrupts
 * 
 * This function configures the button GPIO pins with 
 * falling-edge triggered interrupts for mode toggling
 * (cadence or speed) and BLE advertising start
 * 
 */
static void gpio_configure_ints(void) {
    gpio_pin_configure_dt(&mode_btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&mode_btn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&mode_btn_cb, mode_button_handler, BIT(mode_btn.pin));
    gpio_add_callback(mode_btn.port, &mode_btn_cb);

    gpio_pin_configure_dt(&adv_btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&adv_btn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&adv_btn_cb, adv_button_handler, BIT(adv_btn.pin));
    gpio_add_callback(adv_btn.port, &adv_btn_cb);
}


/**
 * @brief Callback function for adv button ISR
 * 
 * If the sensor is in the appropriate mode, this functions
 * signals the sensor task to begin the BLE advertising
 * phase
 * 
 */
static void adv_button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (curr_state == SENSOR_STATE_ADV) {
        LOG_WRN("ADV CURRENTLY ONGOING, CANNOT START AGAIN\n\r");
        return;
    } else if (curr_state == SENSOR_STATE_CONNECTED) {
        LOG_WRN("CONN EST, CANNOT ADV UNTIL CONN TERM\n\r");
        return;
    } else 
        k_event_post(&sens_event, EVT_BLE_CONN_ADV_START);
}

/**
 * @brief This function retrieves the 1/1024 s unit ticks at the time of current CSC evt
 * 
 * This function takes the RTC peripheral count value and converts it into ticks
 * measured in 1/1024 s units. This tick count is sent to the BLE central
 * along with the cumulative revolution count periodically
 */
static int get_sensor_evt_time(uint16_t *evt_ticks) {
    uint32_t rtc_ticks = 0;
    // 32.768kHz LSE
    int err = counter_get_value(rtc_dev, &rtc_ticks);
    if (err) {
        LOG_ERR("FAILED TO RETRIEVE RTC TICKS\n\r");
        return err;
    }

    // we want ticks measured in 1/1024 s units with rollover at 65535
    *evt_ticks = ((rtc_ticks >> 5) & 0xFFFF);
    return err;
}


/**
 * @brief Mode button callback function
 * 
 * This function toggles the current accelerometer
 * mode between cadence and speed modes. This callback
 * has no effect if the sensor is not in IDLE state.
 * 
 * @note may want to just mask this interrupt while not in IDLE
 */
static void mode_button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (curr_state != SENSOR_STATE_IDLE) {
        LOG_WRN("SENSOR MUST BE IN IDLE STATE TO CHANGE MODE\n\r");
        return;
    }

    curr_mode = accel_toggle_mode();
}


/**
 * @brief BLE send data timer callback
 * 
 * This functions triggers every second when the 
 * sensor is in the connected state
 */
static void sens_tim_cb(struct k_timer *timer_id) {
    k_event_post(&sens_event, EVT_BLE_SEND_DATA);
}
