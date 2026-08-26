#include "../../include/app/stopwatch.h"
#include "../../include/drivers/display.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>


LOG_MODULE_REGISTER(sw_mod, LOG_LEVEL_DBG);

/***************
 * MACROS
 ***************/
#define RTC_NODE    DT_NODELABEL(rtc2)
#define MS_TO_SEC_RATIO     (1000)


/**********************
 * STATIC DECLARATIONS
 ***********************/
static void btn_stopwatch_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);


static const struct device *rtc_dev = DEVICE_DT_GET(RTC_NODE);
static const struct gpio_dt_spec btn_sw = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_stopwatch_ctrl), gpios);
static struct gpio_callback btn_sw_cb;
static struct sw_time stopwatch;
/*********************
 * PUBLIC APIs
 *********************/

/**
 * @brief 
 * 
 * 
 */
int stopwatch_init(void) {
    int32_t ret = 0;
    if (!device_is_ready(rtc_dev)) {
        LOG_ERR("RTC IS NOT READY\n\r");
        return 1;
    }

    if (!gpio_is_ready_dt(&btn_sw)) {
        LOG_ERR("GPIO DEV NOT READY\n\r");
        return 1;
    }

    ret |= gpio_pin_configure_dt(&btn_sw, GPIO_INPUT);
    ret |= gpio_pin_interrupt_configure_dt(&btn_sw, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn_sw_cb, btn_stopwatch_cb, BIT(btn_sw.pin));
    ret |= gpio_add_callback_dt(&btn_sw, &btn_sw_cb);

    if (ret) {
        LOG_ERR("FAILED TO CONFIGURE SW BTN GPIO\n\r");
        return ret;
    }

    stopwatch.state = SW_IDLE;
    return ret;
}


/**
 * @brief
 * 
 * 
 * 
 */
int32_t stopwatch_get_time(void) {
    if (stopwatch.state == SW_IDLE) {
        LOG_WRN("SW NOT STARTED CANNOT GET TIME\n\r");
        return -1;
    }

    int32_t curr_ms = k_uptime_get_32();
    // NEEDS OVERFLOW LOGIC
    stopwatch.s_since_start = (curr_ms - stopwatch.start_epoch) / MS_TO_SEC_RATIO;
    return stopwatch.s_since_start;
}

/******************
 * STATIC DEFS
 *******************/

static void btn_stopwatch_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    if (stopwatch.state == SW_IDLE) {
        stopwatch.state = SW_RUNNING;
        stopwatch.start_epoch = k_uptime_get_32();
    } else if (stopwatch.state == SW_RUNNING) {
        stopwatch.state = SW_IDLE;
        stopwatch.s_since_start = 0;
    }
}


