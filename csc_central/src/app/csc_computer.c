#include "../../include/app/csc_computer.h"
#include "../../include/drivers/barometric_alt.h"
#include "../../include/app/statistics.h"
#include "../../include/common/defines.h"
#include "../../include/app/stopwatch.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(comp_mod, LOG_LEVEL_DBG);

/*****************
 * MACROS
 *****************/
#define COMP_THREAD_STK_SIZE        (1024U)
#define COMP_THREAD_PRIO            (7U)
#define NFY_MSG_Q_LEN               (3U)


/*******************
 * STATIC DECLARATIONS
***********************/
static void comp_thread_fn(void *p1, void *p2, void *p3);
static int comp_handle_ble_nfy_evt(const struct csc_meas_notify *nfy);
static void comp_btn_start_ble_scan_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void comp_btn_reset_cwr_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void configure_btn_ints(void);

K_EVENT_DEFINE(comp_events);
K_MSGQ_DEFINE(nfy_q, sizeof(struct csc_meas_notify), NFY_MSG_Q_LEN, 1);
K_THREAD_DEFINE(comp_thread, COMP_THREAD_STK_SIZE, comp_thread_fn, 
                NULL, NULL, NULL, COMP_THREAD_PRIO, 0, 0);
static const struct gpio_dt_spec cwr_btn = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_reset_cwr), gpios);
static const struct gpio_dt_spec scan_btn = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_scan), gpios);
static struct gpio_callback cwr_btn_cb;
static struct gpio_callback scan_btn_cb;
/*******************
 * PUBLIC APIs
*******************/
/**
 * @brief Initialize computer module and subsystems
 * 
 * 
 */
int comp_init(void) {
    int ret = csc_client_init();
    if (ret) {
        LOG_ERR("FAILED TO INITIALIZE CSC CLIENT SUBSYTEM: %d\n\r", ret);
        return ret;
    }

    ret = stopwatch_init();
    if (ret) {
        LOG_ERR("FAILED TO INTIALIZE STOPWATCH MODULE\n\r");
        return ret;
    }

    ret = bmp280_init();
    if (ret) {
        LOG_ERR("FAILED TO INTIALIZE BAROMETRIC ALTIMETER SUBSYSTEM: %d\n\r", ret);
        return ret;
    }

    if (!gpio_is_ready_dt(&cwr_btn) || !gpio_is_ready_dt(&scan_btn)) {
        LOG_ERR("BUTTONS NOT READY\n\r");
        ret = 1;
        return ret;
    }

    configure_btn_ints();
    return ret;

}



/**
 * @brief Post event to the computer module
 * 
 * This function sets a flag (or multiple) in the
 * computer module's event flag structure. The
 * function accepts an optional parameter for 
 * BLE nfy events. This optional parameter will be 
 * pushed into the rev_nfy message queue.
 * 
 */
int comp_post_event(uint32_t flag, struct csc_meas_notify *opt) {
    int32_t ret = 0;
    k_event_set(&comp_events, flag);
    if ((flag & EVT_BLE_NFY_Msk) && opt) 
        ret = k_msgq_put(&nfy_q, (const void *)opt, K_NO_WAIT);

    return ret;
}


/**********************
 * STATIC DEFINITIONS
 ***********************/

/**
 * @brief Main thread, processes BLE notifications, samples sensors, etc.
 * 
 * This funciton implements the cycling computer thread
 * logic. This function will handle periodic BLE notifications
 * (~once per second), and will sample additional sensors at the 
 * same interval to avoid multiple wakeups.
 * 
 */
static void comp_thread_fn(void *p1, void *p2, void *p3) {
    int32_t ret = 0;
    static struct csc_meas_notify rev_nfy;
    for (;;) {
        ret = k_event_wait(&comp_events, (EVT_BLE_NFY_Msk | EVT_RST_CWR_REQ_Msk | EVT_SCAN_REQ_Msk), false, K_FOREVER);
        if (ret) {
            if (ret & EVT_BLE_NFY_Msk) { // should occur at 1Hz freq, want to sample other sensors as well
                ret = comp_handle_ble_nfy_evt(&rev_nfy);
                k_event_clear(&comp_events, EVT_BLE_NFY_Msk);
            }
            if (ret & EVT_RST_CWR_REQ_Msk) {
                uint32_t rst_cwr = 0;
                ret = cli_req_sc_cp_op(SC_CP_OP_SET_CWR, (void *)&rst_cwr, CSC_SENSOR_MODE_SPEED);
                if (ret)
                    LOG_WRN("CWR RESET FAILED: %d\n\r", ret);
                k_event_clear(&comp_events, EVT_RST_CWR_REQ_Msk);
            }
            if (ret & EVT_SCAN_REQ_Msk) {
                ret = csc_client_scan();
                if (ret)
                    LOG_WRN("FAILED TO INITIATE CSC CLIENT SCAN\n\r");
                k_event_clear(&comp_events, EVT_SCAN_REQ_Msk);
            }
            if (ret & EVT_SW_CHANGE_STATE_Msk) {
                
            }
        }
    }
}



/**
 * @brief This function handles the periodic (1Hz) BLE notifications
 * 
 * 
 * This function essentially handles all arbitration of sensor value 
 * updates. The idea is that we expect a fixed 1Hz frequency
 * from the CSC peripheral. Thus we have based all of our other readings
 * around this event in order to avoid additional timer logic/wakeup 
 * events. Essentially we trade a bit of precision for less wakeup of the 
 * CPU and thus less current consumption
 * 
 * 
 */
static int comp_handle_ble_nfy_evt(const struct csc_meas_notify *rev_nfy) {
    struct generic_sample sample;
    int32_t ret = k_msgq_get(&nfy_q, (void *)rev_nfy, K_NO_WAIT);
    if (ret) {
        LOG_ERR("EXPECTED REV NOTIFY OBJECT, GOT NOTHING: %d\n\r", ret);
        return ret;
    }

    // needs to be processed first as it sleeps the thread and possible triggers unecessary display refreshes
    sample.type = SAMPLE_PRESS_TEMP;
    ret = bmp280_get_samples(&sample.b_alt);
    if (ret)
        LOG_WRN("FAILED TO RETRIEVE PRESSURE SENSOR SAMPLES\n\r");
    else
        cycle_stats_post(&sample);


    if (rev_nfy->flags & CSC_WHEEL_REV_DATA_PRESENT) {
        memcpy((void *)&sample.csc, (const void *)rev_nfy->data, sizeof(sample.csc));
        sample.type = SAMPLE_CSC_SPEED;
    } else if (rev_nfy->flags & CSC_CRANK_REV_DATA_PRESENT) {
        memcpy((void *)&sample.csc, (const void *)rev_nfy->data, sizeof(sample.csc));
        sample.type = SAMPLE_CSC_CADENCE;
    }

    ret = cycle_stats_post(&sample);
    if (ret)
        LOG_WRN("FAILED TO POST REV INFO\n\r");

    ret = stopwatch_get_time();
    if (ret > 0) {
        sample.timestamp = ret;
        ret = 0;
    }

    return ret;
}

static void comp_btn_start_ble_scan_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    k_event_set(&comp_events, EVT_SCAN_REQ_Msk);
}


static void comp_btn_reset_cwr_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    k_event_set(&comp_events, EVT_RST_CWR_REQ_Msk);
}


static void configure_btn_ints(void) {
    // CONFIGURE CWR RESET BTN INTERRUPT
    gpio_pin_configure_dt(&cwr_btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&cwr_btn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&cwr_btn_cb, comp_btn_reset_cwr_isr, BIT(cwr_btn.pin));
    gpio_add_callback_dt(&cwr_btn, &cwr_btn_cb);
    
    // CONFIGURE SCAN BTN INTERRUPT
    gpio_pin_configure_dt(&scan_btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&scan_btn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&scan_btn_cb, comp_btn_start_ble_scan_isr, BIT(scan_btn.pin));
    gpio_add_callback_dt(&scan_btn, &scan_btn_cb);
}

