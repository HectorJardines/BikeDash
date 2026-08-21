#include "../../include/drivers/csc_evt.h"
#include <zephyr/logging/log.h>
#include <nrfx_timer.h>

/***************
 * MACROS
 ***************/
#define TIMER_INST_NUM          NRF_TIMER1
#define TIMER_CSC_EVT_INTERVAL  (997U) // csc evt tim interval in uS
LOG_MODULE_REGISTER(csc_evt, LOG_LEVEL_DBG);


/*******************
 * STATIC DECLARATIONS
 ***********************/
static void tim_tick_handler(nrf_timer_event_t event_type, void * p_context);

static nrfx_timer_t tim_inst = NRFX_TIMER_INSTANCE(TIMER_INST_NUM);
static csv_evt_tick_cb tick_inc_cb = NULL;
/*******************
 * PUBLIC APIs
 *******************/



/**
 * @brief Init timer for Last Crank/Wheel Event times
 * 
 * 
 */
int csc_evt_init(void) {
    nrfx_timer_config_t tim_cnf = NRFX_TIMER_DEFAULT_CONFIG(1000000);
    int err = nrfx_timer_init(&tim_inst, &tim_cnf, tim_tick_handler);
    if (err) {
        LOG_ERR("FAILED TO INTIIALIZE TIMER\n\r");
        return err;
    }

    uint32_t timer_ticks = nrfx_timer_us_to_ticks(&tim_inst, TIMER_CSC_EVT_INTERVAL);
    nrfx_timer_extended_compare(&tim_inst, NRF_TIMER_CC_CHANNEL0, timer_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    return err;
}


/**
 * @brief Resets the timer peripheral
 * 
 * 
 */
void csc_evt_tim_enable(void) {
    nrfx_timer_enable(&tim_inst);
}



/**
 * @brief Resets the timer peripheral
 * 
 * 
 */
void csc_evt_tim_reset(void) {
    nrfx_timer_disable(&tim_inst);
}


/**
 * @brief Registers the TICK inc callback function
 * 
 * 
 */
void csc_evt_reg_tick_cb(csv_evt_tick_cb tick_cb) {
    tick_inc_cb = tick_cb;
}


/********************
 * STATIC DEFS
 *******************/
static void tim_tick_handler(nrf_timer_event_t event_type, void * p_context) {
    if (tick_inc_cb)
        tick_inc_cb();
    else
        LOG_WRN("NO TIMER EVT HANDLER HAS BEEN REGISTERED\n\r");
}
