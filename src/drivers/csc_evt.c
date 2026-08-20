#include "../../include/drivers/csc_evt.h"
#include <nrfx_timer.h>

/***************
 * MACROS
 ***************/






/*******************
 * STATIC DECLARATIONS
 ***********************/





/*******************
 * PUBLIC APIs
 *******************/



/**
 * @brief Init timer for Last Crank/Wheel Event times
 * 
 * 
 */
void csc_evt_init(void);


/**
 * @brief Resets the timer peripheral
 * 
 * 
 */
void csc_evt_tim_reset(void);


/**
 * @brief Registers the TICK inc callback function
 * 
 * 
 */
void csc_evt_reg_tick_cb(csv_evt_tick_cb tick_cb);



/********************
 * STATIC DEFS
 *******************/
