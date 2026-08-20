#ifndef _CSC_EVT_H
#define _CSC_EVT_H

/*******************
 * TYPEDEFS/MACROS
 *******************/
typedef void(*csv_evt_tick_cb)(void);




/*********************
 * PUBLIC APIs
 ********************/


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

#endif