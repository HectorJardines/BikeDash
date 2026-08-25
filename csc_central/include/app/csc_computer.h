#ifndef _COMP_H
#define _COMP_H
#include "../../include/drivers/csc_ble_cli.h"

/******************
 * MACROS
 ******************/



/********************
 * STRUCTS/ENUMS
 ********************/



/*********************
 * PUBLIC APIs
 ********************/


/**
 * @brief Initialize computer module and subsystems
 * 
 * 
 */
int comp_init(void);



/**
 * @brief Post event to the computer module
 * 
 * 
 */
int comp_post_event(uint32_t flag, struct csc_meas_notify *opt);



#endif  /* _COMP_H */
