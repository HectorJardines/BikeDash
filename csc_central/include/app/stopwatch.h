#ifndef _STOPWATCH_H
#define _STOPWATCH_H

#include <stdint.h>

/**************
 * MACROS
 **************/
#define SW_RST      (0U)
#define SW_LAP      (1U)
#define SW_STOP     (2U)
#define SW_START    (3U)


/********************
 * STRUCTS/ENUMS
 *******************/
enum sw_state {
    SW_IDLE,
    SW_RUNNING
};
 
struct sw_time {
    enum sw_state state;

    uint32_t s_since_start;     /* SECONDS SINCE START EPOCH */
    uint32_t start_epoch;       /* START EPOCH IN MS */
};



/**************
 * PUBLIC APIs
 **************/

/**
 * @brief 
 * 
 * 
 */
int stopwatch_init(void);


/**
 * @brief
 * 
 * 
 * 
 */
int32_t stopwatch_get_time(void);


#endif