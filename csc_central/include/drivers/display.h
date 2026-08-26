#ifndef _DISPLAY_H
#define _DISPLAY_H

/****************
 * MACROS
 ****************/
#define DISPLAY_WIDTH   (128U)
#define DISPLAY_HEIGHT  (64U)

/*********************
 * STRUCTS/ENUMS
 *********************/
struct display_stats {
    uint32_t timestamp;
    float speed;
    float avg_speed;
    float elevation;
    float total_elevation;
    float distance;
    float total_dist;
    float cadence;
    float avg_cadence;
    float temp;
};


/*********************
 * PUBLIC APIs
 ********************/

/**
 * @brief Intiialize and configure display
 * 
 * The testing display will be an SSD1306 0.96
 * inch OLED.
 * 
 */
int display_init(void);


/**
 * @brief This function posts updated stats to display thread
 * 
 * 
 * The stats thread will post updated statistics
 * to the display thread.
 * 
 */
int display_signal_update(struct display_stats *stats);



#endif