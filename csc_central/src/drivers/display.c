#include "../../include/drivers/display.h"
#include "../../include/app/statistics.h"
#include "../../include/ui/ui.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

LOG_MODULE_REGISTER(disp_mod, LOG_LEVEL_DBG);

/***************
 * MACROS
***************/
#define DISP_THREAD_STK_SIZE    (1024U) // 1KB
#define DISP_THREAD_PRIO        (7U)
#define DISP_Q_MAX_LEN          (2U)
#define MAX_STR_STAT_LEN        (9U)
// #define MAX_STR_TOTAL_LEN       (9U) // total elevation/distance can be much larger

struct stat_strings {
    char cadence_str[MAX_STR_STAT_LEN];
    char avg_cadence_str[MAX_STR_STAT_LEN];
    char speed_str[MAX_STR_STAT_LEN];
    char avg_speed_str[MAX_STR_STAT_LEN];
    char elev_gain_str[MAX_STR_STAT_LEN];
    char total_gain_str[MAX_STR_STAT_LEN];
    char distance_str[MAX_STR_STAT_LEN];
    char total_distance_str[MAX_STR_STAT_LEN];
    char temp_str[MAX_STR_STAT_LEN];
};

/**********************
 * STATIC DECLARATIONS
 ***********************/
static void display_thread_fn(void *p1, void *p2, void *p3);
static void display_update_labels(struct display_stats *stats);


K_THREAD_DEFINE(disp_thread, DISP_THREAD_STK_SIZE, display_thread_fn, 
                NULL, NULL, NULL, DISP_THREAD_PRIO, 0, 0);
K_MSGQ_DEFINE(disp_q, sizeof(struct display_stats), DISP_Q_MAX_LEN, 1);
static const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
static struct display_stats active_stats = {0};
static struct stat_strings strs;
/********************
 * PUBLIC APIs
 ********************/
/**
 * @brief Initialize main screen objects
 * 
 * This function creates all of the main
 * screen objects including labels and values
 * for speed, cadence, distance, etc.
 * 
 */
int display_init(void) {
    int ret = 1;
    if (!device_is_ready(display)) {
        LOG_ERR("DISPLAY DEVICE NODE IS NOT READY\n\r");
        return ret;
    }

    ui_init();
    if (ret) {
        LOG_ERR("UI COULD NOT BE INTIIALIZED\n\r");
        return ret;
    }

    memset((void *)&strs, 0, sizeof(strs));

    lv_label_set_text_static(objects.avg_cad_val, strs.avg_cadence_str);
    lv_label_set_text_static(objects.cad_value, strs.cadence_str);
    lv_label_set_text_static(objects.ttl_dist_val, strs.total_distance_str);
    lv_label_set_text_static(objects.dist_value, strs.distance_str);
    lv_label_set_text_static(objects.total_elev_val, strs.total_gain_str);
    lv_label_set_text_static(objects.elev_value, strs.elev_gain_str);
    lv_label_set_text_static(objects.temp_label, strs.temp_str);
    lv_label_set_text_static(objects.spd_label, strs.speed_str);
    lv_label_set_text_static(objects.avg_spd_label, strs.avg_speed_str);
}


/*********************
 * STATIC DECLARATONS
 **********************/
static void display_thread_fn(void *p1, void *p2, void *p3) {
    int32_t ret = 0;
    uint32_t delay = 0;
    struct display_stats stats;
    for (;;) {
        ret = k_msgq_get(&disp_q, &stats, K_MSEC(delay));
        if (ret) {
            // DO UPDATING OF LABELS HERE
            display_update_labels(&stats);
        }
        delay = lv_timer_handler();
        if (delay == LV_NO_TIMER_READY)
            delay = LV_DEF_REFR_PERIOD;
    }
}


static void display_update_labels(struct display_stats *stats) {
    if (active_stats.avg_cadence != stats->avg_cadence)
        snprintf(strs.avg_cadence_str, MAX_STR_STAT_LEN, "%d", (int32_t)stats->avg_cadence);
    if (active_stats.cadence != stats->cadence)
        snprintf(strs.cadence_str, MAX_STR_STAT_LEN, "%d", (int32_t)stats->cadence);
    if (active_stats.speed != stats->speed)
        snprintf(strs.speed_str, MAX_STR_STAT_LEN, "%d kmh", (int32_t)stats->speed);
    if (active_stats.avg_speed != stats->avg_speed)
        snprintf(strs.avg_speed_str, MAX_STR_STAT_LEN, "avg %d kmh", (int32_t)stats->avg_speed);
    if (active_stats.distance != stats->distance)
        snprintf(strs.distance_str, MAX_STR_STAT_LEN, "%d", (int32_t)stats->distance);
    if (active_stats.total_dist != stats->total_dist)
        snprintf(strs.total_distance_str, MAX_STR_STAT_LEN, "%d", (int32_t)stats->total_dist);
    if (active_stats.elevation != stats->elevation)
        snprintf(strs.elev_gain_str, MAX_STR_STAT_LEN, "%d", (int32_t)stats->elevation);
    if (active_stats.total_elevation != stats->total_elevation)
        snprintf(strs.total_gain_str, MAX_STR_STAT_LEN, "%d", (int32_t)stats->total_elevation);
    if (active_stats.temp != stats->temp)
        snprintf(strs.temp_str, MAX_STR_STAT_LEN, "%d C", (int32_t)stats->temp);
}

