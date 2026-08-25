#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_MAIN = 1,
    _SCREEN_ID_LAST = 1
};

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *elev_contain;
    lv_obj_t *elev_value;
    lv_obj_t *elev_label;
    lv_obj_t *total_elev_label;
    lv_obj_t *total_elev_val;
    lv_obj_t *cad_contain;
    lv_obj_t *cad_value;
    lv_obj_t *cad_label;
    lv_obj_t *avg_cad_label;
    lv_obj_t *avg_cad_val;
    lv_obj_t *dist_contain;
    lv_obj_t *dist_value;
    lv_obj_t *dist_label;
    lv_obj_t *ttl_dist_label;
    lv_obj_t *ttl_dist_val;
    lv_obj_t *sw_label;
    lv_obj_t *temp_label;
    lv_obj_t *spd_label;
    lv_obj_t *avg_spd_label;
} objects_t;

extern objects_t objects;

void create_screen_main();
void tick_screen_main();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/