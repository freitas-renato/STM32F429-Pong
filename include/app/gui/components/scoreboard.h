#include <lvgl.h>

#include <stdbool.h>

void scoreboard_create(lv_obj_t* parent);

void scoreboard_update(lv_obj_t* parent, int score_left, int score_right);

void scoreboard_reset(lv_obj_t* parent);

void scoreboard_show(lv_obj_t* parent, bool show);