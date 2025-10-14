#ifndef APP_GUI_COMPONENTS_BALL_H
#define APP_GUI_COMPONENTS_BALL_H

#include <lvgl.h>

lv_obj_t* ball_create(lv_obj_t* parent, int radius);

void ball_set_position(lv_obj_t* ball, int32_t x, int32_t y);

void ball_move(lv_obj_t* ball, int32_t dx, int32_t dy);

#endif  // APP_GUI_COMPONENTS_BALL_H