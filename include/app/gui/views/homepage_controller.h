#ifndef HOMEPAGE_CONTROLLER_H
#define HOMEPAGE_CONTROLLER_H

#include <lvgl.h>

void homepage_controller_init(lv_obj_t* parent);

void homepage_controller_input(int player, int direction);

#endif  // HOMEPAGE_CONTROLLER_H
