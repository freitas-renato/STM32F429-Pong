#include "app/gui/components/ball.h"

lv_obj_t* ball_create(lv_obj_t* parent, int radius) {
    if (parent == NULL) {
        parent = lv_screen_active();
    }

    lv_obj_t* ball = lv_obj_create(parent);

    lv_obj_remove_flag(ball, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(ball, LV_RADIUS_CIRCLE, 0);  // Make ball circular
    lv_obj_set_size(ball, radius * 2, radius * 2);
    lv_obj_set_style_border_width(ball, 0, 0);
    lv_obj_set_style_bg_color(ball, lv_color_white(), 0);
    lv_obj_align(ball, LV_ALIGN_CENTER, 0, 0);

    return ball;
}

void ball_set_position(lv_obj_t* ball, int32_t x, int32_t y) {
    if (ball == NULL) {
        return;
    }

    lv_obj_set_pos(ball, x, y);
}

void ball_move(lv_obj_t* ball, int32_t dx, int32_t dy) {
    if (ball == NULL) {
        return;
    }

    int32_t current_x = lv_obj_get_x(ball);
    int32_t current_y = lv_obj_get_y(ball);
    lv_obj_set_pos(ball, current_x + dx, current_y + dy);
}
