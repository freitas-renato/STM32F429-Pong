#include "app/gui/components/paddle.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(paddle_component, LOG_LEVEL_INF);

static const int PADDLE_MOVE_STEP = 15;  // Pixels to move per input

lv_obj_t* paddle_create(lv_obj_t* parent, enum PaddleType type) {
    if (parent == NULL) {
        parent = lv_screen_active();
    }

    lv_obj_t* paddle = lv_obj_create(parent);

    lv_obj_remove_flag(paddle, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(paddle, 8, 0);  // Make paddle rectangular
    lv_obj_set_size(paddle, PADDLE_WIDTH, PADDLE_HEIGHT);
    lv_obj_set_style_border_width(paddle, 0, 0);

    lv_color_t color = type == PADDLE_TYPE_LEFT ? lv_palette_main(LV_PALETTE_RED) : lv_palette_main(LV_PALETTE_BLUE);

    int paddle_x =
        type == PADDLE_TYPE_LEFT ? PADDLE_MARGIN : (lv_disp_get_hor_res(NULL) - PADDLE_WIDTH - PADDLE_MARGIN);
    int paddle_y      = (lv_disp_get_ver_res(NULL) - PADDLE_HEIGHT) / 2;
    int screen_height = lv_disp_get_ver_res(NULL);

    lv_obj_set_style_bg_color(paddle, color, 0);

    lv_obj_set_x(paddle, paddle_x);
    lv_obj_set_y(paddle, paddle_y);

    lv_obj_update_layout(paddle);
    LOG_INF(
        "Paddle created: type=%s, position=(%d, %d), screen_height=%d", type == PADDLE_TYPE_LEFT ? "LEFT" : "RIGHT",
        lv_obj_get_x(paddle), lv_obj_get_y(paddle), screen_height
    );

    return paddle;
}

//?
void paddle_set_position(lv_obj_t* paddle, int32_t y) {
    if (paddle == NULL) {
        return;
    }

    lv_obj_set_y(paddle, y);
}

void paddle_move(lv_obj_t* paddle, enum PaddleType type, enum PaddleMoveDirection direction) {
    if (paddle == NULL || direction == PADDLE_MOVE_NONE) {
        return;
    }

    if (type == PADDLE_TYPE_RIGHT) {
        direction = -direction;  // Invert direction for right paddle
    }

    int32_t current_y = lv_obj_get_y(paddle);
    int32_t new_y     = current_y + (direction * PADDLE_MOVE_STEP);

    // Ensure the paddle stays within screen bounds
    lv_coord_t screen_height = lv_disp_get_ver_res(NULL);
    if (new_y < 0) {
        new_y = 0;
    } else if (new_y + PADDLE_HEIGHT > screen_height) {
        new_y = screen_height - PADDLE_HEIGHT;
    }

    lv_obj_set_y(paddle, new_y);
}
