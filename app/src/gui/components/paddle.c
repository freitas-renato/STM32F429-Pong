#include "app/gui/components/paddle.h"

static enum PaddleType m_paddle_type = PADDLE_LEFT;
static const int PADDLE_MOVE_STEP  = 5;  // Pixels to move per input

lv_obj_t* paddle_create(lv_obj_t* parent, enum PaddleType type) {
    if (parent == NULL) {
        parent = lv_screen_active();
    }

    lv_obj_t* paddle = lv_obj_create(parent);

    lv_obj_remove_flag(paddle, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(paddle, 8, 0);  // Make paddle rectangular
    lv_obj_set_size(paddle, PADDLE_WIDTH, PADDLE_HEIGHT);
    lv_obj_set_style_border_width(paddle, 0, 0);

    m_paddle_type = type;

    lv_align_t align = type == PADDLE_LEFT ? LV_ALIGN_LEFT_MID : LV_ALIGN_RIGHT_MID;
    int margin       = type == PADDLE_LEFT ? PADDLE_MARGIN : -PADDLE_MARGIN;
    lv_color_t color = type == PADDLE_LEFT ? lv_palette_main(LV_PALETTE_RED) : lv_palette_main(LV_PALETTE_BLUE);

    lv_obj_set_style_bg_color(paddle, color, 0);
    lv_obj_align(paddle, align, margin, 0);

    return paddle;
}

//?
void paddle_set_position(lv_obj_t* paddle, int32_t y) {
    if (paddle == NULL) {
        return;
    }

    lv_obj_set_y(paddle, y);
}

void paddle_move(lv_obj_t* paddle, enum PaddleMoveDirection direction) {
    if (paddle == NULL || direction == PADDLE_MOVE_NONE) {
        return;
    }

    if (m_paddle_type == PADDLE_RIGHT) {
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
