#include <lvgl.h>

#define PADDLE_WIDTH  15
#define PADDLE_HEIGHT 70
#define PADDLE_MARGIN 12

enum PaddleType {
    PADDLE_LEFT,
    PADDLE_RIGHT
};

enum PaddleMoveDirection {
    PADDLE_MOVE_RIGHT = -1,
    PADDLE_MOVE_NONE  = 0,
    PADDLE_MOVE_LEFT  = 1
};

lv_obj_t* paddle_create(lv_obj_t* parent, enum PaddleType type);

void paddle_move(lv_obj_t* paddle, enum PaddleMoveDirection direction);