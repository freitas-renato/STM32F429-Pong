// #include <lvgl.h>
#include "app/gui/views/homepage.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app/gui/components/ball.h"
#include "app/gui/components/paddle.h"
#include "app/gui/components/scoreboard.h"

LOG_MODULE_REGISTER(homepage_view, LOG_LEVEL_INF);

#define BALL_SIZE  12
#define BALL_SPEED 3

static lv_obj_t* paddle_left;
static lv_obj_t* paddle_right;
static lv_obj_t* ball;
static lv_obj_t* net_line;  // todo: add net line in the middle of the screen

void homepage_ui_create(lv_obj_t* parent) {
    if (parent == NULL) {
        parent = lv_screen_active();
    }

    // Get screen dimensions
    lv_coord_t screen_width  = lv_disp_get_hor_res(NULL);
    lv_coord_t screen_height = lv_disp_get_ver_res(NULL);

    LOG_INF("Screen dimensions: %dx%d", screen_width, screen_height);

    paddle_left  = paddle_create(parent, PADDLE_TYPE_LEFT);
    paddle_right = paddle_create(parent, PADDLE_TYPE_RIGHT);

    ball = ball_create(parent, BALL_SIZE / 2);

    scoreboard_create(parent);
    scoreboard_update(parent, 0, 0);
    scoreboard_show(parent, true);
}

void homepage_ui_update_ball_position(int8_t h_dir, int8_t v_dir) {
    if (ball == NULL) {
        LOG_ERR("Ball object is NULL");
        return;
    }

    ball_move(ball, h_dir * BALL_SPEED, v_dir * BALL_SPEED);
}

int homepage_ui_get_ball_x_position() {
    return lv_obj_get_x(ball);
}

int homepage_ui_get_ball_y_position() {
    return lv_obj_get_y(ball);
}

void homepage_ui_ball_reset_position() {
    if (ball == NULL) {
        LOG_ERR("Ball object is NULL");
        return;
    }

    lv_coord_t screen_width  = lv_disp_get_hor_res(NULL);
    lv_coord_t screen_height = lv_disp_get_ver_res(NULL);

    int start_x = (screen_width - BALL_SIZE) / 2;
    int start_y = (screen_height - BALL_SIZE) / 2;

    ball_set_position(ball, start_x, start_y);
}

int homepage_ui_get_left_paddle_y_position(void) {
    if (paddle_left == NULL) {
        LOG_ERR("Left paddle object is NULL");
        return -1;
    }

    return lv_obj_get_y(paddle_left);
}

int homepage_ui_get_left_paddle_x_position(void) {
    if (paddle_left == NULL) {
        LOG_ERR("Left paddle object is NULL");
        return -1;
    }

    return lv_obj_get_x(paddle_left);
}

int homepage_ui_get_right_paddle_y_position(void) {
    if (paddle_right == NULL) {
        LOG_ERR("Right paddle object is NULL");
        return -1;
    }

    return lv_obj_get_y(paddle_right);
}

int homepage_ui_get_right_paddle_x_position(void) {
    if (paddle_right == NULL) {
        LOG_ERR("Right paddle object is NULL");
        return -1;
    }

    return lv_obj_get_x(paddle_right);
}

void left_paddle_move(int direction) {
    if (paddle_left == NULL) {
        LOG_ERR("Left paddle object is NULL");
        return;
    }

    paddle_move(paddle_left, PADDLE_TYPE_LEFT, direction);
}

void right_paddle_move(int direction) {
    if (paddle_right == NULL) {
        LOG_ERR("Right paddle object is NULL");
        return;
    }

    paddle_move(paddle_right, PADDLE_TYPE_RIGHT, direction);
}

bool homepage_ui_is_ball_touching_paddle_left() {
    int left_paddle_x = homepage_ui_get_left_paddle_x_position();
    int left_paddle_y = homepage_ui_get_left_paddle_y_position();
    int ball_x        = homepage_ui_get_ball_x_position();
    int ball_y        = homepage_ui_get_ball_y_position();

    return (
        ball_x < (left_paddle_x + PADDLE_WIDTH) && ball_x > left_paddle_x && (ball_y + BALL_SIZE) > left_paddle_y &&
        ball_y < (left_paddle_y + PADDLE_HEIGHT)
    );
}

bool homepage_ui_is_ball_touching_paddle_right() {
    int right_paddle_x = homepage_ui_get_right_paddle_x_position();
    int right_paddle_y = homepage_ui_get_right_paddle_y_position();
    int ball_x         = homepage_ui_get_ball_x_position();
    int ball_y         = homepage_ui_get_ball_y_position();

    return (
        (ball_x + BALL_SIZE) > (right_paddle_x) && (ball_x + BALL_SIZE) < (right_paddle_x + PADDLE_WIDTH) &&
        (ball_y + BALL_SIZE) > right_paddle_y && ball_y < (right_paddle_y + PADDLE_HEIGHT)
    );
}

bool homepage_ui_is_ball_touching_top() {
    int ball_y = homepage_ui_get_ball_y_position();

    return (ball_y <= 0);
}

bool homepage_ui_is_ball_touching_bottom() {
    int ball_y = homepage_ui_get_ball_y_position();

    return ((ball_y + BALL_SIZE) >= lv_disp_get_ver_res(NULL));
}

void homepage_ui_update_score(int player1, int player2) {
    scoreboard_update(lv_screen_active(), player1, player2);
}
