// #include <lvgl.h>
#include "app/gui/views/homepage.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app/gui/components/ball.h"
#include "app/gui/components/paddle.h"
#include "app/gui/components/scoreboard.h"


LOG_MODULE_REGISTER(homepage_view, LOG_LEVEL_INF);

// --- Constants for game elements ---
#define PADDLE_WIDTH  15
#define PADDLE_HEIGHT 70
#define PADDLE_MARGIN 12
#define BALL_SIZE     15

// --- Static pointers to LVGL objects ---
static lv_obj_t* paddle_left;
static lv_obj_t* paddle_right;
static lv_obj_t* ball;
static lv_obj_t* net_line;

/**
 * @brief Creates all the UI components for the Pong game screen.
 * @param parent The parent object to which the game screen will be added.
 * If NULL, the current active screen will be used.
 */
void homepage_ui_create(lv_obj_t* parent) {
    if (parent == NULL) {
        parent = lv_screen_active();
    }

    // Get screen dimensions
    lv_coord_t screen_width  = lv_disp_get_hor_res(NULL);
    lv_coord_t screen_height = lv_disp_get_ver_res(NULL);

    LOG_INF("Screen dimensions: %dx%d", screen_width, screen_height);

    paddle_left  = paddle_create(parent, PADDLE_LEFT);
    paddle_right = paddle_create(parent, PADDLE_RIGHT);

    ball = ball_create(parent, BALL_SIZE / 2);

    scoreboard_create(parent);
    scoreboard_update(parent, 1, 0);
    scoreboard_show(parent, true);
}

void homepage_ui_update_ball_position(int32_t x, int32_t y) {
    // ball_set_position(ball, x, y);
    lv_obj_set_pos(ball, x, y);
}
