#include "app/gui/views/homepage_controller.h"

#include <lvgl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app/gui/views/homepage.h"

LOG_MODULE_REGISTER(homepage_controller, LOG_LEVEL_WRN);

static struct k_work_delayable homepage_work;

static int8_t ball_v_direction = 1;  // 1 for right, -1 for left
static int8_t ball_h_direction = 1;  // 1 for down, -1 for up
static int game_state          = 0;  // 0 = not started, 1 = running

static int player1_score = 0;
static int player2_score = 0;

void homepage_update_work_handler() {
    if (game_state == 0) {
        // Game not started, do nothing
        return;
    }

    lv_coord_t screen_width = lv_disp_get_hor_res(NULL);
    int ball_x              = homepage_ui_get_ball_x_position();

    if (homepage_ui_is_ball_touching_top()) {
        ball_v_direction = 1;  // Move down
    } else if (homepage_ui_is_ball_touching_bottom()) {
        ball_v_direction = -1;  // Move up
    }

    // Check for goals
    if (ball_x <= 0) {
        // hit left wall, goal for player 2
        LOG_INF("Goal for Player 2");
        player2_score++;
        game_state = 0;

        ball_h_direction = -ball_h_direction;  // player that suffered goal serves next
        homepage_ui_update_score(player1_score, player2_score);
        homepage_ui_ball_reset_position();
        return;
    }

    if ((ball_x) >= screen_width) {
        // hit right wall, goal for player 1
        LOG_INF("Goal for Player 1");
        player1_score++;
        game_state = 0;

        ball_h_direction = -ball_h_direction;  // player that suffered goal serves next
        homepage_ui_update_score(player1_score, player2_score);
        homepage_ui_ball_reset_position();
        return;
    }

    if (homepage_ui_is_ball_touching_paddle_left()) {
        ball_h_direction = 1;  // Reverse horizontal direction
        LOG_WRN("Ball hit left paddle");
    } else if (homepage_ui_is_ball_touching_paddle_right()) {
        ball_h_direction = -1;  // Reverse horizontal direction
        LOG_WRN("Ball hit right paddle");
    }

    homepage_ui_update_ball_position(ball_h_direction, ball_v_direction);

    k_work_schedule(&homepage_work, K_MSEC(20));  // Reschedule after 20 milliseconds
}

void homepage_controller_init(lv_obj_t* parent) {
    if (parent == NULL) {
        parent = lv_screen_active();
    }

    // lv_obj_set_style_bg_color(parent, lv_color_hex(0x003a57), LV_PART_MAIN);
    lv_obj_set_style_bg_color(parent, lv_color_black(), LV_PART_MAIN);

    homepage_ui_create(parent);
    homepage_ui_update_score(0, 0);

    k_work_init_delayable(&homepage_work, homepage_update_work_handler);
    k_work_schedule(&homepage_work, K_MSEC(1000));  // Schedule first run after 1 second
}

void homepage_controller_input(int player, int direction) {
    if (game_state == 0) {
        game_state = 1;  // Start the game on first input
        k_work_schedule(&homepage_work, K_MSEC(100));
    }

    if (player == 1) {
        // Move left paddle
        left_paddle_move(direction);
    } else if (player == 2) {
        // Move right paddle
        right_paddle_move(direction);
    }
}
