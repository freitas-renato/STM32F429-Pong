#include "app/gui/views/homepage_controller.h"

#include <lvgl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app/gui/views/homepage.h"

LOG_MODULE_REGISTER(homepage_controller, LOG_LEVEL_INF);


static struct k_work_delayable homepage_work;

static int ball_position[2] = {0, 0};
static int ball_velocity[2] = {2, 2};  // Pixels per update
static int ball_direction   = 1;       // 1 for right, -1 for left

void homepage_update_work_handler() {
    // check if ball potisition hits screen bounds

    // up and down first
    lv_coord_t screen_height = lv_disp_get_ver_res(NULL);
    if (ball_position[1] <= 0 || ball_position[1] >= screen_height) {
        ball_velocity[1] = -ball_velocity[1];  // Reverse vertical direction
    }

    lv_coord_t screen_width = lv_disp_get_hor_res(NULL);
    if (ball_position[0] <= 0) {
        // hit left wall, goal for player 2
        LOG_INF("Goal for Player 2");
        ball_position[0] = screen_width / 2;
        ball_position[1] = screen_height / 2;
        k_work_schedule(&homepage_work, K_MSEC(100));  // Reschedule after 100 milliseconds
        return;
    } 
    
    if (ball_position[0] >= screen_width) {
        // hit right wall, goal for player 1
        LOG_INF("Goal for Player 1");
        ball_position[0] = screen_width / 2;
        ball_position[1] = screen_height / 2;
        k_work_schedule(&homepage_work, K_MSEC(100));  // Reschedule after 100 milliseconds
        return;
    }

    // check if ball hits paddles

    // Update ball position
    ball_position[0] += ball_velocity[0] * ball_direction;
    ball_position[1] += ball_velocity[1] * ball_direction;
    LOG_INF("Ball position: (%d, %d)", ball_position[0], ball_position[1]);

    homepage_ui_update_ball_position(ball_position[0], ball_position[1]);
    // Reschedule the work
    k_work_schedule(&homepage_work, K_MSEC(100));  // Reschedule after 100 milliseconds
}

void homepage_controller_init() {
    ball_position[0] = lv_disp_get_hor_res(NULL) / 2;
    ball_position[1] = lv_disp_get_ver_res(NULL) / 2;
    k_work_init_delayable(&homepage_work, homepage_update_work_handler);
    k_work_schedule(&homepage_work, K_MSEC(1000));  // Schedule first run after 1 second
}
