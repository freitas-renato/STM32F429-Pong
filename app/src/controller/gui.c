#include "app/controller/touch.h"
#include "app/common/rtos.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "app/gui/views/homepage_controller.h"

LOG_MODULE_REGISTER(gui_controller, LOG_LEVEL_DBG);

void gui_controller_thread(void* arg1, void* arg2, void* arg3) {
    while (1) {
        enum PlayerInput input;

        // GUI update logic here
        if (k_msgq_get(&player1_input_msgq, &input, K_NO_WAIT) == 0) {
            // Process input for player 1
            if (input == PLAYER_INPUT_LEFT) {
                // Move player 1 paddle left
                LOG_INF("Player 1 input LEFT");
            } else if (input == PLAYER_INPUT_RIGHT) {
                // Move player 1 paddle right
                LOG_INF("Player 1 input RIGHT");
            }
        }

        if (k_msgq_get(&player2_input_msgq, &input, K_NO_WAIT) == 0) {
            // Process input for player 2
            if (input == PLAYER_INPUT_LEFT) {
                // Move player 2 paddle left
                LOG_INF("Player 2 input LEFT");
            } else if (input == PLAYER_INPUT_RIGHT) {
                // Move player 2 paddle right
                LOG_INF("Player 2 input RIGHT");
            }
        }
        k_sleep(K_MSEC(100));
    }
}

K_THREAD_DEFINE(gui_controller_thread_id, 1024, gui_controller_thread, NULL, NULL, NULL, 7, 0, 0);
