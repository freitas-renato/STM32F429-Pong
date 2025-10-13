#ifndef COMMON_RTOS_H
#define COMMON_RTOS_H

#include <zephyr/kernel.h>

extern struct k_msgq player1_input_msgq;
extern struct k_msgq player2_input_msgq;

enum PlayerInput {
    PLAYER_INPUT_LEFT  = -1,
    PLAYER_INPUT_RIGHT = 1
};

#endif  // COMMON_RTOS_H