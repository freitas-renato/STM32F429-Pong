#include "app/common/rtos.h"

K_MSGQ_DEFINE(player1_input_msgq, sizeof(enum PlayerInput), 20, 4);
K_MSGQ_DEFINE(player2_input_msgq, sizeof(enum PlayerInput), 20, 4);