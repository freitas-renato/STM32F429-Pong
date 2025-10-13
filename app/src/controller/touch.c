#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include "app/common/rtos.h"

LOG_MODULE_REGISTER(touch_controller, LOG_LEVEL_INF);

static const struct device *cap_dev_left  = DEVICE_DT_GET(DT_NODELABEL(touch_sensor_left));
static const struct device *cap_dev_right = DEVICE_DT_GET(DT_NODELABEL(touch_sensor_right));

static void left_touch_input_cb(struct input_event *evt, void *user_data) {
    LOG_DBG("Input event LEFT: type=%d, code=%d, value=%d\n", evt->type, evt->code, evt->value);

    switch (evt->code) {
        case INPUT_KEY_1:  // Button 0
            if (evt->value == 1) {
                enum PlayerInput input = PLAYER_INPUT_RIGHT;
                k_msgq_put(&player1_input_msgq, &input, K_NO_WAIT);
            }
            break;
        case INPUT_KEY_2:  // Button 1
            if (evt->value == 1) {
                enum PlayerInput input = PLAYER_INPUT_LEFT;
                k_msgq_put(&player1_input_msgq, &input, K_NO_WAIT);
            }
            break;
        default:
            break;
    }
}

static void right_touch_input_cb(struct input_event *evt, void *user_data) {
    LOG_DBG("Input event RIGHT: type=%d, code=%d, value=%d\n", evt->type, evt->code, evt->value);
    switch (evt->code) {
        case INPUT_KEY_3:  // Button 0
            if (evt->value == 1) {
                enum PlayerInput input = PLAYER_INPUT_RIGHT;
                k_msgq_put(&player2_input_msgq, &input, K_NO_WAIT);
            }
            break;
        case INPUT_KEY_4:  // Button 1
            if (evt->value == 1) {
                enum PlayerInput input = PLAYER_INPUT_LEFT;
                k_msgq_put(&player2_input_msgq, &input, K_NO_WAIT);
            }
            break;
        default:
            break;
    }
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(touch_sensor_left)), left_touch_input_cb, NULL);
INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(touch_sensor_right)), right_touch_input_cb, NULL);

int touch_controller_init(void) {
    if (!device_is_ready(cap_dev_left)) {
        LOG_ERR("CAP12xx left device not ready.\n");
        return -1;
    }

    if (!device_is_ready(cap_dev_right)) {
        LOG_ERR("CAP12xx right device not ready.\n");
        return -1;
    }

    LOG_INF("CAP12xx devices are ready.\n");

    return 0;
}