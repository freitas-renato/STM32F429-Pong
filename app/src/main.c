#include <lvgl.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app/controller/touch.h"
#include "app/gui/views/homepage.h"
#include "app/gui/views/homepage_controller.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct gpio_dt_spec led_red    = GPIO_DT_SPEC_GET(DT_NODELABEL(led_red), gpios);
static const struct gpio_dt_spec led_yellow = GPIO_DT_SPEC_GET(DT_NODELABEL(led_yellow), gpios);

int main(void) {
    LOG_INF("Hello World from Zephyr!");

    touch_controller_init();

    if (!device_is_ready(led_red.port)) {
        LOG_ERR("Error: LED red device %s is not ready", led_red.port->name);
        return 1;
    }

    if (!device_is_ready(led_yellow.port)) {
        LOG_ERR("Error: LED yellow device %s is not ready", led_yellow.port->name);
        return 1;
    }

    gpio_pin_configure(led_red.port, led_red.pin, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure(led_yellow.port, led_yellow.pin, GPIO_OUTPUT_ACTIVE);

    /* Get the display device from the "chosen" node */
    const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(display_dev)) {
        printk("Display device not found!");
        return 1;
    }

    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);

    lv_timer_handler();
    display_blanking_off(display_dev);

    homepage_ui_create(NULL);
    homepage_controller_init();

    gpio_pin_toggle(led_red.port, led_red.pin);
    gpio_pin_toggle(led_yellow.port, led_yellow.pin);

    for (;;) {
        // LOG_INF("Hello World from Zephyr!");
        uint32_t ret = lv_timer_handler();
        k_sleep(K_MSEC(ret));
    }

    return 0;
}
