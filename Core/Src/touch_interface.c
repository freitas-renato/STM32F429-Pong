#include "touch_interface.h"

#include "i2c.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "buzzer.h"

#define I2C_MAX_TIMEOUT     5000u
#define DEBOUNCE_MAX_TIME   2

cap1206_config_t cap_config = {
        .i2c_handle = &hi2c3,
        .sensitivity = CAP_SENS_128,
        .avg_samples = CAP_AVG_8,
        .sample_time = CAP_SAMP_TIME_128ms,
        .cycle_time = CAP_CYCLE_TIME_105ms,
        .multitouch_enable = 0,
        .max_touch_duration = CAP_MAX_DUR_11200ms,
        .it_repeat_enable = 1,
        .it_repeat_rate = CAP_RPT_RATE_70ms,
        .hold_touch_detect = CAP_RPT_RATE_210ms
};

/** Queue where touch events are handed over to TouchGFX's Model interface*/
extern QueueHandle_t button_queue;

/** Semaphore used to sync I2C calls */
SemaphoreHandle_t i2c_semaphore = NULL;

/** Used as a counter to differentiate single touches and "press and hold" events */
static volatile int _debounce_time = 0;

/** Button event struct where touch characteristics will be stored*/
static button_event_t touch_event;



/******* Public functions *********/

/**
 * @brief CAP1206 initialization task. 
 * 
 * @param arguments 
 */
void CAP_Task(void* arguments) {
    cap1206_ret_status_t ret;
    HAL_StatusTypeDef hal_i2c_status;
    
    // Create and release i2c semaphore
    i2c_semaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(i2c_semaphore);

    //! Halt program until i2c is ready
    //! this should be removed, otherwise our firmware won't start
    while ((hal_i2c_status = HAL_I2C_IsDeviceReady(cap_config.i2c_handle, CAP_DEV_ADDR_8_BIT, 3u, I2C_MAX_TIMEOUT)) != HAL_OK) {
        vTaskDelay(20);
    }

    ret = cap1206_init(&cap_config);
    while (ret == CAP_ERROR) {
        // Retry configuration
        ret = cap1206_init(&cap_config);
        vTaskDelay(500);
    }


    // Enables channels 1~4
    cap1206_channels_enable(&cap_config, CAP_CHANNEL_1 | CAP_CHANNEL_2 | CAP_CHANNEL_3 | CAP_CHANNEL_4);
    vTaskDelay(50);

    // Set input theshold to 32 only on channel 4
    /** @TODO: calibrate for all channels with the final electrode */
    cap1206_set_input_threshold(&cap_config, CAP_CHANNEL_1, 0x10);  //! Temporary values
    cap1206_set_input_threshold(&cap_config, CAP_CHANNEL_2, 0x10);  //! Temporary values
    cap1206_set_input_threshold(&cap_config, CAP_CHANNEL_3, 0x20);  //! Temporary values
    cap1206_set_input_threshold(&cap_config, CAP_CHANNEL_4, 0x20);  //! Temporary values

    buzzer_init();
    // buzzer_1up();

    /** @NOTE: at this point, all our configurations should be complete and ready to go */
    vTaskDelay(300);


    for (;;) {
        //! Suspend task forever, or maybe just let it exit since we will be working on interrupts
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1000);
        vTaskSuspend(NULL);
        vTaskDelay(1000);
    }
}


void process_touch_event(void *pvParameter1, uint32_t ulParameter2) {
    uint8_t cap_touch_status = 0;
    touch_event.isLongPress = false;
    
    // Try to take control of i2c bus
    if (xSemaphoreTake(i2c_semaphore, (TickType_t)10) == pdTRUE) {
        // Check if I2C bus is available
        if (HAL_I2C_IsDeviceReady(cap_config.i2c_handle, CAP_DEV_ADDR_8_BIT, 2u, 2000u) != HAL_OK) {
            xSemaphoreGive(i2c_semaphore);
            return;
        }
    
        // Read touch registers
        if (cap1206_read_touch(&cap_config, &cap_touch_status) == CAP_ERROR) {
            xSemaphoreGive(i2c_semaphore);
            return;
        }

        if (cap_touch_status & CAP_CHANNEL_1) {
            touch_event.button_id = 1;
            touch_event.button_state = 1;
            xQueueSend(button_queue, &touch_event, 0);
        }

        if (cap_touch_status & CAP_CHANNEL_2) {
            touch_event.button_id = 0;
            touch_event.button_state = 1;
            xQueueSend(button_queue, &touch_event, 0);
        }

        if (cap_touch_status & CAP_CHANNEL_3) {
            touch_event.button_id = 2;
            touch_event.button_state = 1;
            xQueueSend(button_queue, &touch_event, 0);
        }

        if (cap_touch_status & CAP_CHANNEL_4) {
            touch_event.button_id = 3;
            touch_event.button_state = 1;
            xQueueSend(button_queue, &touch_event, 0);
        }

        // Finally, send event to touchgfx and release semaphore
        xSemaphoreGive(i2c_semaphore);
    }
}


