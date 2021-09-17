#include "buzzer.h"

#include "FreeRTOS.h"
#include "task.h"

#include "tim.h"


#define TIM_FREQUENCY_HZ 2000000U  /**< This is the internal clock divided by the prescaler */
#define MAX_FREQUENCY_HZ 20000U    /**< Maximum buzzer frequency */
#define MIN_FREQUENCY_HZ 20U       /**< Minimum buzzer frequency */


static float m_volume;
static uint32_t m_counter;


void buzzer_init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    m_volume = 0.8;
    buzzer_set_frequency(1000);
}

void buzzer_set_volume(float volume) {
    m_volume = volume;
    uint32_t compare = (uint32_t) ((float) m_counter * m_volume);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
}

void buzzer_set_frequency(uint16_t frequency) {
    if (frequency == 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        return;
    }

    frequency = (frequency < MIN_FREQUENCY_HZ ? MIN_FREQUENCY_HZ : (frequency > MAX_FREQUENCY_HZ ? MAX_FREQUENCY_HZ : frequency));
    m_counter = TIM_FREQUENCY_HZ / frequency;
    uint32_t compare = (uint32_t) ((float) m_counter * m_volume);

    __HAL_TIM_SET_AUTORELOAD(&htim3, m_counter);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
}

void buzzer_start(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void buzzer_stop(void) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

void buzzer_click(void) {
    buzzer_start();
    buzzer_set_frequency(4000);
    vTaskDelay(2);
    buzzer_stop();
}


/*********************/
/***** Fun stuff *****/
/*********************/


void buzzer_1up(void) {
    buzzer_set_frequency(NOTE_E6);
    vTaskDelay(130 / portTICK_RATE_MS);
    buzzer_set_frequency(NOTE_G6);
    vTaskDelay(130 / portTICK_RATE_MS);
    buzzer_set_frequency(NOTE_E7);
    vTaskDelay(130 / portTICK_RATE_MS);
    buzzer_set_frequency(NOTE_C7);
    vTaskDelay(130 / portTICK_RATE_MS);
    buzzer_set_frequency(NOTE_D7);
    vTaskDelay(130 / portTICK_RATE_MS);
    buzzer_set_frequency(NOTE_G7);
    vTaskDelay(125 / portTICK_RATE_MS);
    buzzer_stop();
}

void buzzer_fireball(void) {
    // Play Fireball sound
    buzzer_set_frequency(NOTE_G4);
    vTaskDelay(100 / portTICK_RATE_MS);
    buzzer_set_frequency(NOTE_G5);
    vTaskDelay(100 / portTICK_RATE_MS);
    buzzer_set_frequency(NOTE_G6);
    vTaskDelay(100 / portTICK_RATE_MS);
    buzzer_stop();
}
