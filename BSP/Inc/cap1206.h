#ifndef __CAP1206_H__
#define __CAP1206_H__

#include <stdint.h>
#include <stdbool.h>

#include "i2c.h"


#define CAP_DEV_ADDR_7_BIT          0x28
#define CAP_DEV_ADDR_8_BIT          0x28 << 1u


/****************** CAP1206 REGISTERS ******************/
#define CAP_PRODUCT_ID              0xFD    /*<! Default value = 0x67 */
#define CAP_MANUFACTURER_ID         0xFE    /*<! Default value = 0x5D */
#define CAP_REVISION                0xFF    /*<! Default value = 0x00 */

#define CAP_MAIN_CONTROL            0x00    /*<! Main Control Register */
#define CAP_GEN_STATUS              0x02    /*<! General Status Register */
#define CAP_SENS_INPUT_STATUS       0x03    /*<! Sensor Input Status */

#define CAP_NOISE_FLAG_STATUS       0x0A    /*<! Noise Flag Status Register */

#define CAP_DELTA_INPUT_1           0x10    /*<! Delta count that is compared to threshold for touch detection */
#define CAP_DELTA_INPUT_2           0x11    /*<! Delta count that is compared to threshold for touch detection */
#define CAP_DELTA_INPUT_3           0x12    /*<! Delta count that is compared to threshold for touch detection */
#define CAP_DELTA_INPUT_4           0x13    /*<! Delta count that is compared to threshold for touch detection */
#define CAP_DELTA_INPUT_5           0x14    /*<! Delta count that is compared to threshold for touch detection */
#define CAP_DELTA_INPUT_6           0x15    /*<! Delta count that is compared to threshold for touch detection */

#define CAP_SENSITIVITY_CONTROL     0x1F    /*<! Controls sensitivity of touch detection */

#define CAP_CONFIGURATION_1         0x20    /*<! [TIMEOUT, X, DIS_DIG_NOISE, DIS_ANA_NOISE, MAX_DUR_EN, X, X, X] */
#define CAP_CONFIGURATION_2         0x44

#define CAP_SENS_INPUT_ENABLE       0x21    /*<! Sensor Input Enable Register */

#define CAP_SENS_INPUT_CONFIG_1     0x22    /*<! Sensor Input Configuration Register*/
#define CAP_SENS_INPUT_CONFIG_2     0x23    /*<! Sensor Input Configuration 2 Register */

#define CAP_MULTI_TOUCH_CFG         0x2A    /*<! Multiple Touch Configuration */
#define CAP_MTP_CFG                 0x2B    /*<! Multiple Touch Pattern Configuration */
#define CAP_MTP                     0x2D    /*<! Multiple Touch Pattern */
#define CAP_BC_OUT                  0x2E    /*<! BASE COUNT OUT OF LIMIT REGISTER */

#define CAP_SAMPLING_CFG            0x24    /*<! Averaging and Sampling Configuration */

#define CAP_CS_CAL                  0x26    /*<! CALIBRATION ACTIVATE AND STATUS REGISTER */

#define CAP_INTERRUPT_ENABLE        0x27    /*<! Enables interrupt generation on sensor pad touch/release */
#define CAP_REPEAT_RATE_ENABLE      0x28    /*<! Enables the repeat rate of the sensor inputs */

#define CAP_INPUT_THRESHOLD_1       0x30    /*<! Delta input is compared to this value for touch detection */
#define CAP_INPUT_THRESHOLD_2       0x31    /*<! Delta input is compared to this value for touch detection */
#define CAP_INPUT_THRESHOLD_3       0x32    /*<! Delta input is compared to this value for touch detection */
#define CAP_INPUT_THRESHOLD_4       0x33    /*<! Delta input is compared to this value for touch detection */
#define CAP_INPUT_THRESHOLD_5       0x34    /*<! Delta input is compared to this value for touch detection */
#define CAP_INPUT_THRESHOLD_6       0x35    /*<! Delta input is compared to this value for touch detection */

#define CAP_CAL_REGISTERS           0xB1

#define CAP_INPUT_BASE_COUNT        0x50    /*<! Base count registers for each sensor, 0x50 to 0x55 */

#define CAP_PWR_BTN_CFG             0x61    /*<! Power Button Configuration */


#define CAP_CHANNEL_1               0x01    /*<! Sensing channel 1 bit field */
#define CAP_CHANNEL_2               0x02    /*<! Sensing channel 2 bit field */
#define CAP_CHANNEL_3               0x04    /*<! Sensing channel 3 bit field */
#define CAP_CHANNEL_4               0x08    /*<! Sensing channel 4 bit field */
#define CAP_CHANNEL_5               0x10    /*<! Sensing channel 5 bit field */
#define CAP_CHANNEL_6               0x20    /*<! Sensing channel 6 bit field */
#define CAP_ALL_CHANNELS            0x3F    /*<! Represents all sensing channels */

typedef enum {
    CAP_OK      = 0x00,
    CAP_ERROR   = 0x01
} cap1206_ret_status_t;

typedef enum {
    CAP_SENS_128 = 0,  // Most sesitive setting
    CAP_SENS_64,
    CAP_SENS_32,
    CAP_SENS_16,
    CAP_SENS_8,
    CAP_SENS_4,
    CAP_SENS_2,
    CAP_SENS_1        // Least sensitive setting
} cap1206_sensitivity_t;

/** Determines the number of samples taken for all channels during the sensing cycle */
typedef enum {
    CAP_AVG_1 = 0,      // 1 sample
    CAP_AVG_2,          // 2 sample average
    CAP_AVG_4,          // 4 sample average
    CAP_AVG_8,          // 8 sample average  (default)
    CAP_AVG_16,         // 16 sample average
    CAP_AVG_32,         // 32 sample average
    CAP_AVG_64,         // 64 sample average
    CAP_AVG_128         // 128 sample average
} cap1206_samples_per_measurement_t;

/** Determines the time to take a single sample */
typedef enum {
    CAP_SAMP_TIME_320us = 0,  // 320us sample time
    CAP_SAMP_TIME_640us,      // 640us sample time
    CAP_SAMP_TIME_128ms,      // 1.28ms sample time (default)
    CAP_SAMP_TIME_256ms,      // 2.56ms sample time
} cap1206_sample_time_t;

/** Determines the sensing cycle time where samples are taken for all enabled channels */
typedef enum {
    CAP_CYCLE_TIME_35ms = 0,    // 35ms cycle time
    CAP_CYCLE_TIME_70ms,        // 70ms cycle time (default)
    CAP_CYCLE_TIME_105ms,       // 105ms cycle time
    CAP_CYCLE_TIME_140ms        // 140ms cycle time
} cap1206_cycle_time_t;

/** Determines the maximum time that a sensor pad is allowed to be touched until a recalibration is forced */
typedef enum {
    CAP_MAX_DUR_560ms = 0,
    CAP_MAX_DUR_840ms,
    CAP_MAX_DUR_1120ms,
    CAP_MAX_DUR_1400ms,
    CAP_MAX_DUR_1680ms,
    CAP_MAX_DUR_2240ms,
    CAP_MAX_DUR_2800ms,
    CAP_MAX_DUR_3360ms,
    CAP_MAX_DUR_3920ms,
    CAP_MAX_DUR_4480ms,
    CAP_MAX_DUR_5600ms,  // Default
    CAP_MAX_DUR_6720ms,
    CAP_MAX_DUR_7840ms,
    CAP_MAX_DUR_8906ms,
    CAP_MAX_DUR_10080ms,
    CAP_MAX_DUR_11200ms
} cap1206_touch_max_duration_t;

/** Determines the time between interrupt assertions when auto repeat is enabled */
typedef enum {
    CAP_RPT_RATE_35ms = 0,
    CAP_RPT_RATE_70ms,
    CAP_RPT_RATE_105ms,
    CAP_RPT_RATE_140ms,
    CAP_RPT_RATE_175ms,     // Default
    CAP_RPT_RATE_210ms,
    CAP_RPT_RATE_245ms,
    CAP_RPT_RATE_280ms,
    CAP_RPT_RATE_315ms,
    CAP_RPT_RATE_350ms,
    CAP_RPT_RATE_385ms,
    CAP_RPT_RATE_420ms,
    CAP_RPT_RATE_455ms,
    CAP_RPT_RATE_490ms,
    CAP_RPT_RATE_525ms,
    CAP_RPT_RATE_560ms
} cap1206_it_repeat_rate_t;



/**
 * @brief Main configuration struct
 * 
 */
typedef struct cap1206_config {
    I2C_HandleTypeDef*                  i2c_handle;         /*<! Pointer to STM32 HAL I2C handler */
    cap1206_sensitivity_t               sensitivity;
    cap1206_samples_per_measurement_t   avg_samples;
    cap1206_sample_time_t               sample_time;
    cap1206_cycle_time_t                cycle_time;

    uint8_t                             multitouch_enable;

    cap1206_touch_max_duration_t        max_touch_duration; /*<! Max touch/hold duration time until recalibration */
    bool                                it_repeat_enable;   /*<! Auto repeat enable */
    cap1206_it_repeat_rate_t            it_repeat_rate;

    cap1206_it_repeat_rate_t            hold_touch_detect;  /*<! Time to start detection of press and hold event (defaults to 280ms) */
} cap1206_config_t;

/**
 * @brief Initializes CAP1206 device with chosen configuration
 *
 * @param config Struct containing the configuration parameters
 * @return CAP_OK if configuration is successful, CAP_ERROR otherwise
 */
cap1206_ret_status_t cap1206_init(cap1206_config_t* config);


/**
 * @brief Enables specific sensor pad channels
 *
 * @param config    Pointer to device configuration struct
 * @param channels  Bitfield with chosen channels
 * @return CAP_OK if configuration is successful, CAP_ERROR otherwise
 */
cap1206_ret_status_t cap1206_channels_enable(cap1206_config_t* config, uint8_t channels);


/**
 * @brief Enables interrupt generation for each channel when sensor pad is touched
 *
 * @param config    Pointer to device configuration struct
 * @param channels  Bitfield with chosen channels
 * @return CAP_OK if configuration is successful, CAP_ERROR otherwise
 */
cap1206_ret_status_t cap1206_interrupt_enable(cap1206_config_t* config, uint8_t channels);


/**
 * @brief Enables generating interrupts for "press and hold" events, based on repeat rate defined on config struct.
 *
 * @param config    Pointer to device configuration struct
 * @param channels  Bitfield with chosen channels
 * @return CAP_OK if configuration is successful, CAP_ERROR otherwise
 */
cap1206_ret_status_t cap1206_repeat_enable(cap1206_config_t* config, uint8_t channels);


/**
 * @brief Reads touch status register to check which sensor pad is touched
 *
 * @param [in]  config     Pointer to device configuration struct
 * @param [out] touched    Touched channels.
 * @return CAP_OK if register read is successful, CAP_ERROR otherwise
 */
cap1206_ret_status_t cap1206_read_touch(cap1206_config_t* config, uint8_t* touched);


/**
 * @brief Sets sensor input threshold values. This value is used to detect touches.
 * A touch is detected when the delta input value exceeds the threshold.
 *
 * @param config                Pointer to device configuration struct
 * @param channel               Channel to be configured
 * @param threshold_multiplier  Threshold value
 * @return
 */
cap1206_ret_status_t cap1206_set_input_threshold(cap1206_config_t* config, uint8_t channel, uint8_t threshold);


/**
 * @brief Reads delta input registers for each channel.
 *
 * @param [in]  config   Pointer to device configuration struct
 * @param [out] input    Pointer to array where values will be stored
 * @return
 */
cap1206_ret_status_t cap1206_read_delta_input(cap1206_config_t* config, uint8_t input_values[6]);


/**
 * @brief Reads calibration registers for each channel.
 * Each calibration value is, in fact, a 10-bit value. The two MSB for each value is
 * present on values[6] and values[7], in order.
 *
 * @param [in]  config          Pointer to device configuration struct
 * @param [out] calib_values    Pointer to array where values will be stored
 * @return
 */
cap1206_ret_status_t cap1206_read_calibration_registers(cap1206_config_t* config, uint8_t calib_values[8]);


//? Sera que eu deixo isso aqui????
uint8_t cap1206_read_reg(uint8_t address);
uint8_t cap1206_write_reg(uint8_t address, uint8_t* data, uint8_t len);

#endif  // __CAP1206_H__
