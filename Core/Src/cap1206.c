#include "cap1206.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "i2c.h"

#include "gpio.h"

static const uint16_t I2C_MAX_TIMEOUT = 5000u;


cap1206_ret_status_t cap1206_init(cap1206_config_t* config) {
    uint8_t _chip_id;

    HAL_I2C_Mem_Read(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_MANUFACTURER_ID, 1, &_chip_id, 1, I2C_MAX_TIMEOUT);
    // if (_chip_id != 0x5D) {
    //     return CAP_ERROR;
    // }

    vTaskDelay(50);

    // Write sensitivity data
    uint8_t _sensitivity_control = (config->sensitivity << 4) | 0x0F;  // Default BASE_SHIFT (1111b)
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_SENSITIVITY_CONTROL, 1, &_sensitivity_control, 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }
    vTaskDelay(50);

    // Write averaging and sampling configuration data
    uint8_t _sampling_control = (config->avg_samples << 4 & 0b01110000) | (config->sample_time << 2 & 0b00001100) | (config->cycle_time & 0b11);
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_SENSITIVITY_CONTROL, 1, &_sampling_control, 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }
    vTaskDelay(50);

    // Multitouch enable/disable
    uint8_t _multitouch = (config->multitouch_enable << 7);
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_MULTI_TOUCH_CFG, 1, &_multitouch , 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }
    vTaskDelay(50);

    // TODO: Check auto repeat enable (configuration register 0x20, 0x44)
    uint8_t _configuration_2 = 0b01100100;  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! funcionou com essa config mas eu n entendi porque ainda
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_CONFIGURATION_2, 1, &_configuration_2 , 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }

    if (config->it_repeat_enable == true) {
        cap1206_repeat_enable(config, CAP_CHANNEL_1 | CAP_CHANNEL_2 | CAP_CHANNEL_3 | CAP_CHANNEL_4);
    } else {
        cap1206_repeat_enable(config, 0x00);  // Disable repeating interrupts for "press and hold"
    }

    // Sensor input configuration registers
    uint8_t _input_config = (config->max_touch_duration << 4) | (config->it_repeat_rate & 0x0F);
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_SENS_INPUT_CONFIG_1, 1, &_input_config , 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }
    vTaskDelay(50);
    _input_config = config->hold_touch_detect & 0x0F;
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_SENS_INPUT_CONFIG_2, 1, &_input_config , 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }
    vTaskDelay(50);


    return CAP_OK;
}

cap1206_ret_status_t cap1206_channels_enable(cap1206_config_t* config, uint8_t channels) {
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_SENS_INPUT_ENABLE, 1, &channels , 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }

    return CAP_OK;
}

cap1206_ret_status_t cap1206_interrupt_enable(cap1206_config_t* config, uint8_t channels) {
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_INTERRUPT_ENABLE, 1, &channels , 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }

    return CAP_OK;
}

cap1206_ret_status_t cap1206_repeat_enable(cap1206_config_t* config, uint8_t channels) {
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_REPEAT_RATE_ENABLE, 1, &channels , 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }

    return CAP_OK;
}

cap1206_ret_status_t cap1206_read_touch(cap1206_config_t* config, uint8_t* touched) {
    uint8_t main_control_reg = 0x00;

    // Clear INT flag on main control register
    HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_MAIN_CONTROL, 1, &main_control_reg, 1, 200);

    if (HAL_I2C_Mem_Read(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_SENS_INPUT_STATUS, 1, touched, 1, 200) != HAL_OK) {
        return CAP_ERROR;
    }

    // // Clear touch flag on main control register
    // if (*touched > 0 && *touched != 0xFF) {
    //     // main_control_reg = cap1206_read_reg(CAP_MAIN_CONTROL);
    //     // HAL_I2C_Mem_Read(&hi2c3, CAP_DEV_ADDR_8_BIT, CAP_MAIN_CONTROL, 1, &main_control_reg, 1, I2C_MAX_TIMEOUT);
    //     // main_control_reg = main_control_reg & 0x00;

    //     // cap1206_write_reg(CAP_MAIN_CONTROL, &main_reg, 1);
    //     HAL_I2C_Mem_Write(&hi2c3, CAP_DEV_ADDR_8_BIT, CAP_MAIN_CONTROL, 1, &main_control_reg, 1, I2C_MAX_TIMEOUT);
    // }

    return CAP_OK;
}

cap1206_ret_status_t cap1206_read_delta_input(cap1206_config_t* config, uint8_t input_values[6]) {
    if (HAL_I2C_Mem_Read(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_DELTA_INPUT_1, 1, input_values, 6, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }

    return CAP_OK;
}

cap1206_ret_status_t cap1206_set_input_threshold(cap1206_config_t* config, uint8_t channel, uint8_t threshold) {
    uint8_t input_threshold_address = 0;

    switch (channel) {
        case CAP_CHANNEL_1:
            input_threshold_address = CAP_INPUT_THRESHOLD_1;
            break;

        case CAP_CHANNEL_2:
            input_threshold_address = CAP_INPUT_THRESHOLD_2;
            break;

        case CAP_CHANNEL_3:
            input_threshold_address = CAP_INPUT_THRESHOLD_3;
            break;

        case CAP_CHANNEL_4:
            input_threshold_address = CAP_INPUT_THRESHOLD_4;
            break;

        case CAP_CHANNEL_5:
            input_threshold_address = CAP_INPUT_THRESHOLD_5;
            break;

        case CAP_CHANNEL_6:
            input_threshold_address = CAP_INPUT_THRESHOLD_6;
            break;

        default:
            return CAP_ERROR;
            break;  // rs
    }


    // Write threshold value to proper channel configuration register
    if (HAL_I2C_Mem_Write(config->i2c_handle, CAP_DEV_ADDR_8_BIT, input_threshold_address, 1, &threshold , 1, I2C_MAX_TIMEOUT) != HAL_OK) {
        return CAP_ERROR;
    }

    return CAP_OK;
}


// uint8_t cap1206_touched() {
//     uint8_t touch_status = 0;
//     uint8_t main_reg;

//     if (HAL_I2C_Mem_Read(&hi2c3, CAP_DEV_ADDR_8_BIT, CAP_SENS_INPUT_STATUS, 1, &touch_status, 1, 1000u) != HAL_OK) {
//         touch_status = 0xFF;
//     }

//     // Clear touch flag on main control register
//     if (touch_status > 0 && touch_status != 0xFF) {
//         main_reg = cap1206_read_reg(CAP_MAIN_CONTROL);
//         main_reg = main_reg & 0x00;
//         cap1206_write_reg(CAP_MAIN_CONTROL, &main_reg, 1);
//     }

//     return touch_status;
// }


cap1206_ret_status_t cap1206_read_calibration_registers(cap1206_config_t* config, uint8_t calib_values[8]) {
    cap1206_ret_status_t ret = CAP_OK;

    if (HAL_I2C_Mem_Read(config->i2c_handle, CAP_DEV_ADDR_8_BIT, CAP_CAL_REGISTERS, 1, calib_values, 6, 1000u) != HAL_OK) {
        ret = CAP_ERROR;
        return ret;
    }

    if (HAL_I2C_Mem_Read(config->i2c_handle, CAP_DEV_ADDR_8_BIT, 0xB9 , 1, &calib_values[6], 1, 1000u) != HAL_OK) {
        ret = CAP_ERROR;
        return ret;
    }

    if (HAL_I2C_Mem_Read(config->i2c_handle, CAP_DEV_ADDR_8_BIT, 0xBA , 1, &calib_values[7], 1, 1000u) != HAL_OK) {
        ret = CAP_ERROR;
        return ret;
    }

    return ret;
}



uint8_t cap1206_read_reg(uint8_t address) {
    uint8_t ret = 0;

    if (HAL_I2C_Mem_Read(&hi2c3, CAP_DEV_ADDR_8_BIT, address, 1, &ret, 1, 1000u) != HAL_OK) {
            ret = 0xFF;
    }

    return ret;
}


uint8_t cap1206_write_reg(uint8_t address, uint8_t* data, uint8_t len) {
    uint8_t ret = 0;

    if (HAL_I2C_Mem_Write(&hi2c3, CAP_DEV_ADDR_8_BIT, address, 1, data, len, 1000u) != HAL_OK) {
            ret = 0xFF;
    }

    return ret;
}
