#ifndef __TOUCH_INTERFACE_H__
#define __TOUCH_INTERFACE_H__

#include <stdint.h>
#include "cap1206.h"

/** CAP1206 main configuration struct */
extern cap1206_config_t cap_config;


// /**
//  * @brief CAP1206 initialization task. 
//  * 
//  * @param arguments 
//  */
// void CAP_Task(void* arguments);


/**
 * @brief Used to interpret touch events. This function will be scheduled
 * to be executed on GPIO_EXTI callback when an alert interrupt is generated
 * by CAP1206.
 * 
 * @param pvParameter1  Not used (NULL)
 * @param ulParameter2  Not used (0)
 * 
 * @note It uses the global cap1206 config struct and I2C functions
 */
void process_touch_event(void *pvParameter1, uint32_t ulParameter2);



#endif  // __TOUCH_INTERFACE_H__