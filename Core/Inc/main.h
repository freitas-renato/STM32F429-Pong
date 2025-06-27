/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR_BUTTON_IT_Pin GPIO_PIN_2
#define PWR_BUTTON_IT_GPIO_Port GPIOE
#define PWR_BUTTON_IT_EXTI_IRQn EXTI2_IRQn
#define RED_LED_Pin GPIO_PIN_5
#define RED_LED_GPIO_Port GPIOE
#define BLUE_LED_Pin GPIO_PIN_6
#define BLUE_LED_GPIO_Port GPIOE
#define P_SCK_Pin GPIO_PIN_6
#define P_SCK_GPIO_Port GPIOF
#define D_CS_Pin GPIO_PIN_1
#define D_CS_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOA
#define NSLEEP_Pin GPIO_PIN_12
#define NSLEEP_GPIO_Port GPIOB
#define PH_IN2_Pin GPIO_PIN_15
#define PH_IN2_GPIO_Port GPIOB
#define STAT1_out_Pin GPIO_PIN_11
#define STAT1_out_GPIO_Port GPIOD
#define STAT2_out_Pin GPIO_PIN_12
#define STAT2_out_GPIO_Port GPIOD
#define DISP_BLIGHT_Pin GPIO_PIN_8
#define DISP_BLIGHT_GPIO_Port GPIOC
#define CAP_ALERT0_Pin GPIO_PIN_15
#define CAP_ALERT0_GPIO_Port GPIOA
#define CAP_ALERT0_EXTI_IRQn EXTI15_10_IRQn
#define CAP_ALERT2_Pin GPIO_PIN_11
#define CAP_ALERT2_GPIO_Port GPIOC
#define CAP_ALERT2_EXTI_IRQn EXTI15_10_IRQn
#define CAP_ALERT1_Pin GPIO_PIN_12
#define CAP_ALERT1_GPIO_Port GPIOC
#define CAP_ALERT1_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
