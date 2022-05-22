/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define MotorAIN2_Pin GPIO_PIN_0
#define MotorAIN2_GPIO_Port GPIOA
#define MotorAIN1_Pin GPIO_PIN_1
#define MotorAIN1_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define GAS_IN_Pin GPIO_PIN_6
#define GAS_IN_GPIO_Port GPIOA
#define BT_Enable_Pin GPIO_PIN_7
#define BT_Enable_GPIO_Port GPIOA
#define LeftSensor_Pin GPIO_PIN_0
#define LeftSensor_GPIO_Port GPIOB
#define LeftSensor_EXTI_IRQn EXTI0_IRQn
#define MotorSTBY_Pin GPIO_PIN_1
#define MotorSTBY_GPIO_Port GPIOB
#define MotorPWMA_Pin GPIO_PIN_8
#define MotorPWMA_GPIO_Port GPIOA
#define MotorPWMB_Pin GPIO_PIN_11
#define MotorPWMB_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOB
#define RightSensor_Pin GPIO_PIN_4
#define RightSensor_GPIO_Port GPIOB
#define RightSensor_EXTI_IRQn EXTI4_IRQn
#define CenterSensor_Pin GPIO_PIN_5
#define CenterSensor_GPIO_Port GPIOB
#define CenterSensor_EXTI_IRQn EXTI9_5_IRQn
#define MotorBN2_Pin GPIO_PIN_6
#define MotorBN2_GPIO_Port GPIOB
#define MotorBN1_Pin GPIO_PIN_7
#define MotorBN1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
