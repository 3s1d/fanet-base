/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define SXRX_Pin GPIO_PIN_6
#define SXRX_GPIO_Port GPIOA
#define SXTX_Pin GPIO_PIN_7
#define SXTX_GPIO_Port GPIOA
#define WIND_DIR_Pin GPIO_PIN_0
#define WIND_DIR_GPIO_Port GPIOB
#define SXDIO0_EXTI8_Pin GPIO_PIN_8
#define SXDIO0_EXTI8_GPIO_Port GPIOA
#define SXDIO0_EXTI8_EXTI_IRQn EXTI9_5_IRQn
#define SXSEL_Pin GPIO_PIN_9
#define SXSEL_GPIO_Port GPIOA
#define SXRESET_Pin GPIO_PIN_10
#define SXRESET_GPIO_Port GPIOA
#define WIND_SPEED_EXIT11_Pin GPIO_PIN_11
#define WIND_SPEED_EXIT11_GPIO_Port GPIOA
#define WIND_SPEED_EXIT11_EXTI_IRQn EXTI15_10_IRQn
#define WIND_DIR_PULL_Pin GPIO_PIN_12
#define WIND_DIR_PULL_GPIO_Port GPIOA
#define WIND_PWR_Pin GPIO_PIN_15
#define WIND_PWR_GPIO_Port GPIOA
#define VCC_SUPPLY_Pin GPIO_PIN_6
#define VCC_SUPPLY_GPIO_Port GPIOB
#define ISMISOL_Pin GPIO_PIN_7
#define ISMISOL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
