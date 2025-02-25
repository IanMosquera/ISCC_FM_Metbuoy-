/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define FREE_FLAG            1
#define BUSY_FLAG            2

#define STS40_I2C_ADDR 		(0x46 << 1)
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define AVE(__x__)		((__x__[0] + __x__[1])/2)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void GetSTS40TempC(void);
void PrintPC(char *szFormat, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STAT_Pin GPIO_PIN_0
#define STAT_GPIO_Port GPIOA
#define CE5V_Pin GPIO_PIN_4
#define CE5V_GPIO_Port GPIOA
#define ENUVLO_Pin GPIO_PIN_1
#define ENUVLO_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOE
#define SW1_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */
// Sensor UART Definitions
#define UART_Senix	 0
#define UART_Airmar	 1



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
