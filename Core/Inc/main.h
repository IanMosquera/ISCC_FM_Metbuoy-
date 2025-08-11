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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STAT_Pin GPIO_PIN_0
#define STAT_GPIO_Port GPIOA
#define GIMON_Pin GPIO_PIN_3
#define GIMON_GPIO_Port GPIOA
#define CE5V_Pin GPIO_PIN_4
#define CE5V_GPIO_Port GPIOA
#define DS_EFUSE_Pin GPIO_PIN_1
#define DS_EFUSE_GPIO_Port GPIOB
#define SW_OFF_Pin GPIO_PIN_4
#define SW_OFF_GPIO_Port GPIOE
#define SW_OFF_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

#define DATA_64                ((uint64_t)0x1234567812345678)

#define ADC_BUF_LEN 								64
#define Timer16_Max_Counter  				15

// State Machine Definition
#define	STATE_IDLE					00
#define STATE_READDATA			01
#define STATE_READLOAD			02
#define STATE_LONGPRESSED		03

#define STATE_LEDTOGGLE			00
#define STATE_QUICKBLINK		01

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
