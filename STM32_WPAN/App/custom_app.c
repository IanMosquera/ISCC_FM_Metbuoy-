/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"
#include "LTC4162.h"
#include "ASTI_RTC.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* SPP */
  uint8_t               Rx_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

// extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
// extern UART_HandleTypeDef huart1;

extern LTC4162 ltc;

uint8_t i;
char system_Message[128];

uint8_t aShowTime[16] = "hh:ms:ss";
uint8_t rtcTime[12]	=	 "12:03:00";
uint8_t rtcDate[14]	=	 "24-02-01";

static char	a_SzString[70];		/*buffer for everything else*/

// STS40 Variables
extern uint8_t	STS40_RXBuffer[3]; 	// RX buffer for I2C
extern uint8_t	sts40_TXCODE; 			// measure T with highest precision
extern volatile float Temp_C;

extern uint16_t adc_buff_average[2];
extern float gIMON;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* SPP */
static void Custom_Rx_Update_Char(void);
static void Custom_Rx_Send_Notification(void);

/* USER CODE BEGIN PFP */
void ReadChargingData(void);
void SPP_Transmit(void);
void ReadGIMON(void);

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* SPP */
    case CUSTOM_STM_TX_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TX_READ_EVT */

      /* USER CODE END CUSTOM_STM_TX_READ_EVT */
      break;

    case CUSTOM_STM_TX_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TX_WRITE_NO_RESP_EVT */
    	pNotification->DataTransfered.pPayload[pNotification->DataTransfered.Length] = '\0';
      /* USER CODE END CUSTOM_STM_TX_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_RX_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RX_READ_EVT */

      /* USER CODE END CUSTOM_STM_RX_READ_EVT */
      break;

    case CUSTOM_STM_RX_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RX_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_RX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_RX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RX_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_RX_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	UTIL_SEQ_RegTask(1 << CFG_TASK_READ_CHGDATA, UTIL_SEQ_RFU, ReadChargingData);
	UTIL_SEQ_RegTask(1 << CFG_TASK_USER_SW1_PRESSED, UTIL_SEQ_RFU, SPP_Transmit);


	sprintf((char *)a_SzString,"TestMessage\r\n");
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void ReadChargingData(void){
//	RTC_ReadDate(rtcDate);
//	RTC_ReadTime(rtcTime);


	LTC4162_ReadVIN(&ltc);
	LTC4162_ReadIIN(&ltc);
	LTC4162_ReadVOUT(&ltc);
	LTC4162_ReadVBAT(&ltc);
	LTC4162_ReadIBAT(&ltc);
	LTC4162_ReadDieTemp(&ltc);
	LTC4162_ReadChargerState(&ltc);
	LTC4162_ReadChargeStatus(&ltc);
	LTC4162_ReadNTC(&ltc);
	GetSTS40TempC();
	ReadGIMON();

	sprintf((char *)system_Message, "%5.2f, "
																	"%6.3f, "
																	"%5.2f, "
																	"%6.3f, "
																	"%5.2f, "
																	"%.4f, "
																	"%s, "
																	"%s, "
																	"%.2f, "
																	"%.2f, "
																	"%.2f",
																	ltc.vIN,
																	ltc.iIN,
																	ltc.vBAT,
																	ltc.iBAT,
																	ltc.vOUT,
																	gIMON,
																	ltc.chargerStateStr,
																	ltc.chargeStatusStr,
																	ltc.dieTemp,
																	ltc.NTCDegrees,
																	Temp_C);

}

void SPP_Transmit(void){
	sprintf((char *)a_SzString,"TestMessage\r\n");
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&a_SzString[0]);
}

void ReadGIMON(void){
	uint16_t temp;

	temp = (adc_buff_average[0] + adc_buff_average[1]);
	gIMON = (temp * ( 1.765f/4096.0f)) *5.0125;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim2)
	{
//		ReadChargingData();
//		UTIL_SEQ_SetTask(1 << CFG_TASK_READ_CHGDATA, CFG_SCH_PRIO_0);
//		PrintPC("\r\n%s", system_Message);

//		UTIL_SEQ_SetTask(1 << CFG_TASK_USER_SW1_PRESSED, CFG_SCH_PRIO_0);
//		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&a_SzString[0]);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == SW1_Pin)
	{
		sprintf(a_SzString, "SW1 Pressed\r\n");
		PrintPC("%s", a_SzString);
//		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&a_SzString[0]);

	}
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* SPP */
__USED void Custom_Rx_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Rx_UC_1*/

  /* USER CODE END Rx_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_RX, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Rx_UC_Last*/

  /* USER CODE END Rx_UC_Last*/
  return;
}

void Custom_Rx_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Rx_NS_1*/

  /* USER CODE END Rx_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_RX, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Rx_NS_Last*/

  /* USER CODE END Rx_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
