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
#include "usbd_cdc_if.h"
#include "LTC4162.h"
#include "ASTI_RTC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* icmService */
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

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

extern LTC4162 ltc;


static char	a_SzString[70];		/*buffer for everything else*/
uint8_t txbuff[70];

char system_Message[128];

// ADC Variables
extern float gIMON;

// STS40 Variables
extern uint8_t	STS40_RXBuffer[3];     // RX buffer for I2C
extern uint8_t	sts40_TXCODE; 	// measure T with highest precision
extern volatile float Temp_C;

// RTC Variables
extern RTC_HandleTypeDef hrtc;
uint8_t aShowTime[16] = "hh:ms:ss";
uint8_t aShowDateTime[20] = "YY/MM/DD,hh:ms:ss";
uint8_t rtcTime[12]	=	 "12:03:00";
uint8_t rtcDate[14]	=	 "24-02-01";
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* icmService */
static void Custom_Rx_Update_Char(void);
static void Custom_Rx_Send_Notification(void);

/* USER CODE BEGIN PFP */
void ReadChargingData(void);
void ReadTempData(void);
void ReadConfigBitsRegister(void);
void ReadSystemStatusRegister(void);
void ReadGIMON(void);
void ReadOutCurrent(void);
void FilterCommands(uint8_t * pPayload, uint8_t Length);
void ReadRTCTime(void);
void ReadRTCDate(void);

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

    /* icmService */
    case CUSTOM_STM_TX_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TX_WRITE_NO_RESP_EVT */
    	pNotification->DataTransfered.pPayload[pNotification->DataTransfered.Length] = '\0';
    	FilterCommands(pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length);
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

	UTIL_SEQ_RegTask(1 << CFG_TASK_READCHGDATA, UTIL_SEQ_RFU, ReadChargingData);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READTEMPDATA, UTIL_SEQ_RFU, ReadTempData);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READCFBTREG, UTIL_SEQ_RFU, ReadConfigBitsRegister);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READSYSSTREG, UTIL_SEQ_RFU, ReadSystemStatusRegister);

	sprintf(a_SzString, "BLE Transmit Test\r\n");

	// Start Timer for Reading Charging Data
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void ReadChargingData(void){
//	RTC_ReadDate(rtcDate);
//	RTC_ReadTime(rtcTime);

	LTC4162_ReadIIN(&ltc);
	LTC4162_ReadIBAT(&ltc);
	LTC4162_ReadVIN(&ltc);
	LTC4162_ReadVOUT(&ltc);
	LTC4162_ReadVBAT(&ltc);
	LTC4162_ReadChargerState(&ltc);
	LTC4162_ReadChargeStatus(&ltc);
	ReadOutCurrent();
//	ReadGIMON();

	sprintf((char *)system_Message, "\r\nCHG_DATA: %5.2f, "
																	"%6.3f, "
																	"%5.2f, "
																	"%6.3f, "
																	"%5.2f, "
																	"%6.3f, "
																	"%s, "
																	"%s\r\n",
																	ltc.vIN,
																	ltc.iIN,
																	ltc.vBAT,
																	ltc.iBAT,
																	ltc.vOUT,
																	ltc.iOUT,
																	ltc.chargerStateStr,
																	ltc.chargeStatusStr);

	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
	PrintPC("%s", system_Message);
}

void ReadTempData(void){
	LTC4162_ReadDieTemp(&ltc);
	LTC4162_ReadNTC(&ltc);
	GetSTS40TempC();

	sprintf((char *)system_Message, "DieTemp: %5.2f, "
																	"NTC: %5.2f, "
																	"BoardTemp: %5.2f\r\n",
			ltc.dieTemp,
			ltc.NTCDegrees,
			Temp_C);

	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
	PrintPC("%s", system_Message);
}

void ReadConfigBitsRegister(void){
	LTC4162_ReadConfigBitsReg(&ltc);

	// If there is sun, reset system configuration to zero
	if (ltc.ssReg.vin_gt_vbat){
		LTC4162_WriteConfigBitsReg(&ltc, zero_cfg);
	}
	else {
		LTC4162_WriteConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);
	}

	sprintf((char *)system_Message, "CON_BITS: %s%s%s%s%s\r\n",
			ltc.confBits.mppt_en?"mppt_en, ":"",
			ltc.confBits.force_telemetry_on?"f_tel_on, ":"",
			ltc.confBits.telemetry_speed?"tel_HS, ":"",
			ltc.confBits.run_bsr?"run_bsr, ":"",
			ltc.confBits.suspend_charger?"susp_chg ":"");

	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
	PrintPC("%s", system_Message);
}

void ReadSystemStatusRegister(void){
	LTC4162_ReadSystemStatusReg(&ltc);
	// if (ltc.ssReg.vin_gt_vbat) LTC4162_WriteConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);

	sprintf((char *)system_Message, "SYS_STAT: %s%s%s%s%s%s%s%s\r\n",
				ltc.ssReg.intvcc_gt_2p8v?"intvcc>2.8v, ":"",
				ltc.ssReg.vin_gt_4p2v?"vin>4.2v, ":"",
				ltc.ssReg.vin_gt_vbat?"vin>vbat, ":"",
				ltc.ssReg.vin_ovlo?"vin_ovlo, ":"",
				ltc.ssReg.thermal_shutdown?"thm_shdn, ":"",
				ltc.ssReg.no_rt?"no_rt, ":"",
				ltc.ssReg.cell_count_err?"cell_ctr_err, ":"",
				ltc.ssReg.en_chg?"en_chg ":"");

		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
		PrintPC("%s", system_Message);
}

void ReadOutCurrent(void){
	LTC4162_ReadConfigBitsReg(&ltc);
	LTC4162_ReadSystemStatusReg(&ltc);
	//if (ltc.ssReg.vin_gt_vbat == 0) LTC4162_WriteConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);

	if (ltc.ssReg.vin_gt_vbat){
		if(ltc.ssReg.en_chg){
			// suspend charger
			// SetConfigBitsReg(&ltc, suspend_charger);
			// HAL_Delay(20);
			ltc.iOUT = ltc.iIN - ltc.iBAT;
			// clear suspend charger
			//SetConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);
		}
		else{
			//SetConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);
			HAL_Delay(20);
			ltc.iOUT = ltc.iIN;
		}
	}else{
		// SetConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);

		ltc.iOUT = ltc.iBAT;
	}


}

void FilterCommands(uint8_t * pPayload, uint8_t Length){
	uint8_t address, buf[2];
	uint8_t str[64];
	uint16_t val;

	// For LTC
	if ((pPayload[0]=='L') & (pPayload[1]=='T') & (pPayload[2]=='C')){
		// Read Register
		if (pPayload[3]=='R'){
			address = (hexCharToInt(pPayload[4]) << 4) | hexCharToInt(pPayload[5]);
			LTC4162_ReadRegisters(&ltc, address, buf, 2);
			val = (buf[1] << 8) | buf[0];

			sprintf((char *)str,"REG 0x%02X: %d\r\n",address, val);
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&str[0]);

		// Write Register
		}else if (pPayload[3]=='W' && pPayload[6] == ':'){
			address = (hexCharToInt(pPayload[4]) << 4) |  hexCharToInt(pPayload[5]);  /* Covert Hex to Integer*/
			if (pPayload[11] == '\0'){
				buf[0] = (hexCharToInt(pPayload[7]) << 4) |  hexCharToInt(pPayload[8]); /* Covert Hex to Integer*/
				HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, LTC4162_I2C_ADDR, address, 1, buf, 2, HAL_MAX_DELAY);
				if (status != HAL_OK) status = 1;
			}else if (pPayload[13] =='\0'){
				buf[0] = (hexCharToInt(pPayload[9]) << 4) |  hexCharToInt(pPayload[10]); /*Covert Hex to Integer*/
				buf[1] = (hexCharToInt(pPayload[7]) << 4) |  hexCharToInt(pPayload[8]); /*Covert Hex to Integer*/
				HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, LTC4162_I2C_ADDR, address, 1, buf, 2, HAL_MAX_DELAY);
				if (status != HAL_OK) status = 1;
			}
		}
		else{
		}
	}
	// For RTC
	else if((pPayload[0]=='R') & (pPayload[1]=='T') & (pPayload[2]=='C')){
		if ((pPayload[3]=='R') & (pPayload[4]=='T')){
			// ReadRTCTime
			RTC_ReadTime(aShowTime);
			sprintf((char *)str,"%s\r\n",aShowTime);
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&str[0]);
		}
		else if((pPayload[3]=='R') & (pPayload[4]=='D') & (pPayload[5]=='\r')){
			// ReadRTCDate
			RTC_ReadDate(aShowTime);
			sprintf((char *)str,"%s\r\n",aShowTime);
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&str[0]);
		}
		else if ((pPayload[3]=='R') & (pPayload[4]=='D') & (pPayload[5]=='T')){
			// ReadRTCDate
			RTC_ReadDateTime(aShowDateTime);
			sprintf((char *)str,"%s\r\n",aShowDateTime);
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&str[0]);
		}
		else if((pPayload[3]=='W') & (pPayload[4]=='T')){
		  RTC_TimeTypeDef	 sTime = 	{0};
		  uint8_t hrs = ((pPayload[6]  - 48) * 10) + (pPayload[7]  - 48);
		  uint8_t min = ((pPayload[9]  - 48) * 10) + (pPayload[10] - 48);
		  uint8_t sec = ((pPayload[12] - 48) * 10) + (pPayload[13] - 48);
			sTime.Hours = hrs;
			sTime.Minutes = min;
			sTime.Seconds = sec;
			RTC_WriteTime(&sTime);
		}
		else if((pPayload[3]=='W') & (pPayload[4]=='D') & (pPayload[5]==':')){
			RTC_DateTypeDef	 sDate = 	{0};
			uint8_t yrs = ((pPayload[6]  - 48) * 10) + (pPayload[7]  - 48);
			uint8_t mon = ((pPayload[9]  - 48) * 10) + (pPayload[10] - 48);
			uint8_t day = ((pPayload[12] - 48) * 10) + (pPayload[13] - 48);
			sDate.Year = yrs;
			sDate.Month = mon;
			sDate.Date = day;
			RTC_WriteDate(&sDate);
		}
		else if((pPayload[3]=='W') & (pPayload[4]=='D') & (pPayload[5]=='T')){
			RTC_DateTypeDef	 sDate = 	{0};
			uint8_t yrs = ((pPayload[7]  - 48) * 10) + (pPayload[8]  - 48);
			uint8_t mon = ((pPayload[10] - 48) * 10) + (pPayload[11] - 48);
			uint8_t day = ((pPayload[13] - 48) * 10) + (pPayload[14] - 48);
			sDate.Year = yrs;
			sDate.Month = mon;
			sDate.Date = day;

			RTC_TimeTypeDef	 sTime = 	{0};
			uint8_t hrs = ((pPayload[16] - 48) * 10) + (pPayload[17]  - 48);
			uint8_t min = ((pPayload[19] - 48) * 10) + (pPayload[20] - 48);
			uint8_t sec = ((pPayload[22] - 48) * 10) + (pPayload[23] - 48);
			sTime.Hours = hrs;
			sTime.Minutes = min;
			sTime.Seconds = sec;

			RTC_WriteDateTime(&sTime, &sDate);
		}
	}
}

void ReadRTCTime(void){
	//Get RTC time, other parameters to be displayed are taken from
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);


	//Save time to buffer
	sprintf((char *)rtcTime,   "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
}

void ReadRTCDate(void){
	//Get RTC time, other parameters to be displayed are taken from
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	//Save date to buffer
	sprintf((char *)rtcDate,   "%02d-%02d-%02d ", sDate.Year, sDate.Month, sDate.Date);
}

void ReadGIMON(void){
//	uint16_t temp;
//
//	temp = (adc_buff_average[0] + adc_buff_average[1]);
//	gIMON = (temp * ( 1.765f/4096.0f)) *5.0125;
}


/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* icmService */
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

/**
 *  @brief	Custom function for updating character value
 *  @param	None
 *  @retval 	None
 */
void SPP_Transmit(void){
  SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&a_SzString[0]);
}

/**
 *  @brief	Timer interrupt function for performing registered tasks
 *  @param	*htim	Timer handler
 *  @retval 	None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim2){
		UTIL_SEQ_SetTask(1 << CFG_TASK_READCHGDATA, CFG_SCH_PRIO_0);
		UTIL_SEQ_SetTask(1 << CFG_TASK_READCFBTREG, CFG_SCH_PRIO_0);
		UTIL_SEQ_SetTask(1 << CFG_TASK_READSYSSTREG, CFG_SCH_PRIO_0);
		// UTIL_SEQ_SetTask(1 << CFG_TASK_READTEMPDATA, CFG_SCH_PRIO_0);

		HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
	}
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
