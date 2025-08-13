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
#include "arQ.h"
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
extern dateTime_t DT;

extern uint8_t MidnightEfuseReset_Flag;
extern float gIMON;
extern uint8_t	STS40_RXBuffer[3];     // RX buffer for I2C
extern uint8_t	sts40_TXCODE; 	// measure T with highest precision
extern volatile float Temp_C;
extern RTC_HandleTypeDef hrtc;

static char	a_SzString[70];		/*buffer for everything else*/

bool QuickTwoBlink_Flag = false;
bool SW_Pressed_Flag = false;

char system_Message[128];

uint8_t aShowTime[16] = "hh:ms:ss";
uint8_t aShowDateTime[20] = "YY/MM/DD,hh:ms:ss";
uint8_t rtcTime[12]	=	 "12:03:00";
uint8_t rtcDate[14]	=	 "24-02-01";
uint8_t ReadDataTimer = 0;
uint8_t ReadLoadCurrentTimer = 0;
uint8_t Delay_Reset_Ctr = 0;
uint8_t LED_Ctr = 0;
uint8_t LED_State = 0;
uint8_t LongPress_Ctr = 0;
uint8_t ToSecond_Ctr = 0;
uint8_t State = 0;

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* icmService */
static void Custom_Rx_Update_Char(void);
static void Custom_Rx_Send_Notification(void);

/* USER CODE BEGIN PFP */
void EnableLoadTest(void);
void LED_STAT_TwoBlink(void);
void LED_STAT_Toggle(void);
void LEDStateMachineTest(void);
bool ThreeSecondsLongPressed(void);
void MidnightEfuseResetTest(void);
void ProgrameStateMachineTest(void);
void ReadChargingData(void);
void ReadEFuse(void);
void ReadTempData(void);
void ReadConfigBitsRegister(void);
void ReadSystemStatusRegister(void);
void ReadLoadCurrent(void);
void FilterCommands(uint8_t * pPayload, uint8_t Length);
void ReadRTCTime(void);
void ReadRTCDate(void);
void ReadRTCTask(void);
void SecondsCounterTest(void);
bool TimeToReadData(void);
void TimeToReadDataTest(void);
bool TimeToReadLoadCurrent(void);
void Toggle_Load(void);


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

	UTIL_SEQ_RegTask(1 << CFG_TASK_READCHGDATA,  	UTIL_SEQ_RFU, ReadChargingData);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READEFUSE,  		UTIL_SEQ_RFU, ReadEFuse);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READ_IOUT,  		UTIL_SEQ_RFU, ReadLoadCurrent);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READTEMPDATA,	UTIL_SEQ_RFU, ReadTempData);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READCFBTREG,		UTIL_SEQ_RFU, ReadConfigBitsRegister);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READ_RTC_DATA,	UTIL_SEQ_RFU, ReadRTCTask);
	UTIL_SEQ_RegTask(1 << CFG_TASK_READSYSSTREG, 	UTIL_SEQ_RFU, ReadSystemStatusRegister);
	UTIL_SEQ_RegTask(1 << CFG_TASK_TOGGLE_LOAD, 	UTIL_SEQ_RFU, Toggle_Load);
	UTIL_SEQ_RegTask(1 << CFG_TASK_SEND_STR, 			UTIL_SEQ_RFU, SPP_Transmit);

	MidnightEfuseReset_Flag = false;

	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

void FilterCommands(uint8_t * pPayload, uint8_t Length)
{
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

	// For iscc Date and Time
	else if((pPayload[0]=='D') & (pPayload[1]=='T'))
	{
		if ((pPayload[3]=='R') & (pPayload[4]=='D') & (pPayload[5]=='T'))
		{
		sprintf((char *)str,"%02d/%02d/%02d,%02d:%02d:%02d\r\n", DT.Year, DT.Month, DT.Days, DT.Hour, DT.Min, DT.Sec);
		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&str[0]);
		}
		else if((pPayload[3]=='W') & (pPayload[4]=='D') & (pPayload[5]=='T'))
		{
			DT.Year = ((pPayload[7]  - 48) * 10) + (pPayload[8]  - 48);
			DT.Month = ((pPayload[10] - 48) * 10) + (pPayload[11] - 48);
			DT.Days = ((pPayload[13] - 48) * 10) + (pPayload[14] - 48);
			DT.Hour = ((pPayload[16] - 48) * 10) + (pPayload[17]  - 48);
			DT.Min = ((pPayload[19] - 48) * 10) + (pPayload[20] - 48);
			DT.Sec = ((pPayload[22] - 48) * 10) + (pPayload[23] - 48);
		}
	}


	// For EFUSE
	else if((pPayload[0]=='E') & (pPayload[1]=='N') & (pPayload[2]=='E') & (pPayload[3]=='F')) EnableLoad();
	else if((pPayload[0]=='D') & (pPayload[1]=='S') & (pPayload[2]=='E') & (pPayload[3]=='F')) DisableLoad();

}


void EnableLoadTest(void)
{
	if (SW_Pressed_Flag)
	{
		if (ThreeSecondsLongPressed())
		{
			UTIL_SEQ_SetTask(1 << CFG_TASK_TOGGLE_LOAD, CFG_SCH_PRIO_0);
			LED_Ctr = 0;
		}
	}
}



void LED_STAT_TwoBlink(void)
{
	if (LED_Ctr < 4)
	{
		if (LED_Ctr == 0) HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_SET);
		if (LED_Ctr == 1) HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);
		if (LED_Ctr == 2) HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_SET);
		if (LED_Ctr == 3) HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);
		LED_Ctr++;
	}
	else
	{
		LED_Ctr = 0;
		LED_State = STATE_LEDTOGGLE;
	}
}


void LED_STAT_Toggle(void)
{
	if (LED_Ctr < 10)
		LED_Ctr++;
	else
	{
		HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
		LED_Ctr = 0;
		State = STATE_IDLE;
	}
}


void LEDStateMachineTest(void)
{
	switch (LED_State)
	{
		case STATE_LEDTOGGLE:
			LED_STAT_Toggle();
			break;
		case STATE_QUICKBLINK:
			LED_STAT_TwoBlink();
			break;
		default:
			break;
	}
}





void MidnightEfuseResetTest(void)
{
	if (MidnightEfuseReset_Flag)
	{
		if (Delay_Reset_Ctr == 0) DisableLoad();

		if (Delay_Reset_Ctr < 10)
		{
			Delay_Reset_Ctr++;
		}
		else
		{
			EnableLoad();
			Delay_Reset_Ctr = 0;
			MidnightEfuseReset_Flag = false;
		}
	}
}



void ProgrameStateMachineTest(void)
{
	switch(State)
	{
		case STATE_IDLE:
			EnableLoadTest();
			TimeToReadDataTest();
			break;

		case STATE_READDATA:
			UTIL_SEQ_SetTask(1 << CFG_TASK_READCHGDATA, CFG_SCH_PRIO_0);
			UTIL_SEQ_SetTask(1 << CFG_TASK_READSYSSTREG, CFG_SCH_PRIO_0);
			State = STATE_READLOAD;
			break;

		case STATE_READLOAD:
			UTIL_SEQ_SetTask(1 << CFG_TASK_READ_IOUT, CFG_SCH_PRIO_0);
			UTIL_SEQ_SetTask(1 << CFG_TASK_READEFUSE, CFG_SCH_PRIO_0);
			LED_Ctr = 0;
			State = STATE_IDLE;
			break;

		default:
			break;
	}
}

void ReadChargingData(void)
{
	LTC4162_ReadVIN(&ltc);
	LTC4162_ReadIIN(&ltc);
	LTC4162_ReadVBAT(&ltc);
	LTC4162_ReadIBAT(&ltc);
	LTC4162_ReadVOUT(&ltc);
	LTC4162_ReadChargerState(&ltc);
	LTC4162_ReadChargeStatus(&ltc);

	sprintf((char *)system_Message, "\r\nCHG_DATA: "
																	"%5.2f, "
																	"%6.3f, "
																	"%5.2f, "
																	"%6.3f, "
																	"%5.2f, "
																	"%s, %s\r\n",
																	ltc.vIN,
																	ltc.iIN,
																	ltc.vBAT,
																	ltc.iBAT,
																	ltc.vOUT,
																	ltc.chargerStateStr, ltc.chargeStatusStr);

	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
	xprintf(PC, "%s", system_Message);
}




void ReadEFuse(void)
{
	if (HAL_GPIO_ReadPin(nEF_FLT_GPIO_Port, nEF_FLT_Pin) == GPIO_PIN_RESET)
	{
		xprintf(PC, "Efuse: Fault!\r\n");
		DisableLoad();
		HAL_Delay(1000);
		EnableLoad();
	}
	else
	{
		xprintf(PC, "Efuse: OK!\r\n");
	}
}





void ReadTempData(void){
	LTC4162_ReadDieTemp(&ltc);
	LTC4162_ReadNTC(&ltc);
	GetSTS40TempC();

	sprintf((char *)system_Message,
			"DieTemp : %5.2f, "
			"NTC: %5.2f, "
			"BoardTemp: %5.2f\r\n",
			ltc.dieTemp,
			ltc.NTCDegrees,
			Temp_C);

	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
	xprintf(PC, "%s", system_Message);
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
	xprintf(PC, "%s", system_Message);
}

void ReadSystemStatusRegister(void){
	LTC4162_ReadSystemStatusReg(&ltc);

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
		xprintf(PC, "%s", system_Message);;
}





void ReadLoadCurrent(void)
{
	LTC4162_ReadConfigBitsReg(&ltc);
	LTC4162_ReadSystemStatusReg(&ltc);
	//if (ltc.ssReg.vin_gt_vbat == 0) LTC4162_WriteConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);

	if (ltc.ssReg.vin_gt_vbat)
	{
		if(ltc.ssReg.en_chg){
			// suspend charger
			SetConfigBitsReg(&ltc, suspend_charger);
			sprintf((char *)system_Message, "Reading Current Load:");
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
			xprintf(PC, "%s", system_Message);
			HAL_Delay(1000);
			LTC4162_ReadIIN(&ltc);
			ltc.iOUT = ltc.iIN;

			SetConfigBitsReg(&ltc, suspend_charger);
			sprintf((char *)system_Message, "...\r\n");
			xprintf(PC, "%s", system_Message);
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
			HAL_Delay(1000);
			LTC4162_ReadIIN(&ltc);
			ltc.iOUT = ltc.iIN;

			/*			SetConfigBitsReg(&ltc, suspend_charger);
			sprintf((char *)system_Message, "#\r\n");
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
			HAL_Delay(1000);
			LTC4162_ReadIIN(&ltc);
			ltc.iOUT = ltc.iIN;*/

			sprintf((char *)system_Message, "LOAD_CUR: %6.3f\r\n", ltc.iOUT);
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
			xprintf(PC, "%s", system_Message);
			// clear suspend charger
			SetConfigBitsReg(&ltc, zero_cfg);
		}
		else{
			SetConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);
			HAL_Delay(500);
			LTC4162_ReadIIN(&ltc);
			ltc.iOUT = ltc.iIN;
			sprintf((char *)system_Message, "LOAD_CUR: %6.3f\r\n", ltc.iOUT);
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
			xprintf(PC, "%s", system_Message);
			SetConfigBitsReg(&ltc, zero_cfg);
		}
	}
	else
	{
		SetConfigBitsReg(&ltc, force_telemetry_on | telemetry_speed);
		LTC4162_ReadIBAT(&ltc);
		ltc.iOUT = -ltc.iBAT;
		sprintf((char *)system_Message, "LOAD_CUR: %6.3f\r\n", ltc.iOUT);
		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
		xprintf(PC, "%s", system_Message);
		SetConfigBitsReg(&ltc, zero_cfg);
	}
}



void ReadRTCTime(void){
	//Get RTC time, other parameters to be displayed are taken from
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);


	//Save time to buffer
	sprintf((char *)rtcTime,   "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
}

void ReadRTCDate(void){
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf((char *)rtcDate,   "%02d-%02d-%02d ", sDate.Year, sDate.Month, sDate.Date);
}


void ReadRTCTask(void)
{
	RTC_ReadDate(rtcDate);
	RTC_ReadTime(rtcTime);
	RTC_ReadDate(rtcDate);
	RTC_ReadTime(rtcTime);

	sprintf((char *)system_Message, "RTC: %s %s\r\n", rtcDate, rtcTime);
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&system_Message[0]);
	xprintf(PC, "%s", system_Message);
}




void SecondsCounterTest(void)
{
	// Timer ticks every tenth of a second
	// Calls CountTimeSeconds() every 10/10 of tenth of a second
	if (ToSecond_Ctr < 10)
	{
		ToSecond_Ctr++;
	}
	else
	{
		CountTimeSeconds();
		ToSecond_Ctr = 0;
	}
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





bool ThreeSecondsLongPressed(void)
{
	if (LongPress_Ctr < 30)
	{
		LongPress_Ctr++;
		return false;
	}
	else
	{
		LongPress_Ctr = 0;
		return true;
	}
}





bool TimeToReadData(void)
{
	if (ReadDataTimer >= 99)
	{
		ReadDataTimer = 0;
		return true;
	}
	else
	{
		ReadDataTimer++;
		return false;
	}
}




void TimeToReadDataTest(void)
{
	if (TimeToReadData())
		State = STATE_READDATA;
}


bool TimeToReadLoadCurrent(void)
{
	if (ReadLoadCurrentTimer >= 29)
	{
		ReadLoadCurrentTimer = 0;
		return true;
	}
	else
	{
		ReadLoadCurrentTimer++;
		return false;
	}

}




void Toggle_Load(void)
{
	if (HAL_GPIO_ReadPin(SW_OFF_GPIO_Port, SW_OFF_Pin) == GPIO_PIN_RESET)
	{
		HAL_GPIO_TogglePin(DS_EFUSE_GPIO_Port, DS_EFUSE_Pin);

		sprintf(a_SzString, "+LOAD toggled!\r\n");
		SPP_Transmit();
		xprintf(PC, a_SzString);

		LED_Ctr = 0;
		LED_State = STATE_QUICKBLINK;
	}
	SW_Pressed_Flag = false;
}





void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SW_OFF_Pin)
  {
  	SW_Pressed_Flag = true;

  	sprintf(a_SzString, "SW_OFF button pressed\r\n");
		UTIL_SEQ_SetTask(1 << CFG_TASK_SEND_STR, CFG_SCH_PRIO_0);

		LongPress_Ctr = 0;
		//State = STATE_LONGPRESSED;
  }
}

/**
 *  @brief	Timer interrupt function for performing registered tasks
 *  @param	*htim	Timer handler
 *  @retval 	None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		SecondsCounterTest();
		MidnightEfuseResetTest();
		LEDStateMachineTest();
		ProgrameStateMachineTest();
	}
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
