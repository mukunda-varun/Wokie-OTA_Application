/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "dcMotor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END Header_android_task */
void  Decode_Message_Info();
void Send_Status_Android(uint8_t msg_id);
void Execute_Step(uint32_t step);
void Send_Abort_Command();
void Send_Android_Debug_Packet(void);
void Process_WOKIE_Control(uint8_t cmd_id1, uint16_t  data);
extern void dacSetVoltage( uint16_t output, char writeEEPROM );
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Android_task_t */
osThreadId_t Android_task_tHandle;
const osThreadAttr_t Android_task_t_attributes = {
  .name = "Android_task_t",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for inductionContro */
osThreadId_t inductionControHandle;
const osThreadAttr_t inductionContro_attributes = {
  .name = "inductionContro",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for motorControl */
osThreadId_t motorControlHandle;
const osThreadAttr_t motorControl_attributes = {
  .name = "motorControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for sensorRead */
osThreadId_t sensorReadHandle;
const osThreadAttr_t sensorRead_attributes = {
  .name = "sensorRead",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for errorHandle_t */
osThreadId_t errorHandle_tHandle;
const osThreadAttr_t errorHandle_t_attributes = {
  .name = "errorHandle_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */

uint8_t cooking_started=0;
uint8_t temp_raise=0, ackCnt = 0, wattAckCnt = 0;
uint8_t counter=0;
uint8_t pressCnt = 0;


/*Sequence for Seven Segment display
 * BFAGDHCE(76543210)
 * Look-up table for the values of the Display*/
#if WOKIE_GREEN_BRD_MC_EN == 1
uint8_t displayHexArray[13]={ZERO_VALUE, ONE_VALUE, TWO_VALUE, THREE_VALUE, FOUR_VALUE, FIVE_VALUE, SIX_VALUE, SEVEN_VALUE, EIGHT_VALUE, NINE_VALUE, E_VALUE, N_VALUE, F_VALUE};
uint8_t tm1668Display[13] = {'0','1','2','3','4','5','6','7','8','9','E','N','F'};
#elif WOKIE_GREEN_V_1_5_EN
uint8_t displayHexArray[15]={ZERO_VALUE, ONE_VALUE, TWO_VALUE, THREE_VALUE, FOUR_VALUE, FIVE_VALUE, SIX_VALUE, SEVEN_VALUE, EIGHT_VALUE, NINE_VALUE, E_VALUE, N_VALUE, F_VALUE, DASH_VALUE, A_VALUE};
uint8_t tm1668Display[15] = {'0','1','2','3','4','5','6','-', 'A','7','8','9','E','N','F'};
#endif
uint16_t wattageLevelsTM1668[13] = {WATT_500, WATT_800, WATT_1000, WATT_1300, WATT_1500, WATT_1900, WATT_2300, WATT_2700, WATT_3100, WATT_3500, WATT_4000, WATT_4500, WATT_5000};
uint16_t temperatureLevelsTM1668[13] = {TEMP_60, TEMP_80, TEMP_100, TEMP_120, TEMP_130, TEMP_140, TEMP_150, TEMP_160, TEMP_170, TEMP_180, TEMP_200, TEMP_230, TEMP_260};
/*Look-up table for the values of the Display*/


/********Induction Variables*************/

/********Induction Variables*************/

/*Look-up table for DAC*/
uint16_t wattageLevelDAC[10] = {INDUCTION_OFF_COUNT, LEVEL1_COUNT, LEVEL2_COUNT, LEVEL3_COUNT, LEVEL4_COUNT, LEVEL5_COUNT, LEVEL6_COUNT, LEVEL7_COUNT, LEVEL8_COUNT, LEVEL9_COUNT};
uint8_t displayData_BS84C12A[12] = {ZERO_BS84C12A, ONE_BS84C12A, TWO_BS84C12A, THREE_BS84C12A, FOUR_BS84C12A, FIVE_BS84C12A, SIX_BS84C12A, SEVEN_BS84C12A, EIGHT_BS84C12A, NINE_BS84C12A, ERROR_BS84C12A, DASH_BS84C12A};
/*Look-up table for DAC*/

/*Variables for DAC wattage setting*/
uint8_t wattageLevel_BS84C12A;
uint16_t wattageSettingDACCounts_BS84C12A;

uint8_t displayValueHex;
uint8_t bs84c12aInductionError;
/*ADC DMA Variables*/
#define ADC1_CONVERTED_DATA_BUFFER_SIZE ((uint32_t)  7)
uint32_t ADC1_DATA[ADC1_CONVERTED_DATA_BUFFER_SIZE] ;
uint32_t adcDataDisplaySegments[ADC1_CONVERTED_DATA_BUFFER_SIZE];
float displayVtgFromADC[ADC1_CONVERTED_DATA_BUFFER_SIZE];

/*ADC DMA Variables*/
uint8_t displaySegmentArray[7] = {0, 0, 0, 0, 0, 0, 0};
uint8_t displayData = 0;
/*Variables for DAC wattage setting*/

/*Data Buffers & index variables*/
uint8_t spiReadBuff[MAX_ARRAY_SIZE] = "";
uint8_t filterBuff[MAX_ARRAY_SIZE]="";
uint32_t startIndex = 0,copyStartIndex = 0;
uint32_t soi;
/*Data Buffers & index variables*/

/*Uart Buffers & Variables*/
uint8_t uartTxBuff[MAX_ARRAY_SIZE] = "";
uint8_t uartData = 0;
char uartPrintArr[6];
/*Uart Buffers & Variables*/


/*Android Variables*/
uint8_t received_data[USB_PACKET_LENGTH],Data_Reciption;
uint32_t received_data_size;
uint32_t receive_total;
uint8_t data_id,cmd_id, tempCmdID;
//uint8_t autoTemperatureMode, autoMode;
uint16_t dataAndroid = 0;
uint8_t and_ack[64];
float temperatureACKAuto, temperatureACKManual, speedCorrectionFactor;
uint16_t motorRotateCleaningTime, motorRotateCleaningCnt;
/*Android Variables*/

/*For Wokie Added on 14-04-21*/
uint8_t msg[USB_PACKET_LENGTH];
/*For Wokie Added on 14-04-21*/


/********Sensor Variables*************/
uint16_t objTemperature = 0.0f;
int temperatureArray[20] = {};
float avgTemperature = 0.0f, sumTemperature = 0, previousTemperature = 0;
char temperatureBuff[100] = "";
uint16_t initialOnOffCnt = 0;
/********Input Capture Variables*************/
volatile float current_speed = 0;
volatile float previous_speed = 0;

float speed_avg[10];
float sum_speed = 0;
float avg_speed = 0, correctionFactor = 0.0f;
int speed_cnt = 0;
char speedSensorBuff[100] = "";

volatile uint8_t gu8_State = IC_IDLE;
volatile uint8_t no_tooth = 0;
volatile uint32_t gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
volatile uint32_t gu16_TIM2_OVC = 0;
volatile float gu32_Freq = 0;
volatile float speed_ic = 0;
/********Averaging IC Variables*************/
volatile uint32_t tick_arr[25];
volatile uint32_t sum_tick = 0;
volatile uint32_t avg_tick = 0;
volatile uint8_t avgCnt = 0;
/********Averaging IC Variables*************/
/********Input Capture Variables*************/

/********Sensor Variables*************/

/********DC Motor Variables*************/
float dutyCycle, cleanDutyCycle;
/********DC Motor Variables*************/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void *argument);
void android_task(void *argument);
void InductionControlTask(void *argument);
void MotorControlTask(void *argument);
void SensorReadTask(void *argument);
void errorHandleTask(void *argument);

/* USER CODE BEGIN PFP */

void clearErrorCounts(void)
{
//	 timerCnt.errorConfirmationCnt = 0;
	 timerCnt.speedSensorErrorCnt = 0;
	 timerCnt.temperatureSensorErrorCnt = 0;
	 timerCnt.inductionBoardErrorCnt = 0;
	 timerCnt.inductionErrorCnt = 0;
	 timerCnt.stopErrorCnt = 0;
}
void clearErrors(void)
{
	 processError.temperatureSensorError = 0;
	 processError.speedSensorError = 0;
	 processError.inductionBoardError = 0;
	 processError.processStopError = 0;
	 processError.errorNumberAndroid = 0;
}
void clearProcessVariables(void)
{
	 androidProcessStruct.temperatureAutoMode = 0;
	 androidProcessStruct.sendTempAckOnce = 0;
	 androidProcessStruct.manualMode =0;
	 androidProcessStruct.startofCooking =0;
}
/* @brief:  Send Acknowledment Response to USB
 * @params: id->Message ID
 * 			data->Data or Message
 * @return: NOTHING*/
void  Send_Response(uint8_t msgId ,uint8_t data)
{
	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = msgId;
	msg[2] = data;
	msg[63] = 0x12;
	CDC_Transmit_FS(msg,64);
}

void  Send_Status_Wait_ACK()
{
	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = STATUS_ID;
	msg[2] = 2;
	msg[3] = (int)avgTemperature;
	msg[4] = ((int)avgTemperature >> 8);
	msg[5] = avg_speed;
	msg[6] = 0;
	msg[7] = 0;
	msg[63] = 0x12;
	CDC_Transmit_FS(msg,64);
}

void  Send_Status_data()
{
	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = STATUS_ID;
	msg[2] = 1;
	msg[3] = (int)avgTemperature;
	msg[4] = ((int)avgTemperature >> 8);
	msg[5] = avg_speed;
	msg[6] = 0;
	msg[7] = 0;
	msg[63] = 0x12;
	osDelay(20);
	CDC_Transmit_FS(msg,64);
}


void  Send_Standby_Status()
{
	memset(msg,0,64);

		msg[0] = 0x13;
		msg[1] = STATUS_ID;
		msg[2] = 2;
		msg[3] = 0xFF;
		msg[4] = 0xFF;
		msg[5] = 0xFF;
		msg[6] = 0xFF;
		msg[7] = 0;
		msg[63] = 0x12;

	CDC_Transmit_FS(msg,64);
}

void  Send_temp_Response(uint8_t id ,uint8_t data,uint8_t info)
{
//	static int8_t temp = 100;
	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = id;
	msg[2] = data;
	msg[3] = info;
	msg[63] = 0x12;
	CDC_Transmit_FS(msg,64);
}

void  Send_Error_Msg(uint8_t errorNo)
{
	memset(msg,0,64);
	msg[0] = 0x13;
	msg[1] = ERROR_ID;
	msg[2] = errorNo;
	msg[63] = 0x12;
	CDC_Transmit_FS(msg,64);
}


/* @brief:  Send Acknowledment Response to USB
 * @params: id->Message ID
 * 			data->Data or Message
 * @return: NOTHING*/
void  Send_ADR_Cmd(void)
{
	memcpy(msg,and_ack,64);

	CDC_Transmit_FS(msg,64);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void stopHeaterBasedOnError(void)
{
	if(processError.speedSensorError == 0 && processError.temperatureSensorError == 0 && processError.inductionBoardError == 0)
	{
		  processError.processStopError = 0;
		  timerCnt.stopErrorCnt = 0;
	}
	if(processError.speedSensorError == 1 || processError.temperatureSensorError == 1 || processError.inductionBoardError == 1)
	{
		processError.processStopError = 1;
	}
	if(processError.processStopError == 1 && processError.errorNumberAndroid <= 15)
	{
		Send_Error_Msg(processError.errorNumberAndroid);
		dutyCycle = 0;
		drumMotor = dcMotorInit;
		#if	BS84C12A_DRIVE_ENABLE == 1
			  if(displayData != 'X')
			  {
				  dacSetVoltage(INDUCTION_OFF_COUNT, 0);
			  }
		#elif TM1668_DRIVE_ENABLE == 1
			  if(((uartPrintArr[2] != 'F' && uartPrintArr[3] != 'F' ) || (uartPrintArr[1] == '0' && uartPrintArr[2] == 'N' )) && (uartPrintArr[1] != '-' && uartPrintArr[2] != '-'))
			  {
				 keyPressState = pressOnOff;
			  }
		#endif
	}
	else if(processError.errorNumberAndroid > 15)
	{
		processError.processStopError = 0;
	}
}
void speedSensorErrorCheck(void)
{
	  if(androidProcessStruct.startofCooking == 1)
	  {
		  if(timerCnt.speedSensorErrorCnt++ >= SENSOR_ERROR_CHECK_CNT)
		  {
			  avg_speed = 0;
	#if 0
		#if	BS84C12A_DRIVE_ENABLE == 1
			  if(displayData != 'X')
			  {
				  dacSetVoltage(INDUCTION_OFF_COUNT, 0);
			  }
		#elif TM1668_DRIVE_ENABLE == 1
			  if(((uartPrintArr[2] != 'F' && uartPrintArr[3] != 'F' ) || (uartPrintArr[1] == '0' && uartPrintArr[2] == 'N' )) && (uartPrintArr[1] != '-' && uartPrintArr[2] != '-'))
			  {
				 keyPressState = pressOnOff;
			  }
		#endif
	#endif
			  processError.errorNumberAndroid = speedSensorError;
			  processError.speedSensorError = 1;
		  }
	  }
	  else
	  {
		  if(timerCnt.speedSensorErrorCnt++ >= 80)
		  {
			  avg_speed = 0;
		  }
	  }
}
/* @brief:  Function to assign the machine parameter values
 * @params: NOTHING
 * @return: NOTHING*/
void parameterValueAssignment(void)
{
#if ANDROID_MC_SETTINGS == 1
	if(machineSettings.temperatureLevel1 == 0)
	{
		deltaTempStruct.deltaLevel1 = DEFAULT_DELTA_TEMP1;
	}
	else
	{
		deltaTempStruct.deltaLevel1 = machineSettings.temperatureLevel1;
	}
	if(machineSettings.temperatureLevel2 == 0)
	{
		deltaTempStruct.deltaLevel2 = DEFAULT_DELTA_TEMP2;
	}
	else
	{
		deltaTempStruct.deltaLevel2 = machineSettings.temperatureLevel2;
	}
	if(machineSettings.temperatureLevel3 == 0)
	{
		deltaTempStruct.deltaLevel3 = DEFAULT_DELTA_TEMP3;
	}
	else
	{
		deltaTempStruct.deltaLevel3 = machineSettings.temperatureLevel3;
	}
	if(machineSettings.temperatureLevel4 == 0)
	{
		deltaTempStruct.deltaLevel4 = DEFAULT_DELTA_TEMP4;
	}
	else
	{
		deltaTempStruct.deltaLevel4 = machineSettings.temperatureLevel4;
	}
	if(machineSettings.temperatureLevel5 == 0)
	{
		deltaTempStruct.deltaLevel5 = DEFAULT_DELTA_TEMP5;
	}
	else
	{
		deltaTempStruct.deltaLevel5 = machineSettings.temperatureLevel5;
	}
	if(machineSettings.temperatureLevel6 == 0)
	{
		deltaTempStruct.deltaLevel6 = DEFAULT_DELTA_TEMP6;
	}
	else
	{
		deltaTempStruct.deltaLevel6 = machineSettings.temperatureLevel6;
	}
	if(machineSettings.temperatureLevel7 == 0)
	{
		deltaTempStruct.deltaLevel7 = DEFAULT_DELTA_TEMP7;
	}
	else
	{
		deltaTempStruct.deltaLevel7 = machineSettings.temperatureLevel7;
	}
	if(machineSettings.temperatureLevel8 == 0)
	{
		deltaTempStruct.deltaLevel8 = DEFAULT_DELTA_TEMP8;
	}
	else
	{
		deltaTempStruct.deltaLevel8 = machineSettings.temperatureLevel8;
	}
	if(machineSettings.temperatureLevel9 == 0)
	{
		deltaTempStruct.deltaLevel9 = DEFAULT_DELTA_TEMP9;
	}
	else
	{
		deltaTempStruct.deltaLevel9 = machineSettings.temperatureLevel9;
	}
//	deltaTempStruct.deltaLevel2 = machineSettings.temperatureLevel2;
//	deltaTempStruct.deltaLevel3 = machineSettings.temperatureLevel3;
//	deltaTempStruct.deltaLevel4 = machineSettings.temperatureLevel4;
//	deltaTempStruct.deltaLevel5 = machineSettings.temperatureLevel5;
//	deltaTempStruct.deltaLevel6 = machineSettings.temperatureLevel6;
//	deltaTempStruct.deltaLevel7 = machineSettings.temperatureLevel7;
//	deltaTempStruct.deltaLevel8 = machineSettings.temperatureLevel8;
//	deltaTempStruct.deltaLevel9 = machineSettings.temperatureLevel9;

	if(machineSettings.wattageLevel1 == 0)
	{
		deltaTempStruct.deltaWattageLevel1 = DEFAULT_DELTA_WATT1;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel1 = machineSettings.wattageLevel1;
	}
	if(machineSettings.wattageLevel2 == 0)
	{
		deltaTempStruct.deltaWattageLevel2 = DEFAULT_DELTA_WATT2;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel2 = machineSettings.wattageLevel2;
	}
	if(machineSettings.wattageLevel3 == 0)
	{
		deltaTempStruct.deltaWattageLevel3 = DEFAULT_DELTA_WATT3;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel3 = machineSettings.wattageLevel3;
	}
	if(machineSettings.wattageLevel4 == 0)
	{
		deltaTempStruct.deltaWattageLevel4 = DEFAULT_DELTA_WATT4;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel4 = machineSettings.wattageLevel4;
	}
	if(machineSettings.wattageLevel5 == 0)
	{
		deltaTempStruct.deltaWattageLevel5 = DEFAULT_DELTA_WATT5;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel5 = machineSettings.wattageLevel5;
	}
	if(machineSettings.wattageLevel6 == 0)
	{
		deltaTempStruct.deltaWattageLevel6 = DEFAULT_DELTA_WATT6;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel6 = machineSettings.wattageLevel6;
	}
	if(machineSettings.wattageLevel7 == 0)
	{
		deltaTempStruct.deltaWattageLevel7 = DEFAULT_DELTA_WATT7;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel7 = machineSettings.wattageLevel7;
	}
	if(machineSettings.wattageLevel8 == 0)
	{
		deltaTempStruct.deltaWattageLevel8 = DEFAULT_DELTA_WATT8;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel8 = machineSettings.wattageLevel8;
	}
	if(machineSettings.wattageLevel9 == 0)
	{
		deltaTempStruct.deltaWattageLevel9 = DEFAULT_DELTA_WATT9;
	}
	else
	{
		deltaTempStruct.deltaWattageLevel9 = machineSettings.wattageLevel9;
	}
	/*deltaTempStruct.deltaWattageLevel1 = machineSettings.wattageLevel1;
	deltaTempStruct.deltaWattageLevel2 = machineSettings.wattageLevel2;
	deltaTempStruct.deltaWattageLevel3 = machineSettings.wattageLevel3;
	deltaTempStruct.deltaWattageLevel4 = machineSettings.wattageLevel4;
	deltaTempStruct.deltaWattageLevel5 = machineSettings.wattageLevel5;
	deltaTempStruct.deltaWattageLevel6 = machineSettings.wattageLevel6;
	deltaTempStruct.deltaWattageLevel7 = machineSettings.wattageLevel7;
	deltaTempStruct.deltaWattageLevel8 = machineSettings.wattageLevel8;
	deltaTempStruct.deltaWattageLevel9 = machineSettings.wattageLevel9;*/

	if(miscellaneousSetting.defaultTemperatureAckAuto == 0)
	{
		miscellaneousSetting.defaultTemperatureAckAuto = DEFAULT_TEMP_ACK_AUTO;
	}
	else
	{
		miscellaneousSetting.defaultTemperatureAckAuto = miscellaneousSetting.defaultTemperatureAckAuto;
	}
	if(miscellaneousSetting.defaultTemperatureAckManual == 0)
	{
		miscellaneousSetting.defaultTemperatureAckManual = DEFAULT_TEMP_ACK_MANUAL;
	}
	else
	{
		miscellaneousSetting.defaultTemperatureAckManual = miscellaneousSetting.defaultTemperatureAckManual;
	}
//	machineSettings.defaultTemperatureAckAuto = machineSettings.defaultTemperatureAckAuto;
//	machineSettings.defaultTemperatureAckManual = machineSettings.defaultTemperatureAckManual;
	temperatureSensorOffset.positiveTemperatureOffsetValue = temperatureSensorOffset.positiveTemperatureOffsetValue;
	temperatureSensorOffset.negativeTemperatureOffsetValue = temperatureSensorOffset.negativeTemperatureOffsetValue;
	miscellaneousSetting.speedSensorEnable = miscellaneousSetting.speedSensorEnable;
	if(miscellaneousSetting.speedSensorCorrectionFactor == 0)
	{
		miscellaneousSetting.speedSensorCorrectionFactor = DEFAULT_SPEED_CORRECTION;
	}
	else
	{
		miscellaneousSetting.speedSensorCorrectionFactor = miscellaneousSetting.speedSensorCorrectionFactor;
	}
//	machineSettings.speedSensorCorrectionFactor = machineSettings.speedSensorCorrectionFactor;
	if(miscellaneousSetting.defaultMotorDutyCycleAuto == 0)
	{
		miscellaneousSetting.defaultMotorDutyCycleAuto = DEFAULT_AUTO_DC_LEVEL;
	}
	else
	{
		miscellaneousSetting.defaultMotorDutyCycleAuto = miscellaneousSetting.defaultMotorDutyCycleAuto;
	}
	if(miscellaneousSetting.defaultMotorDutyCycleManual == 0)
	{
		miscellaneousSetting.defaultMotorDutyCycleManual = DEFAULT_MANUAL_DC_LEVEL;
	}
	else
	{
		miscellaneousSetting.defaultMotorDutyCycleManual = miscellaneousSetting.defaultMotorDutyCycleManual;
	}
//	machineSettings.defaultMotorDutyCycleAuto = machineSettings.defaultMotorDutyCycleAuto;
//	machineSettings.defaultMotorDutyCycleManual = machineSettings.defaultMotorDutyCycleManual;


	if(miscellaneousSetting.endOfDayCleaningTime == 0)
	{
		miscellaneousSetting.endOfDayCleaningTime = DEFAULT_EOD_TIME / TIME_500MS_IN_SEC;
	}
	else
	{
		miscellaneousSetting.endOfDayCleaningTime = (miscellaneousSetting.endOfDayCleaningTime / TIME_500MS_IN_SEC);
	}
	if(miscellaneousSetting.endOfRecipeCleaningTime == 0)
	{
		miscellaneousSetting.endOfRecipeCleaningTime = DEFAULT_EOR_TIME / TIME_500MS_IN_SEC;
	}
	else
	{
		miscellaneousSetting.endOfRecipeCleaningTime = (miscellaneousSetting.endOfRecipeCleaningTime / TIME_500MS_IN_SEC);
	}
	if(miscellaneousSetting.cleanMotorSpeed == 0)
	{
		miscellaneousSetting.cleanMotorSpeed = DEFAULT_MOTOR_CLEANING;
	}
	else
	{
		miscellaneousSetting.cleanMotorSpeed = miscellaneousSetting.cleanMotorSpeed;
	}
//	machineSettings.endOfDayCleaningTime = (machineSettings.endOfDayCleaningTime / 0.5);
//	machineSettings.endOfRecipeCleaningTime = (machineSettings.endOfRecipeCleaningTime / 0.5);
//	machineSettings.cleanMotorSpeed = machineSettings.cleanMotorSpeed;


	if(miscellaneousSetting.speedSensorCorrectionFactor > 0)
	{
		speedCorrectionFactor = ((float) miscellaneousSetting.speedSensorCorrectionFactor / (float)10);
	}
	if(miscellaneousSetting.defaultTemperatureAckAuto > 0)
	{
		temperatureACKAuto = ((float)miscellaneousSetting.defaultTemperatureAckAuto / (float)100);
		temperatureACKAuto = (1 - temperatureACKAuto);
	}
	if(miscellaneousSetting.defaultTemperatureAckManual > 0)
	{
		temperatureACKManual = ((float)miscellaneousSetting.defaultTemperatureAckManual / (float)100);
		temperatureACKManual = (1 - temperatureACKManual);
	}
#elif ANDROID_MC_SETTINGS == 0
	deltaTempStruct.deltaLevel1 = DEFAULT_DELTA_TEMP1;
	deltaTempStruct.deltaLevel2 = DEFAULT_DELTA_TEMP2;
	deltaTempStruct.deltaLevel3 = DEFAULT_DELTA_TEMP3;
	deltaTempStruct.deltaLevel4 = DEFAULT_DELTA_TEMP4;
	deltaTempStruct.deltaLevel5 = DEFAULT_DELTA_TEMP5;
	deltaTempStruct.deltaLevel6 = DEFAULT_DELTA_TEMP6;
	deltaTempStruct.deltaLevel7 = DEFAULT_DELTA_TEMP7;
	deltaTempStruct.deltaLevel8 = DEFAULT_DELTA_TEMP8;
	deltaTempStruct.deltaLevel9 = DEFAULT_DELTA_TEMP9;
	deltaTempStruct.deltaWattageLevel1 = DEFAULT_DELTA_WATT1;
	deltaTempStruct.deltaWattageLevel2 = DEFAULT_DELTA_WATT2;
	deltaTempStruct.deltaWattageLevel3 = DEFAULT_DELTA_WATT3;
	deltaTempStruct.deltaWattageLevel4 = DEFAULT_DELTA_WATT4;
	deltaTempStruct.deltaWattageLevel5 = DEFAULT_DELTA_WATT5;
	deltaTempStruct.deltaWattageLevel6 = DEFAULT_DELTA_WATT6;
	deltaTempStruct.deltaWattageLevel7 = DEFAULT_DELTA_WATT7;
	deltaTempStruct.deltaWattageLevel8 = DEFAULT_DELTA_WATT8;
	deltaTempStruct.deltaWattageLevel9 = DEFAULT_DELTA_WATT9;

	miscellaneousSetting.defaultTemperatureAckAuto = DEFAULT_TEMP_ACK_AUTO;				//in %
	miscellaneousSetting.defaultTemperatureAckManual = DEFAULT_TEMP_ACK_MANUAL;			//in %
	temperatureSensorOffset.positiveTemperatureOffsetValue = DEFAULT_TEMP_POS_OFFSET;
	temperatureSensorOffset.negativeTemperatureOffsetValue = DEFAULT_TEMP_NEG_OFFSET;
	miscellaneousSetting.speedSensorEnable = DEFAULT_SPEED_SENSOR_ENABLE;
	miscellaneousSetting.speedSensorCorrectionFactor = DEFAULT_SPEED_CORRECTION;
	miscellaneousSetting.defaultMotorDutyCycleAuto = DEFAULT_AUTO_DC_LEVEL;
	miscellaneousSetting.defaultMotorDutyCycleManual = DEFAULT_MANUAL_DC_LEVEL;
	miscellaneousSetting.endOfDayCleaningTime = (DEFAULT_EOD_TIME / TIME_500MS_IN_SEC);
	miscellaneousSetting.endOfRecipeCleaningTime = (DEFAULT_EOR_TIME / TIME_500MS_IN_SEC);
	miscellaneousSetting.cleanMotorSpeed = DEFAULT_MOTOR_CLEANING;

	speedCorrectionFactor = ((float) miscellaneousSetting.speedSensorCorrectionFactor / (float)10);
	temperatureACKAuto = ((float)miscellaneousSetting.defaultTemperatureAckAuto / (float)100);
	temperatureACKManual = ((float)miscellaneousSetting.defaultTemperatureAckManual / (float)100);

	temperatureACKAuto = (1 - temperatureACKAuto);
	temperatureACKManual = (1 - temperatureACKManual);
#endif

}
/* @brief:  Function to return  the speed value after averaging
 * @params: NOTHING
 * @return: Speed value in RPM*/
#if SPEED_SENSOR_ENABLE == 1
float getSpeed(void)
{
	const int tim_clock = 10000;										//Timer 3 configured for 1us resolution
	const int sec_value = 60;
	sum_tick = 0;
	avg_tick = 0;
	/* Convert the time tick value into frequency,
	 * avg_tick is the time difference period of  2 pulses. Since ticks are in resolution of 1us
	 * (1000000 / tick) -> frequency for 2 tooth
	 * F = (1000000/tick * 65)
	 * */
	gu32_Freq = ((float)(tim_clock)/(gu32_Ticks));						//Calculate the Frequency
	speed_ic = (float)(gu32_Freq * sec_value) ;							//Calculate the speed value
	return speed_ic;
}
#endif
#if TM1668_DRIVE_ENABLE == 1
/**
  * @brief  Function to Segregate the packet received from induction for valid start & end of packet
  * @params	NOTHING
  * @retval Return Success/Failure
  */
uint8_t segregatePacket(void)
{
	for(startIndex = 0; startIndex < MAX_ARRAY_SIZE; startIndex++)
	{
		if(uartTxBuff[startIndex] == DISPLAY_SOF)
		{
			soi = startIndex;
			while(uartTxBuff[startIndex] != DISPLAY_EOF)
			{
				filterBuff[copyStartIndex] = uartTxBuff[startIndex];
				if((startIndex - soi) >= 60)
				{
					soi = 0;
					copyStartIndex = 0;
					startIndex = 0;
					break;
				}
				startIndex++;
				copyStartIndex++;
			}
			filterBuff[++copyStartIndex] = DISPLAY_EOF;
			return 1;
		}
	}
	return 0;
}
/**
  * @brief  Function to Get the data on the Display with TM1668
  * @params	NOTHING
  * @retval Return Success
  */
#if WOKIE_GREEN_V_1_5_EN == 1
/* @brief:  Function to check the data being displayed on Display with Green-V1.5 brd
 * @params: NOTHING
 * @return: success*/
state checkForDisplayValue_new_green(void)
{
	memset(uartPrintArr, 0, sizeof(uartPrintArr));
//	if(filterBuff[FIRST_SEG_VALID_INDEX-7] == FIRST_SEG_VALUE_NG)
	{
		for(uint8_t firstSeg = 0; firstSeg < 15; firstSeg++)
		{
			if(filterBuff[FIRST_SEG_DATA_INDEX-7] == displayHexArray[firstSeg])
			{
				uartPrintArr[0] = tm1668Display[firstSeg];
				break;
			}
			else
			{
				uartPrintArr[0] = '-';
			}
		}
	}
//	if(filterBuff[SECOND_SEG_VALID_INDEX-11] == SECOND_SEG_VALUE_NG)
	{
		for(uint8_t secondSeg = 0; secondSeg < 15; secondSeg++)
		{
			if(filterBuff[SECOND_SEG_DATA_INDEX-11] == displayHexArray[secondSeg])
			{
				uartPrintArr[1] = tm1668Display[secondSeg];
				break;
			}
			else
			{
				uartPrintArr[1] = '-';
			}
		}
	}
//	if(filterBuff[THIRD_SEG_VALID_INDEX-15] == THIRD_SEG_VALUE_NG)
	{
		for(uint8_t thirdSeg = 0; thirdSeg < 15; thirdSeg++)
		{
			if(filterBuff[THIRD_SEG_DATA_INDEX-15] == displayHexArray[thirdSeg])
			{
				uartPrintArr[2] = tm1668Display[thirdSeg];
				break;
			}
			else
			{
				uartPrintArr[2] = '-';
			}
		}
	}
//	if(filterBuff[FOURTH_SEG_VALID_INDEX-19] == FOURTH_SEG_VALUE_NG)
	{
		for(uint8_t fourthSeg = 0; fourthSeg < 15; fourthSeg++)
		{
			if(filterBuff[FOURTH_SEG_DATA_INDEX-19] == displayHexArray[fourthSeg])
			{
				uartPrintArr[3] = tm1668Display[fourthSeg];
				break;
			}
			else
			{
				uartPrintArr[3] = '-';
			}
		}
	}
	if(uartPrintArr[0] == '\0' && uartPrintArr[1] == '\0' && uartPrintArr[2] == '\0' && uartPrintArr[3] == '\0')
	{
		uartPrintArr[0] = '-';
		uartPrintArr[1] = '-';
		uartPrintArr[2] = '-';
		uartPrintArr[3] = '-';
	}
	uartPrintArr[4] = '\r';
	uartPrintArr[5] = '\n';
	return processSuccess;
}
#elif WOKIE_GREEN_BRD_MC_EN == 1
/**
  * @brief  Function to Get the data on the Display with TM1668 for normal version of packet
  * @params	NOTHING
  * @retval Return Success
  */
state checkForDisplayValue(void)
{
	memset(uartPrintArr, 0, sizeof(uartPrintArr));
	if(filterBuff[FIRST_SEG_VALID_INDEX] == FIRST_SEG_VALUE)
	{
		for(uint8_t firstSeg = 0; firstSeg < 13; firstSeg++)
		{
			if(filterBuff[FIRST_SEG_DATA_INDEX] == displayHexArray[firstSeg])
			{
				uartPrintArr[0] = tm1668Display[firstSeg];
				break;
			}
			else
			{
				uartPrintArr[0] = '-';
			}
		}
	}
	if(filterBuff[SECOND_SEG_VALID_INDEX] == SECOND_SEG_VALUE)
	{
		for(uint8_t secondSeg = 0; secondSeg < 13; secondSeg++)
		{
			if(filterBuff[SECOND_SEG_DATA_INDEX] == displayHexArray[secondSeg])
			{
				uartPrintArr[1] = tm1668Display[secondSeg];
				break;
			}
			else
			{
				uartPrintArr[1] = '-';
			}
		}
	}
	if(filterBuff[THIRD_SEG_VALID_INDEX] == THIRD_SEG_VALUE)
	{
		for(uint8_t thirdSeg = 0; thirdSeg < 13; thirdSeg++)
		{
			if(filterBuff[THIRD_SEG_DATA_INDEX] == displayHexArray[thirdSeg])
			{
				uartPrintArr[2] = tm1668Display[thirdSeg];
				break;
			}
			else
			{
				uartPrintArr[2] = '-';
			}
		}
	}
	if(filterBuff[FOURTH_SEG_VALID_INDEX] == FOURTH_SEG_VALUE)
	{
		for(uint8_t fourthSeg = 0; fourthSeg < 13; fourthSeg++)
		{
			if(filterBuff[FOURTH_SEG_DATA_INDEX] == displayHexArray[fourthSeg])
			{
				uartPrintArr[3] = tm1668Display[fourthSeg];
				break;
			}
			else
			{
				uartPrintArr[3] = '-';
			}
		}
	}
	if(uartPrintArr[0] == '\0' && uartPrintArr[1] == '\0' && uartPrintArr[2] == '\0' && uartPrintArr[3] == '\0')
	{
		uartPrintArr[0] = '-';
		uartPrintArr[1] = '-';
		uartPrintArr[2] = '-';
		uartPrintArr[3] = '-';
	}
	uartPrintArr[4] = '\r';
	uartPrintArr[5] = '\n';
	return processSuccess;
}
/**
  * @brief  Function to Get the data on the Display with TM1668 for shorter version of packet
  * @params	NOTHING
  * @retval Return Success
  */
state checkForDisplayValueShortPacket(void)
{
	memset(uartPrintArr, 0, sizeof(uartPrintArr));
	if(filterBuff[FIRST_SEG_VALID_INDEX - 5] == FIRST_SEG_VALUE)
	{
		for(uint8_t firstSeg = 0; firstSeg < 13; firstSeg++)
		{
			if(filterBuff[FIRST_SEG_DATA_INDEX - 5] == displayHexArray[firstSeg])
			{
				uartPrintArr[0] = tm1668Display[firstSeg];
				break;
			}
			else
			{
				uartPrintArr[0] = '-';
			}
		}
	}
	if(filterBuff[SECOND_SEG_VALID_INDEX - 5] == SECOND_SEG_VALUE)
	{
		for(uint8_t secondSeg = 0; secondSeg < 13; secondSeg++)
		{
			if(filterBuff[SECOND_SEG_DATA_INDEX - 5] == displayHexArray[secondSeg])
			{
				uartPrintArr[1] = tm1668Display[secondSeg];
				break;
			}
			else
			{
				uartPrintArr[1] = '-';
			}
		}
	}
	if(filterBuff[THIRD_SEG_VALID_INDEX - 5] == THIRD_SEG_VALUE)
	{
		for(uint8_t thirdSeg = 0; thirdSeg < 13; thirdSeg++)
		{
			if(filterBuff[THIRD_SEG_DATA_INDEX - 5] == displayHexArray[thirdSeg])
			{
				uartPrintArr[2] = tm1668Display[thirdSeg];
				break;
			}
			else
			{
				uartPrintArr[2] = '-';
			}
		}
	}
	if(filterBuff[FOURTH_SEG_VALID_INDEX - 5] == FOURTH_SEG_VALUE)
	{
		for(uint8_t fourthSeg = 0; fourthSeg < 13; fourthSeg++)
		{
			if(filterBuff[FOURTH_SEG_DATA_INDEX - 5] == displayHexArray[fourthSeg])
			{
				uartPrintArr[3] = tm1668Display[fourthSeg];
				break;
			}
			else
			{
				uartPrintArr[3] = '-';
			}
		}
	}
	if(uartPrintArr[0] == '\0' && uartPrintArr[1] == '\0' && uartPrintArr[2] == '\0' && uartPrintArr[3] == '\0')
	{
		uartPrintArr[0] = '-';
		uartPrintArr[1] = '-';
		uartPrintArr[2] = '-';
		uartPrintArr[3] = '-';
	}
	uartPrintArr[4] = '\r';
	uartPrintArr[5] = '\n';
	return processSuccess;
}
#endif
/**
  * @brief  Function to get wattage level for the TM1668 based induction
  * @params	wattage level level to be set
  * @retval returns the equivalent wattage value to set
  */
uint16_t wattageGreenInLevels(uint8_t wattageLevel)
{
	switch(wattageLevel)
	{
		case 1:
			 return WATT_500;
			 break;
		case 2:
			 return WATT_1000;
			 break;
		case 3:
			 return WATT_1500;
			 break;
		case 4:
			 return WATT_2300;
			 break;
		case 5:
			 return WATT_2700;
			 break;
		case 6:
			 return WATT_3100;
			 break;
		case 7:
			 return WATT_3500;
			 break;
		case 8:
			 return WATT_4000;
			 break;
		case 9:
			 return WATT_4500;
			 break;
		case 10:
			 return WATT_5000;
			 break;
		default:
			return 0;
			break;
	}
}
/**
  * @brief  Function to Get the error value of induction
  * @params	NOTHING
  * @retval Return error number
  */
uint8_t getErrorNumber(void)
{
#if WOKIE_GREEN_BRD_MC_EN == 1
	uint8_t ind,  ret;
	for(ind = 0; ind < 10; ind++)
	{
		if(tm1668Display[ind] == uartPrintArr[2])
		{
			ret =  ind;
			break;
		}
	}
	if(ret >= 0 && ret < 10)
		return  ret;
	else
		return -1;
#elif WOKIE_GREEN_V_1_5_EN
	uint8_t ind,  ret;
	for(ind = 0; ind < 9; ind++)
	{
		if(tm1668Display[ind] == uartPrintArr[2])
		{
			ret =  ind;
			break;
		}
	}
	if(ret >= 0 && ret < 9)
		return  ret;
	else
		return -1;
#endif
}
/**
  * @brief  Function to Disable pin for a button press of TM1668
  * @params	NOTHING
  * @retval NOTHING
  */
void disablePinForPress(void)
{
  uartData = 0;
  inductionPins.onOffPin = 0;
  inductionPins.wattagePin = 0;
  inductionPins.temperaturePin = 0;
  inductionPins.incrementPin = 0;
  inductionPins.decrementPin = 0;
  HAL_GPIO_DeInit(GPIOE, INDUCTION_DECREMENT_Pin| INDUCTION_INCREMENT_Pin | INDUCTION_WATTAGE_Pin |
		  INDUCTION_ON_OFF_Pin | INDUCTION_TEMPERATURE_Pin | INDUCTION_SPARE_Pin);
//  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}
/**
  * @brief  Function to Enable pin for a button press of TM1668
  * @params	GPIO to be enabled
  * @retval NOTHING
  */
void enablePinForPress(uint16_t gpioPin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}
/**
  * @brief  Function to simulate a button press of TM1668
  * @params	button to be simulated
  * @retval NOTHING
  */
void inductionKeyPress(keyState_t buttonPressState)
{
	switch(keyPressState)
	{
		case pressIdle:
			  break;
		case pressOnOff:
			  inductionPins.onOffPin = 1;
			  inductionPins.wattagePin = 0;
			  inductionPins.temperaturePin = 0;
			  inductionPins.incrementPin = 0;
			  inductionPins.decrementPin = 0;
			  enablePinForPress(INDUCTION_ON_OFF_Pin);
			  keyPressState = pressComplete;
			  break;
		case pressWattage:
			  inductionPins.onOffPin = 0;
			  inductionPins.wattagePin = 1;
			  inductionPins.temperaturePin = 0;
			  inductionPins.incrementPin = 0;
			  inductionPins.decrementPin = 0;
			  enablePinForPress(INDUCTION_WATTAGE_Pin);
			  keyPressState = pressComplete;
			  break;
		case pressTemperature:
			  inductionPins.onOffPin = 0;
			  inductionPins.wattagePin = 0;
			  inductionPins.temperaturePin = 1;
			  inductionPins.incrementPin = 0;
			  inductionPins.decrementPin = 0;
			  enablePinForPress(INDUCTION_TEMPERATURE_Pin);
			  keyPressState = pressComplete;
			  break;
		case pressIncrement:
			  inductionPins.onOffPin = 0;
			  inductionPins.wattagePin = 0;
			  inductionPins.temperaturePin = 0;
			  inductionPins.incrementPin = 1;
			  inductionPins.decrementPin = 0;
			  enablePinForPress(INDUCTION_INCREMENT_Pin);
			  keyPressState = pressComplete;
			  break;
		case pressDecrement:
			  inductionPins.onOffPin = 0;
			  inductionPins.wattagePin = 0;
			  inductionPins.temperaturePin = 0;
			  inductionPins.incrementPin = 0;
			  inductionPins.decrementPin = 1;
			  enablePinForPress(INDUCTION_DECREMENT_Pin);
			  keyPressState = pressComplete;
			  break;
		case pressComplete:
			  disablePinForPress();
			  break;

	}
}
/**
  * @brief  Function to get the round-off wattage value to be set
  * @params	actual wattage
  * @retval returns the rounded wattage value to closest available
  */
uint16_t getWattageValueToSet(uint16_t wattageValue)
{
	uint16_t returnWattage;
	if((wattageValue < wattageLevelsTM1668[1] && wattageValue >= wattageLevelsTM1668[0]) || (wattageValue < wattageLevelsTM1668[0]))
	{
		returnWattage = wattageLevelsTM1668[0];
	}
	else if(wattageValue < wattageLevelsTM1668[2] && wattageValue >= wattageLevelsTM1668[1])
	{
		returnWattage = wattageLevelsTM1668[1];
	}
	else if(wattageValue < wattageLevelsTM1668[3] && wattageValue >= wattageLevelsTM1668[2])
	{
		returnWattage = wattageLevelsTM1668[2];
	}
	else if(wattageValue < wattageLevelsTM1668[4] && wattageValue >= wattageLevelsTM1668[3])
	{
		returnWattage = wattageLevelsTM1668[3];
	}
	else if(wattageValue < wattageLevelsTM1668[5] && wattageValue >= wattageLevelsTM1668[4])
	{
		returnWattage = wattageLevelsTM1668[4];
	}
	else if(wattageValue < wattageLevelsTM1668[6] && wattageValue >= wattageLevelsTM1668[5])
	{
		returnWattage = wattageLevelsTM1668[5];
	}
	else if(wattageValue < wattageLevelsTM1668[7] && wattageValue >= wattageLevelsTM1668[6])
	{
		returnWattage = wattageLevelsTM1668[6];
	}
	else if(wattageValue < wattageLevelsTM1668[8] && wattageValue >= wattageLevelsTM1668[7])
	{
		returnWattage = wattageLevelsTM1668[7];
	}
	else if(wattageValue < wattageLevelsTM1668[9] && wattageValue >= wattageLevelsTM1668[8])
	{
		returnWattage = wattageLevelsTM1668[8];
	}
	else if(wattageValue < wattageLevelsTM1668[10] && wattageValue >= wattageLevelsTM1668[9])
	{
		returnWattage = wattageLevelsTM1668[9];
	}
	else if(wattageValue < wattageLevelsTM1668[11] && wattageValue >= wattageLevelsTM1668[10])
	{
		returnWattage = wattageLevelsTM1668[10];
	}
	else if(wattageValue >= wattageLevelsTM1668[11])
	{
		returnWattage = wattageLevelsTM1668[11];
	}
	return returnWattage;
}
#elif BS84C12A_DRIVE_ENABLE == 1
/**
  * @brief  Function to get the 12-bit dac value to be set for MCP7425
  * @params	level of wattage
  * @retval returns the 12-bit count to be set with DAC for Wattage level mapped with the table
  */
uint16_t getWattageLevelCount_BS84C12A(uint8_t level)
{
	if(level >= 0 && level < 10)
	{
		return wattageLevelDAC[level];
	}
	else if(level > 10)
	{
		return 1;
	}
	else
		return 0;
}
/**
  * @brief  Function to get the character being displayed on the display
  * @params	charValue : value after getting the data from 7-segment display
  * @retval returns the character being display (or) value of Hex for characters
  */
uint8_t getDisplayChar_BS84C12A(uint8_t charValue)
{
	if(charValue == ZERO_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 1;//Disk or Drive failure Error
		}
		else
		{
			return '0';
		}
	}
	else if(charValue == ONE_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 2;//Surface Sensor open/short ckt Error
		}
		else
		{
			return '1';
		}
	}
	else if(charValue ==TWO_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 3;//IGBT open/close ckt Error
		}
		else
		{
			return '2';
		}
	}
	else if(charValue == THREE_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 4;//Mains is high Error
		}
		else
		{
			return '3';
		}
	}
	else if(charValue == FOUR_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 5;//Mains is low Error
		}
		else
		{
			return '4';
		}
	}
	else if(charValue == FIVE_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 6;//Furnace sensor over temperature Error
		}
		else
		{
			return '5';
		}
	}
	else if(charValue == SIX_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 7;//IGBT sensor over temperature Error
		}
		else
		{
			return '6';
		}
	}
	else if(charValue == SEVEN_BS84C12A)
	{
		return '7';
	}
	else if(charValue == EIGHT_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 9;//Communication Error
		}
		else
		{
			return '8';
		}
	}
	else if(charValue == NINE_BS84C12A)
	{
		return '9';
	}
	else if(charValue == ERROR_BS84C12A)
	{
		timerCnt.inductionErrorCnt++;
		if(timerCnt.inductionErrorCnt >= 10)
		{
			bs84c12aInductionError = 1;
			timerCnt.inductionErrorCnt = 0;
		}
		return 'E';
	}
	else if(charValue == DASH_BS84C12A)
	{
		if(bs84c12aInductionError == 1)
		{
			return 8;//Wok Error
		}
		else
		{
			return '-';
		}
	}
	else
	{
		return  'X';
	}
	#if GET_DISPLAY_WITH_ADC
	for(uint8_t i = 0; i < 7; i++)
	{
		adcDataDisplaySegments[i] = ADC1_DATA[i];
		displayVtgFromADC[i] = adcDataDisplaySegments[i] * ADC_COUNTS_TO_VTG;
	}
	if(displayVtgFromADC[0] >= DISP_SEGMENT_HIGH_VAL)
	{
		charValue |= (1 << 0);
	}
	else
	{
		charValue &= ~(1 << 0);
	}
	if(displayVtgFromADC[1] >= DISP_SEGMENT_HIGH_VAL)
	{
		charValue |= (1 << 1);
	}
	else
	{
		charValue &= ~(1 << 1);
	}
	if(displayVtgFromADC[2] >= DISP_SEGMENT_HIGH_VAL)
	{
		charValue |= (1 << 2);
	}
	else
	{
		charValue &= ~(1 << 2);
	}
	if(displayVtgFromADC[3] >= DISP_SEGMENT_HIGH_VAL)
	{
		charValue |= (1 << 3);
	}
	else
	{
		charValue &= ~(1 << 3);
	}
	if(displayVtgFromADC[4] >= DISP_SEGMENT_HIGH_VAL)
	{
		charValue |= (1 << 4);
	}
	else
	{
		charValue &= ~(1 << 4);
	}
	if(displayVtgFromADC[5] >= DISP_SEGMENT_HIGH_VAL)
	{
		charValue |= (1 << 5);
	}
	else
	{
		charValue &= ~(1 << 5);
	}
	if(displayVtgFromADC[6] >= DISP_SEGMENT_HIGH_VAL)
	{
		charValue |= (1 << 6);
	}
	else
	{
		charValue &= ~(1 << 6);
	}
	charValue &= ~(1 << 7);
	#endif
}
#endif

/**
  * @brief  Function to get dutycycle for the dc motor
  * @params	dc motor level to be set
  * @retval returns the dutycycle to be set
  */
uint8_t getDCMotorlevels(uint8_t level)
{
	switch(level)
	{
		case 0:
			 return 0;
			 break;
		case 1:
			 return 30;
			 break;
		case 2:
			 return 45;
			 break;
		case 3:
			 return 60;
			 break;
		case 4:
			 return 80;
			 break;
		case 5:
			 return 100;
			 break;
		default:
			return 0;
			break;
	}
}
/**
  * @brief  Function to service the data for wokie machine parameter controls, like induction on/off, motor on/off, etc
  * @params	cmd_id1 : ID for controlling parameter
  * 		data : data to be set (for e.g. motor level, wattage level, temperature value)
  * @retval NOTHING
  */
void Process_WOKIE_Control(uint8_t cmd_id1, uint16_t  data)
{
	if(cmd_id1 == HEATER_ON_WATT)//level 1 - 10
	{
		 wattageMode.valueFromAndroid  = data;
#if TM1668_DRIVE_ENABLE == 1
		 wattageMode.valueToSet = wattageGreenInLevels(wattageMode.valueFromAndroid);
		 if(wattageMode.valueToSet >= 500)
		 {
			 initialOnOffCnt = 0;
		 }
#elif BS84C12A_DRIVE_ENABLE == 1
		 wattageLevel_BS84C12A = wattageMode.valueFromAndroid;
		 wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(wattageLevel_BS84C12A);
//		  dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
#endif
		 inductionMode.wattageMode = 1;
		 inductionMode.temperatureMode = 0;
		 Send_ADR_Cmd();
		 osDelay(10);
		 memset(and_ack, 0, 64);
	}
	else if(cmd_id1 == HEATER_ON_TEMP)
	{
#if		 TM1668_DRIVE_ENABLE == 1
		 initialOnOffCnt = 0;
#endif
		 wattageMode.valueToSet = 0;
		 inductionMode.temperatureMode = 1;
		 inductionMode.wattageMode = 0;
		 temperatureMode.valueFromAndroid  = data;
		 temperatureMode.valueToSet = temperatureMode.valueFromAndroid;
		 if(androidProcessStruct.sendTempAckOnce == 0 && androidProcessStruct.manualMode == 1/* && autoMode == 0*/)
		 {
			 temp_raise = 1;
			 Send_temp_Response(INSTRUCTION_SET,2,1);
			 osDelay(15);
		 }
		 else if(androidProcessStruct.sendTempAckOnce == 1)
		 {
			 Send_temp_Response(INSTRUCTION_SET,2,3);
			 osDelay(15);
		 }
	}
	else if(cmd_id1==ROTATE_DRUM)
	{
		 dutyCycle = getDCMotorlevels(data);
		 drumMotor = dcMotorInit;
		 Send_ADR_Cmd();
		 osDelay(15);
		 memset(and_ack, 0, 64);
	}
	else if(cmd_id1==HEATER_OFF)
	{
#if BS84C12A_DRIVE_ENABLE
		 wattageLevel_BS84C12A=0;
		 wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(wattageLevel_BS84C12A);
		 dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
#else
		if(uartPrintArr[2] != 'F' &&uartPrintArr[3] != 'F' )
		{
			keyPressState = pressOnOff;
		}
#endif
		Send_ADR_Cmd();
		osDelay(15);
		memset(and_ack, 0, 64);
	}
	else if(cmd_id1==ROTATION_OFF)
	{
		dutyCycle = 0;
		drumMotor = dcMotorInit;
		Send_ADR_Cmd();
		osDelay(15);
		memset(and_ack, 0, 64);
	}
	else if(cmd_id1==TOSS)
	{
		Send_ADR_Cmd();
		osDelay(15);
		memset(and_ack, 0, 64);
	}
}
/**
  * @brief  Function to control the temperature based on the parameters passed
  * @params	setTemperatureValue : Set temperature Value
  * 		currentTemperature : Current temperature of Drum
  * @retval NOTHING
  */
void temperatureControlCurve(uint16_t setTemperatureValue, uint16_t currentTemperature)
{
#if	DELTA_METHOD == 1
	deltaTempStruct.deltaTemperature = (setTemperatureValue - currentTemperature);
	if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel1  && deltaTempStruct.deltaTemperature <= 300)
	{
		temperatureLevelIndicate = level1;
#if			TM1668_DRIVE_ENABLE
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel1);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(deltaTempStruct.deltaWattageLevel1);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel2 && deltaTempStruct.deltaTemperature < deltaTempStruct.deltaLevel1 && deltaTempStruct.deltaTemperature <= 300)
	{
		temperatureLevelIndicate = level2;
#if			TM1668_DRIVE_ENABLE
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel2);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(deltaTempStruct.deltaWattageLevel2);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel3 && deltaTempStruct.deltaTemperature < deltaTempStruct.deltaLevel2 && deltaTempStruct.deltaTemperature <= 300)
	{
		temperatureLevelIndicate = level3;
#if			TM1668_DRIVE_ENABLE
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel3);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(deltaTempStruct.deltaWattageLevel3);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel4 && deltaTempStruct.deltaTemperature < deltaTempStruct.deltaLevel3 && deltaTempStruct.deltaTemperature <= 300)
	{
		temperatureLevelIndicate = level4;
#if			TM1668_DRIVE_ENABLE
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel4);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(deltaTempStruct.deltaWattageLevel4);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel5 && deltaTempStruct.deltaTemperature < deltaTempStruct.deltaLevel4 && deltaTempStruct.deltaTemperature <= 300)
	{
		temperatureLevelIndicate = level5;
#if			TM1668_DRIVE_ENABLE
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel5);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(deltaTempStruct.deltaWattageLevel5);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel6 && deltaTempStruct.deltaTemperature < deltaTempStruct.deltaLevel5 && deltaTempStruct.deltaTemperature <= 300)
	{
		temperatureLevelIndicate = level6;
#if			TM1668_DRIVE_ENABLE
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel6);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(deltaTempStruct.deltaWattageLevel6);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel7 && deltaTempStruct.deltaTemperature < deltaTempStruct.deltaLevel6 && deltaTempStruct.deltaTemperature <= 300)
	{
		temperatureLevelIndicate = level7;
#if			TM1668_DRIVE_ENABLE
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel7);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(deltaTempStruct.deltaWattageLevel7);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel8 && deltaTempStruct.deltaTemperature < deltaTempStruct.deltaLevel7 && deltaTempStruct.deltaTemperature <= 300)
	{
#if 0
		if(temp_raise == 1 && autoMode == 0 && androidProcessStruct.temperatureAutoMode == 0)
		{
			androidProcessStruct.sendTempAckOnce = 1;
			temp_raise=0;
			Send_temp_Response(INSTRUCTION_SET,2,3);
			osDelay(15);
		}
#endif
		temperatureLevelIndicate = level8;
#if			TM1668_DRIVE_ENABLE
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel8);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(deltaTempStruct.deltaWattageLevel8);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(deltaTempStruct.deltaTemperature >= deltaTempStruct.deltaLevel9 && deltaTempStruct.deltaTemperature < deltaTempStruct.deltaLevel8 && deltaTempStruct.deltaTemperature <= 300)
	{
		temperatureLevelIndicate = level9;
#if			TM1668_DRIVE_ENABLE
//		keyPressState = pressOnOff;
		wattageMode.valueToSet = wattageGreenInLevels(deltaTempStruct.deltaWattageLevel9);
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(0);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	else if(currentTemperature >= (setTemperatureValue + 5))
	{
		temperatureLevelIndicate = level10;
#if			TM1668_DRIVE_ENABLE
		keyPressState = pressOnOff;
#elif		BS84C12A_DRIVE_ENABLE == 1
		wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(0);
		dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
#endif
	}
	if(temperatureACKAuto > 0 && temperatureACKAuto < 1)
	{
		if(PERCENT_CALCULATION(setTemperatureValue, currentTemperature) < temperatureACKAuto)
		{
			if(androidProcessStruct.temperatureAutoMode == 1)
			{
				Send_Status_data();
				osDelay(10);
				androidProcessStruct.temperatureAutoMode = 0;
//				androidProcessStruct.manualMode = 1;
			}
		}
	}
	if(temperatureACKManual > 0 && temperatureACKManual < 1)
	{
		if(PERCENT_CALCULATION(setTemperatureValue, currentTemperature) < temperatureACKManual)
		{
			if(androidProcessStruct.sendTempAckOnce == 0 && androidProcessStruct.manualMode == 1)
			{
				Send_temp_Response(INSTRUCTION_SET,2,3);
				osDelay(15);
				androidProcessStruct.sendTempAckOnce = 1;
			}
		}
	}


#elif DELTA_METHOD == 0

	if(setTemperatureValue >= 80 && setTemperatureValue <= 260)
	{
		if(currentTemperature > 25 && currentTemperature < 125 && setTemperatureValue > 125)
		{
			temperatureLevelIndicate = level1;
#if			TM1668_DRIVE_ENABLE
			wattageMode.valueToSet = wattageGreenInLevels(9);
#elif		BS84C12A_DRIVE_ENABLE == 1
			wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(8);
			dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
#endif
		}
		if(currentTemperature > 126 && currentTemperature < 175 && setTemperatureValue > 170)
		{
			temperatureLevelIndicate = level2;
#if			TM1668_DRIVE_ENABLE
			wattageMode.valueToSet = wattageGreenInLevels(8);
#elif		BS84C12A_DRIVE_ENABLE == 1
			wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(7);
			dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
#endif
		}
		if(currentTemperature > 176 && currentTemperature < 220 && setTemperatureValue > 220)
		{
			temperatureLevelIndicate = level3;
#if			TM1668_DRIVE_ENABLE
			wattageMode.valueToSet = wattageGreenInLevels(7);
#elif		BS84C12A_DRIVE_ENABLE == 1
			wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(6);
			dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
#endif
		}
		if(currentTemperature > 221 && currentTemperature < 230 && setTemperatureValue > 230)
		{
			temperatureLevelIndicate = level4;
#if			TM1668_DRIVE_ENABLE
			wattageMode.valueToSet = wattageGreenInLevels(6);
#elif		BS84C12A_DRIVE_ENABLE == 1
			wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(5);
			dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
#endif
		}
		if(currentTemperature > 231 && currentTemperature < 240 && setTemperatureValue > 235)
		{
			temperatureLevelIndicate = level5;
#if			TM1668_DRIVE_ENABLE
			wattageMode.valueToSet = wattageGreenInLevels(5);
#elif		BS84C12A_DRIVE_ENABLE == 1
			wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(4);
			dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
#endif
		}
		if(currentTemperature > (setTemperatureValue - 9) && currentTemperature < (setTemperatureValue - 4))
		{
			if(temp_raise == 1 && autoMode == 0)
			{
				androidProcessStruct.sendTempAckOnce = 1;
				temp_raise=0;
				Send_temp_Response(INSTRUCTION_SET,2,3);
				osDelay(15);
			}
			temperatureLevelIndicate = level6;
#if			TM1668_DRIVE_ENABLE
			wattageMode.valueToSet = wattageGreenInLevels(4);
#elif		BS84C12A_DRIVE_ENABLE == 1
			wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(3);
			dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
#endif
		}
		if(currentTemperature > (setTemperatureValue - 2))
		{
			keyPressState = pressOnOff;
#if			0
			if(temp_raise == 1)
			{
				androidProcessStruct.sendTempAckOnce = 1;
				Send_temp_Response(INSTRUCTION_SET,2,3);
				temp_raise=0;
				osDelay(15);
			}
#endif
		}
//		if(PERCENT_CALCULATION(setTemperatureValue, currentTemperature) < TEMP_ACK_PERCENT)
		if(currentTemperature >= 120)
		{
			if(androidProcessStruct.temperatureAutoMode == 1)
			{
				Send_Status_data();
				androidProcessStruct.temperatureAutoMode = 0;
				androidProcessStruct.manualMode = 1;
			}
			else if(androidProcessStruct.sendTempAckOnce == 0 && androidProcessStruct.temperatureAutoMode == 0 && autoMode == 0)
			{
				Send_temp_Response(INSTRUCTION_SET,2,3);
				osDelay(15);
				androidProcessStruct.sendTempAckOnce = 1;
			}
		}
	}
#endif
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
//  PWM_Initialize(drumDCMotor);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);


#if ANDROID_MC_SETTINGS == 0
  parameterValueAssignment();
#endif

#if DC_MOTOR_TASK_ENABLE
#if SPARE_DC_PWM_EN ==  1
  PWM_Initialize(spareDCMotor);
#elif DRUM_DC_PWM_EN == 1
  PWM_Initialize(drumDCMotor);
#endif

#if SPEED_SENSOR_ENABLE == 1
  miscellaneousSetting.speedSensorEnable = 1;
  if(miscellaneousSetting.speedSensorEnable == 1)
  {
	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  }
#endif
#endif

  HAL_TIM_Base_Start_IT(&htim6);		//5ms timer

#if BS84C12A_DRIVE_ENABLE
#if GET_DISPLAY_WITH_ADC
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_DATA, ADC1_CONVERTED_DATA_BUFFER_SIZE) != HAL_OK)
	{
		Error_Handler();
	}
#endif
  dacSetVoltage(INDUCTION_OFF_COUNT, 0);
  osDelay(20);
#endif
//	HAL_Delay(2000);
  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Android_task_t */
  Android_task_tHandle = osThreadNew(android_task, NULL, &Android_task_t_attributes);

  /* creation of inductionContro */
  inductionControHandle = osThreadNew(InductionControlTask, NULL, &inductionContro_attributes);

  /* creation of motorControl */
  motorControlHandle = osThreadNew(MotorControlTask, NULL, &motorControl_attributes);

  /* creation of sensorRead */
  sensorReadHandle = osThreadNew(SensorReadTask, NULL, &sensorRead_attributes);

  /* creation of errorHandle_t */
  errorHandle_tHandle = osThreadNew(errorHandleTask, NULL, &errorHandle_t_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 42-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4200-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, INDUCTION_K1_Pin|DRUM_DC_INA_Pin|DRUM_DC_INB_Pin|SPARE_DC_EN_Pin
                          |SPI_CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG_LED_Pin|USB_PowerSwitchOn_Pin|BUZZER_Pin|OIL_STEP_DIR_Pin
                          |OIL_STEP_PULSE_Pin|WATER_STEP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SOLENOID_1_Pin|SOLENOID_2_Pin|WATER_STEP_DIR_Pin|WATER_STEP_PULSE_Pin
                          |SPARE_STEP_EN_Pin|SPARE_STEP_DIR_Pin|SPARE_STEP_PULSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OIL_STEP_EN_GPIO_Port, OIL_STEP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INDUCTION_WATTAGE_Pin INDUCTION_INCREMENT_Pin INDUCTION_DECREMENT_Pin INDUCTION_SPARE_Pin
                           INDUCTION_ON_OFF_Pin INDUCTION_TEMPERATURE_Pin */
  GPIO_InitStruct.Pin = INDUCTION_WATTAGE_Pin|INDUCTION_INCREMENT_Pin|INDUCTION_DECREMENT_Pin|INDUCTION_SPARE_Pin
                          |INDUCTION_ON_OFF_Pin|INDUCTION_TEMPERATURE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INDUCTION_K1_Pin DRUM_DC_INA_Pin DRUM_DC_INB_Pin SPARE_DC_EN_Pin
                           SPI_CS2_Pin */
  GPIO_InitStruct.Pin = INDUCTION_K1_Pin|DRUM_DC_INA_Pin|DRUM_DC_INB_Pin|SPARE_DC_EN_Pin
                          |SPI_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_LED_Pin USB_PowerSwitchOn_Pin BUZZER_Pin OIL_STEP_DIR_Pin
                           OIL_STEP_PULSE_Pin WATER_STEP_EN_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin|USB_PowerSwitchOn_Pin|BUZZER_Pin|OIL_STEP_DIR_Pin
                          |OIL_STEP_PULSE_Pin|WATER_STEP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A_SEG_Pin B_SEG_Pin C_SEG_Pin D_SEG_Pin
                           E_SEG_Pin F_SEG_Pin */
  GPIO_InitStruct.Pin = A_SEG_Pin|B_SEG_Pin|C_SEG_Pin|D_SEG_Pin
                          |E_SEG_Pin|F_SEG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : G_SEG_Pin */
  GPIO_InitStruct.Pin = G_SEG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(G_SEG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SOLENOID_1_Pin SOLENOID_2_Pin WATER_STEP_DIR_Pin WATER_STEP_PULSE_Pin
                           SPARE_STEP_EN_Pin SPARE_STEP_DIR_Pin SPARE_STEP_PULSE_Pin */
  GPIO_InitStruct.Pin = SOLENOID_1_Pin|SOLENOID_2_Pin|WATER_STEP_DIR_Pin|WATER_STEP_PULSE_Pin
                          |SPARE_STEP_EN_Pin|SPARE_STEP_DIR_Pin|SPARE_STEP_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OIL_FLOW_SENSOR_Pin WATER_FLOW_SENSOR_Pin GREEN_LED_STATUS_Pin RED_LED_STATUS_Pin
                           SPARE_INPUT_Pin USB_OTG_Overcurrent_Pin */
  GPIO_InitStruct.Pin = OIL_FLOW_SENSOR_Pin|WATER_FLOW_SENSOR_Pin|GREEN_LED_STATUS_Pin|RED_LED_STATUS_Pin
                          |SPARE_INPUT_Pin|USB_OTG_Overcurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OIL_STEP_EN_Pin */
  GPIO_InitStruct.Pin = OIL_STEP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OIL_STEP_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_DE_Pin */
  GPIO_InitStruct.Pin = UART_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART_DE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
#if TM1668_DRIVE_ENABLE == 1
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin != GPIO_PIN_5)
	{
		 if (inductionPins.onOffPin)
		 {
			if(HAL_GPIO_ReadPin(INDUCTION_ON_OFF_GPIO_Port,INDUCTION_ON_OFF_Pin)==GPIO_PIN_SET)
			{
			   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin, SET);
			}
			else
			{   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin, RESET);

			}
		 }
		 if (inductionPins.wattagePin)
		 {
			if(HAL_GPIO_ReadPin(INDUCTION_WATTAGE_GPIO_Port,INDUCTION_WATTAGE_Pin)==GPIO_PIN_SET)
			{
			   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin, SET);
			}
			else
			{   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin, RESET);

			}
		 }
		 if (inductionPins.temperaturePin)
		 {
			if(HAL_GPIO_ReadPin(INDUCTION_TEMPERATURE_GPIO_Port,INDUCTION_TEMPERATURE_Pin)==GPIO_PIN_SET)
			{
			   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin,SET);
			}
			else
			{   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin,RESET);

			}
		 }
		 if (inductionPins.incrementPin)
		 {
			if(HAL_GPIO_ReadPin(INDUCTION_INCREMENT_GPIO_Port,INDUCTION_INCREMENT_Pin)==GPIO_PIN_SET)
			{
			   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin,SET);
			}
			else
			{   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin,RESET);

			}
		 }
		 if (inductionPins.decrementPin)
		 {
			if(HAL_GPIO_ReadPin(INDUCTION_DECREMENT_GPIO_Port,INDUCTION_DECREMENT_Pin)==GPIO_PIN_SET)
			{
			   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin,SET);
			}
			else
			{   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin,RESET);

			}
		 }
	}
	else
	{
		HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin,RESET);
	}
}
#endif

#if SPEED_SENSOR_ENABLE == 1
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM3)			//1us resolution
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if interrupt source is channel 1
		{
//			processError.processStopError = 0;
			processError.speedSensorError = 0;
			timerCnt.speedSensorErrorCnt = 0;
			if(gu8_State == IC_IDLE)
			{
				gu32_T1 = TIM3->CCR2;
				gu16_TIM2_OVC = 0;
				gu8_State = IC_DONE;
			}
			else if(gu8_State == IC_DONE)
			{
				gu32_T2 = TIM3->CCR2;
				gu32_Ticks = (gu32_T2 + (gu16_TIM2_OVC * (0xFFFF + 1))) - gu32_T1;
				gu8_State = IC_IDLE;
			}
		}
		else
		{
			timerCnt.speedSensorErrorCnt = 0;
			current_speed = 0;
			avg_speed = 0;
		}
	}
}
#endif
#if BS84C12A_DRIVE_ENABLE == 1
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	/* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
	if(hadc->Instance==ADC1)
	{
//		SCB_InvalidateDCache_by_Addr((uint32_t *) &ADC1_DATA[0], ADC1_CONVERTED_DATA_BUFFER_SIZE);
//		var = 1;
	}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	/* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */


	if(hadc->Instance==ADC1)
	{
//		memcpy(adcDataDisplaySegments, ADC1_DATA, (ADC1_CONVERTED_DATA_BUFFER_SIZE * 2));
//		var = 2;
//		SCB_InvalidateDCache_by_Addr((uint32_t *) &ADC1_DATA[ADC1_CONVERTED_DATA_BUFFER_SIZE/2], ADC1_CONVERTED_DATA_BUFFER_SIZE);
	}
}
#endif
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
#if TM1668_DRIVE_ENABLE == 1
		inductionKeyPress(keyPressState);
		pressCnt = 0;
		memset(spiReadBuff, 0, sizeof(spiReadBuff));
		HAL_SPI_Receive(&hspi2, (uint8_t *)&spiReadBuff, MAX_ARRAY_SIZE, 1000);
		memcpy(uartTxBuff,spiReadBuff,MAX_ARRAY_SIZE);
		segregatePacket();
#if 	WOKIE_GREEN_BRD_MC_EN == 1
		if(copyStartIndex > VALID_PACKET_SIZE)
		{
			copyStartIndex = 0;
			startIndex = 0;
			checkForDisplayValue();
		}
		else if(copyStartIndex < VALID_PACKET_SIZE)
		{
			copyStartIndex = 0;
			startIndex = 0;
			checkForDisplayValueShortPacket();
		}
#elif 	WOKIE_GREEN_V_1_5_EN == 1
	    if(copyStartIndex == 18)
		{
			copyStartIndex = 0;
			startIndex = 0;
			checkForDisplayValue_new_green();
		}
#endif
		if(uartPrintArr[1] == 'E')
		{
			if(timerCnt.inductionBoardErrorCnt++ >= 200)
			{
				processError.inductionErrorNumber = getErrorNumber();
				Send_Error_Msg(processError.inductionErrorNumber);
				if(processError.inductionErrorNumber != -1)
				{
					processError.inductionErrorNumber = processError.inductionErrorNumber;
				}
				else
				{
					processError.inductionErrorNumber = 'x';
				}
				dutyCycle = 0;
				drumMotor = dcMotorInit;
				inductionMode.temperatureMode = 0;
				inductionMode.wattageMode = 0;
				wattageMode.valueToSet = 0;
				temperatureMode.valueToSet = 0;
				timerCnt.inductionBoardErrorCnt = 0;
				keyPressState = pressOnOff;
				processError.inductionBoardError = 1;
			}
		}
#if			UART_DEBUG_EN == 1
	    HAL_UART_Transmit(&huart1, (uint8_t *)&uartPrintArr, sizeof(uartPrintArr), 100);
#endif
	    osDelay(20);
	#elif BS84C12A_DRIVE_ENABLE == 1
			displayData = getDisplayChar_BS84C12A(displayValueHex);
			if(bs84c12aInductionError == 1)
			{
				if(++timerCnt.inductionBoardErrorCnt >= 20)
				{
					if(displayData != 'E' && displayData != 'X')
					{
						timerCnt.inductionBoardErrorCnt = 0;
						bs84c12aInductionError = 0;
						processError.errorNumberAndroid = displayData;
						processError.inductionBoardError = 1;
					}
				}
			}
		    osDelay(100);
	#endif
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_android_task */
/**
 * @brief Function implementing the Android_task_t thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_android_task */
void android_task(void *argument)
{
  /* USER CODE BEGIN android_task */
	/* Infinite loop */
	for(;;)
	{
#if ANDROID_INTERFACE_ENABLE == 1
		if(Data_Reciption)
		{
			Data_Reciption=0;
			if (received_data[0] == USB_SOF && received_data[USB_PACKET_LENGTH-1] == USB_EOF)
			{
				data_id = received_data[1];
				cmd_id = received_data[2];
				switch(data_id)
				{
					case HANDSHAKE:
						 Send_Response(HANDSHAKE,1);
						 break;
					case START_OF_COOKING:
#if 					TM1668_DRIVE_ENABLE == 1
						 initialOnOffCnt = 0;
#endif
						 androidProcessStruct.startofCooking = 1;
//						 androidProcessStruct.endOfCooking = 0;
						 androidProcessStruct.sendTempAckOnce = 0;
						 clearErrors();
						 clearErrorCounts();
						 if(received_data[2] != 0)
						 {
							 dutyCycle = getDCMotorlevels(miscellaneousSetting.defaultMotorDutyCycleAuto);
							 drumMotor = dcMotorInit;
							 osDelay(20);
							 androidProcessStruct.temperatureAutoMode = 1;
							 androidProcessStruct.manualMode = 0;
							 if(received_data[3] != 0)
							 {
								 dataAndroid = (( received_data[3] << 8) | (received_data[2]));
							 }
							 else
							 {
								 dataAndroid = cmd_id;
							 }
							 Process_WOKIE_Control(HEATER_ON_TEMP, dataAndroid);
							 Send_Status_Wait_ACK();
						 }
						 else
						 {
							 dutyCycle = getDCMotorlevels(miscellaneousSetting.defaultMotorDutyCycleManual);
							 drumMotor = dcMotorInit;
							 osDelay(20);
							 Send_Status_data();
							 androidProcessStruct.temperatureAutoMode = 0;
							 androidProcessStruct.manualMode = 1;
						 }
						break;
					case END_OF_COOKING:
				#if BS84C12A_DRIVE_ENABLE
						 wattageLevel_BS84C12A=0;
						 wattageSettingDACCounts_BS84C12A = getWattageLevelCount_BS84C12A(wattageLevel_BS84C12A);
						 dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
				#elif TM1668_DRIVE_ENABLE ==  1

						 if((uartPrintArr[2] != 'F' && uartPrintArr[3] != 'F' ) || (uartPrintArr[1] == '0' && uartPrintArr[2] == 'N' ))
						 {
							 keyPressState = pressOnOff;
						 }
				#endif
						 dutyCycle = 0;
						 drumMotor = dcMotorInit;
						 cleanDutyCycle = 0;
						 motorRotateCleaning = dcMotorInit;

						 wattageMode.valueToSet = 0;

						 temperatureMode.valueToSet = 0;
						 temperatureMode.valueToSet = 0;

						 inductionMode.temperatureMode = 0;
						 inductionMode.wattageMode = 0;

						 clearErrors();
						 clearErrorCounts();

						 clearProcessVariables();
//						 androidProcessStruct.endOfCooking = 1;

//						 timerCnt.endOfCookingCnt = 0;

						 Send_Standby_Status();
						 osDelay(15);
//						 Send_Response(END_OF_COOKING,1);
						 break;
					case INSTRUCTION_SET:
						 if(androidProcessStruct.manualMode == 1)
						 {
							 dataAndroid = ((received_data[4]<<8) | (received_data[3]));
							 memcpy(and_ack, received_data, 64);
							 Process_WOKIE_Control(cmd_id, dataAndroid);
						 }
						 break;
					case ROTATE_ID:
						 motorRotateCleaningTime = (DEFAULT_ROTATE_TIME / TIME_500MS_IN_SEC);				// 1 secs
						 cleanDutyCycle = getDCMotorlevels(miscellaneousSetting.cleanMotorSpeed);
						 motorRotateCleaning = dcMotorInit;
						 tempCmdID = ROTATE_ID;
						 Send_Response(tempCmdID,1);
						 break;
					case MC_SETTINGS:
			#if			ANDROID_MC_SETTINGS == 1
						 if(cmd_id == 1)
						 {
							 memcpy(&machineSettings,&received_data[3],sizeof(machineSettings));
						 }
						 else if(cmd_id == 2)
						 {
							 memcpy(&temperatureSensorOffset,&received_data[3],sizeof(temperatureSensorOffset));
						 }
						 else if(cmd_id == 3)
						 {
							 memcpy(&miscellaneousSetting,&received_data[3],sizeof(miscellaneousSetting));
						 }
						 parameterValueAssignment();
			#endif
						 Send_Response(MC_SETTINGS,cmd_id);
						 break;
					case EOD_CLEANING_ID:
						 motorRotateCleaningTime = miscellaneousSetting.endOfDayCleaningTime;
						 cleanDutyCycle = getDCMotorlevels(miscellaneousSetting.cleanMotorSpeed);
						 motorRotateCleaning =dcMotorInit;
						 tempCmdID = EOD_CLEANING_ID;
						 Send_Response(tempCmdID,1);
						 break;
					case POST_CLEANING_ID:
						 motorRotateCleaningTime = miscellaneousSetting.endOfRecipeCleaningTime;
						 cleanDutyCycle = getDCMotorlevels(miscellaneousSetting.cleanMotorSpeed);
						 motorRotateCleaning =dcMotorInit;
						 tempCmdID = POST_CLEANING_ID;
						 Send_Response(tempCmdID,1);
						 break;
					case MANUAL_CLEANING:
						 motorRotateCleaningTime = miscellaneousSetting.endOfRecipeCleaningTime;
						 cleanDutyCycle = getDCMotorlevels(miscellaneousSetting.cleanMotorSpeed);
						 motorRotateCleaning = dcMotorInit;
						 tempCmdID = MANUAL_CLEANING;
						 Send_Response(tempCmdID,1);
						 break;

				}
			}
		}
		if(	androidProcessStruct.startofCooking )
		{
			Send_Status_Wait_ACK();
		}
		/*if(androidProcessStruct.endOfCooking && timerCnt.endOfCookingCnt++ >= 2)
		{
			Send_Standby_Status();
			osDelay(15);
			timerCnt.endOfCookingCnt = 0;
			androidProcessStruct.endOfCooking = 0;
		}*/
#endif
		osDelay(1000);
	}
  /* USER CODE END android_task */
}

/* USER CODE BEGIN Header_InductionControlTask */
/**
* @brief Function implementing the inductionControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InductionControlTask */
void InductionControlTask(void *argument)
{
  /* USER CODE BEGIN InductionControlTask */
  /* Infinite loop */
  for(;;)
  {
#if TM1668_DRIVE_ENABLE == 1
	#if WATTAGE_MODE_ENABLE == 1
	  if(inductionMode.wattageMode == 1 && processError.processStopError == 0)
	  {
		  inductionMode.temperatureMode = 0;
		  if(uartPrintArr[0] == '-' && uartPrintArr[1] == '0' && uartPrintArr[2] == 'F' && uartPrintArr[3] == 'F')
		  {
			  keyPressState = pressOnOff;
		  }
		  else if(uartPrintArr[0] == '-' && uartPrintArr[1] == '0' && uartPrintArr[2] == 'N' && uartPrintArr[3] == '-')
		  {
			  keyPressState = pressWattage;
		  }
		  if(uartPrintArr[0] == '-')
		  {
			  uartPrintArr[0] = uartPrintArr[1];
			  uartPrintArr[1] = uartPrintArr[2];
			  uartPrintArr[2] = uartPrintArr[3];
			  uartPrintArr[3] = '\0';
		  }
		  wattageMode.valueSetOnInduction = atoi(uartPrintArr);
		  if(wattageMode.valueToSet < wattageMode.valueSetOnInduction && wattageMode.valueToSet >= WATT_500  && wattageMode.valueSetOnInduction <= WATT_5000)
		  {
			  if(wattageMode.valueSetOnInduction > 400)
			  {
				  keyPressState =  pressDecrement;
			  }
		  }
		  else if(wattageMode.valueToSet > wattageMode.valueSetOnInduction && wattageMode.valueSetOnInduction >= WATT_500 && wattageMode.valueSetOnInduction <= WATT_5000)
		  {
			  keyPressState =  pressIncrement;
		  }
		  else if(wattageMode.valueToSet == wattageMode.valueSetOnInduction && wattageMode.valueSetOnInduction >= WATT_500 && wattageMode.valueToSet > 0)
		  {
			  inductionMode.wattageMode = 0;
		  }
	  }
	  else if(androidProcessStruct.startofCooking == 0)
	  {
		  if(((uartPrintArr[2] != 'F' && uartPrintArr[3] != 'F' ) || (uartPrintArr[1] == '0' && uartPrintArr[2] == 'N' )) && (uartPrintArr[1] != '-' && uartPrintArr[2] != '-'))
		  {
			 keyPressState = pressOnOff;
		  }
	  }
	#endif
	#if TEMPERATURE_CONTROL_MODE == 1
	  if(inductionMode.temperatureMode == 1 && processError.processStopError == 0)
	  {
		  inductionMode.wattageMode = 0;
		#if TEMPERATURE_CURVE == 1
		  temperatureControlCurve(temperatureMode.valueToSet, avgTemperature);
		  if(uartPrintArr[0] == '-' && uartPrintArr[1] == '0' && uartPrintArr[2] == 'F' && uartPrintArr[3] == 'F')
		  {
			  keyPressState = pressOnOff;
		  }
		  else if(uartPrintArr[0] == '-' && uartPrintArr[1] == '0' && uartPrintArr[2] == 'N' && uartPrintArr[3] == '-')
		  {
			  keyPressState = pressWattage;
		  }
		  if(uartPrintArr[0] == '-')
		  {
			  uartPrintArr[0] = uartPrintArr[1];
			  uartPrintArr[1] = uartPrintArr[2];
			  uartPrintArr[2] = uartPrintArr[3];
			  uartPrintArr[3] = '\0';
		  }
		  wattageMode.valueSetOnInduction = atoi(uartPrintArr);
		  if(wattageMode.valueToSet < wattageMode.valueSetOnInduction && wattageMode.valueToSet >= WATT_500  && wattageMode.valueSetOnInduction <= WATT_5000)
		  {
			  if(wattageMode.valueSetOnInduction > 400)
			  {
				  keyPressState =  pressDecrement;
			  }
		  }
		  else if(wattageMode.valueToSet > wattageMode.valueSetOnInduction && wattageMode.valueSetOnInduction >= WATT_500 && wattageMode.valueSetOnInduction <= WATT_5000)
		  {
			  keyPressState =  pressIncrement;
		  }
		  else if(wattageMode.valueToSet == wattageMode.valueSetOnInduction && wattageMode.valueSetOnInduction >= WATT_500 && wattageMode.valueToSet > 0)
		  {

		  }
		#endif
	  }
	  else if(androidProcessStruct.startofCooking == 0)
	  {
		  if(((uartPrintArr[2] != 'F' && uartPrintArr[3] != 'F' ) || (uartPrintArr[1] == '0' && uartPrintArr[2] == 'N' )) && (uartPrintArr[1] != '-' && uartPrintArr[2] != '-'))
		  {
			 keyPressState = pressOnOff;
		  }
	  }
	#endif
	osDelay(250);
#elif BS84C12A_DRIVE_ENABLE == 1
	#if 	WATTAGE_MODE_ENABLE == 1
	  if(inductionMode.wattageMode == 1 && processError.processStopError == 0)
	  {
		  dacSetVoltage(wattageSettingDACCounts_BS84C12A, 0);
		  osDelay(20);
	  }
	#endif
	#if 	TEMPERATURE_CONTROL_MODE == 1
	  if(inductionMode.temperatureMode == 1 && processError.processStopError == 0)
	  {
		  temperatureControlCurve(temperatureMode.valueToSet, avgTemperature);
	  }
	#endif
	osDelay(2000);
#else
	osDelay(500);
#endif
  }
  /* USER CODE END InductionControlTask */
}

/* USER CODE BEGIN Header_MotorControlTask */
/**
* @brief Function implementing the motorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControlTask */
void MotorControlTask(void *argument)
{
  /* USER CODE BEGIN MotorControlTask */
  /* Infinite loop */
  for(;;)
  {
#if DC_MOTOR_TASK_ENABLE == 1
	switch(drumMotor)
	{
		case dcMotorIdle:
			break;
		case dcMotorInit:
#if SPARE_DC_PWM_EN == 1
			 dcMotorSetPWM(dutyCycle, spareDCMotor, CLOCKWISE);
#elif DRUM_DC_PWM_EN == 1
			 dcMotorSetPWM(dutyCycle, drumDCMotor, ANTICLOCKWISE);
#endif
			 drumMotor = dcMotorCompleted;
			 break;
		case dcMotorCompleted:
			 break;
	}
	switch(motorRotateCleaning)
	{
		case dcMotorIdle:
			break;
		case dcMotorInit:
#if SPARE_DC_PWM_EN == 1
			 dcMotorSetPWM(cleanDutyCycle, spareDCMotor, CLOCKWISE);
#elif DRUM_DC_PWM_EN == 1
			 dcMotorSetPWM(cleanDutyCycle, drumDCMotor, ANTICLOCKWISE);
#endif
			 motorRotateCleaningCnt = 0;
			 if(cleanDutyCycle <= 0)
			 {
				 motorRotateCleaning = dcMotorCompleted;
				 dcMotorSetPWM(0, drumDCMotor, ANTICLOCKWISE);
			 }
			 else
			 {
				 motorRotateCleaning = dcMotorWaitForTimeComlete;
			 }
			 break;
		case dcMotorWaitForTimeComlete:
			 if(motorRotateCleaningCnt++ >= motorRotateCleaningTime)
			 {
				 cleanDutyCycle = 0;
				 dcMotorSetPWM(cleanDutyCycle, drumDCMotor, ANTICLOCKWISE);
				 Send_Response(tempCmdID,2);
				 motorRotateCleaning = dcMotorCompleted;
			 }
			 break;
		case dcMotorCompleted:
			 motorRotateCleaning = dcMotorIdle;
			 break;
	}
	if(dutyCycle <= 0)
	{
	#if	BS84C12A_DRIVE_ENABLE == 1
		if(displayData != 'X')
		{
			 dacSetVoltage(INDUCTION_OFF_COUNT, 0);
		}
	#elif TM1668_DRIVE_ENABLE == 1
		if(((uartPrintArr[2] != 'F' && uartPrintArr[3] != 'F' ) || (uartPrintArr[1] == '0' && uartPrintArr[2] == 'N' )) && (uartPrintArr[1] != '-' && uartPrintArr[2] != '-'))
		{
			 keyPressState = pressOnOff;
		}
	#endif
	}
	osDelay(500);
#elif DC_MOTOR_TASK_ENABLE == 0
	  osDelay(5000);
#endif
  }
  /* USER CODE END MotorControlTask */
}

/* USER CODE BEGIN Header_SensorReadTask */
/**
* @brief Function implementing the sensorRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorReadTask */
void SensorReadTask(void *argument)
{
  /* USER CODE BEGIN SensorReadTask */
  /* Infinite loop */
  for(;;)
  {
#if SPEED_SENSOR_ENABLE == 1
	  if(miscellaneousSetting.speedSensorEnable ==  1)
	  {
		  current_speed = getSpeed();
		  if(current_speed < 2)
		  {
			  current_speed = previous_speed;
		  }
		  speed_avg[speed_cnt] = (current_speed / GEAR_TOOTH_NUMBER);
		  sum_speed += speed_avg[speed_cnt];
		  if(speed_cnt++ >= SENSOR_AVG_CNT)
		  {
			  avg_speed = (float)sum_speed/(float)SENSOR_AVG_CNT;
			  sum_speed = 0;
			  speed_cnt = 0;
			  if(avg_speed > 0)
			  {
				  if(avg_speed >= 80)
				  {
					  avg_speed = 80;
				  }
				  else
				  {
					  avg_speed = avg_speed *  (float)speedCorrectionFactor;
				  }
		#if			UART_DEBUG_EN == 1
				  snprintf(speedSensorBuff, sizeof(speedSensorBuff),"Speed : %d \r\n", (int)avg_speed);
				  HAL_UART_Transmit(&huart1, (uint8_t *)&speedSensorBuff, strlen(speedSensorBuff), 100);
				  memset(speedSensorBuff, 0, sizeof(speedSensorBuff));
		#endif
			  }
			  else
			  {
#if			UART_DEBUG_EN == 1
				  HAL_UART_Transmit(&huart1, (uint8_t *)"Speed : Error Reading \r\n", sizeof("Speed : Error Reading \r\n"), 100);
#endif
			  }
		  }
		  previous_speed = current_speed;
		  speedSensorErrorCheck();
	  }
#endif
#if IR_TEMPERATURE_SENSOR_ENABLE == 1
	#if PROCESS_ERROR_ENABLE == 1
	  objTemperature = MLX90614_ReadTemp(MLX90614_DEFAULT_SA, MLX90614_TOBJ1);
	  if(objTemperature == 0 || objTemperature > 400)
	  {
		  if(androidProcessStruct.startofCooking == 1)
		  {
			  if(timerCnt.temperatureSensorErrorCnt++ >= SENSOR_ERROR_CHECK_CNT)
			  {
				  processError.temperatureSensorError = 1;
				  processError.errorNumberAndroid = temperatureSensorError;
				  timerCnt.temperatureSensorErrorCnt = 0;
			  }
		  }
	  }
	  else if(objTemperature > 0)
	  {
		  temperatureArray[avgCnt] = (int)objTemperature;
		  sumTemperature += temperatureArray[avgCnt];
		  if(avgCnt++ >= SENSOR_AVG_CNT)
		  {
			  avgTemperature = (float)sumTemperature/(float)SENSOR_AVG_CNT;
			  if(temperatureSensorOffset.positiveTemperatureOffsetValue > 0 && avgTemperature > 80)
			  {
				  avgTemperature = avgTemperature + temperatureSensorOffset.positiveTemperatureOffsetValue;
			  }
			  else if(temperatureSensorOffset.negativeTemperatureOffsetValue > 0 && avgTemperature > 80)
			  {
				  avgTemperature = avgTemperature - temperatureSensorOffset.negativeTemperatureOffsetValue;
			  }
			  else
			  {
				  avgTemperature = avgTemperature;
			  }
			  sumTemperature = 0;
			  avgCnt = 0;
		  }
		  if(avgTemperature > 0)
		  {
#if			UART_DEBUG_EN == 1
			  snprintf(temperatureBuff, sizeof(temperatureBuff),"Temperature : %d \r\n", (int)avgTemperature);
			  HAL_UART_Transmit(&huart1, (uint8_t *)&temperatureBuff, strlen(temperatureBuff), 100);
			  memset(temperatureBuff, 0, sizeof(temperatureBuff));
#endif
		  }
		  else
		  {
#if			UART_DEBUG_EN == 1
			  HAL_UART_Transmit(&huart1, (uint8_t *)"Temperature : Error Reading \r\n", sizeof("Temperature : Error Reading \r\n"), 100);
#endif
		  }
		  processError.temperatureSensorError = 0;
		  timerCnt.temperatureSensorErrorCnt = 0;
	  }
	#elif PROCESS_ERROR_ENABLE == 0
	  objTemperature = MLX90614_ReadTemp(MLX90614_DEFAULT_SA, MLX90614_TOBJ1);
	  temperatureArray[avgCnt] = (int)objTemperature;
	  sumTemperature += temperatureArray[avgCnt];
	  if(avgCnt++ >= SENSOR_AVG_CNT)
	  {
		  avgTemperature = (((float)sumTemperature/(float)SENSOR_AVG_CNT));
		  if(avgTemperature > 80)
		  {
			  avgTemperature = avgTemperature+20;
		  }
		  sumTemperature = 0;
		  avgCnt = 0;
		  if(avgTemperature > 0)
		  {
			  previousTemperature = avgTemperature;
#if			UART_DEBUG_EN == 1
			  snprintf(temperatureBuff, sizeof(temperatureBuff),"Temperature : %d \r\n", (int)avgTemperature);
			  HAL_UART_Transmit(&huart1, (uint8_t *)&temperatureBuff, strlen(temperatureBuff), 100);
			  memset(temperatureBuff, 0, sizeof(temperatureBuff));
#endif
		  }
		  else
		  {
#if			UART_DEBUG_EN == 1
			  HAL_UART_Transmit(&huart1, (uint8_t *)"Temperature : Error Reading \r\n", sizeof("Temperature : Error Reading \r\n"), 100);
#endif
		  }
	  }
	#endif
#endif
	  osDelay(50);
  }
  /* USER CODE END SensorReadTask */
}

/* USER CODE BEGIN Header_errorHandleTask */
/**
* @brief Function implementing the errorHandle_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_errorHandleTask */
void errorHandleTask(void *argument)
{
  /* USER CODE BEGIN errorHandleTask */
  /* Infinite loop */
  for(;;)
  {
	#if PROCESS_ERROR_ENABLE == 1
	  stopHeaterBasedOnError();
//	  Send_Error_Msg(processError.errorNumberAndroid);
	#endif
    osDelay(5000);
  }
  /* USER CODE END errorHandleTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if(htim->Instance ==  TIM3)
  {
		gu16_TIM2_OVC++;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
