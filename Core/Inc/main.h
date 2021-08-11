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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <mlx_temperature_i2c.h>
#include "mcp4725.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/***********************ENUMS***********************/
enum{
	IC_IDLE = 0,
	IC_DONE,
};

typedef enum
{
	level1 = 0,
	level2,
	level3,
	level4,
	level5,
	level6,
	level7,
	level8,
	level9,
	level10,
}tempLevel_t;
tempLevel_t temperatureLevelIndicate;

typedef enum state
{
	  idleState=1,
	  dataAvailable,
	  checkForStart,
	  copyDataTillEOP,
	  completeSegregate,
	  waitState,
}processSegregateState;
processSegregateState segregate_state;

typedef enum check
{
	  dataIdle = 1,
	  dataCheckStart,
	  dataCheckWaitForMsg,
	  dataCheckComplete,
}checkState;
checkState checkData;

typedef enum
{
	processFail = 0,
	processSuccess,
}state;
state msgExtractStatus;

typedef enum
{
	pressIdle = 0,
	pressOnOff,
	pressWattage,
	pressTemperature,
	pressDecrement,
	pressIncrement,
	pressComplete,
}keyState_t;
keyState_t keyPressState;

typedef enum
{
   HANDSHAKE=1,
   START_OF_COOKING,			//2
   END_OF_COOKING,				//3
   INSTRUCTION_SET, 			// 4
   STATUS_ID,					//5
   ROTATE_ID,					//6
   EOD_CLEANING_ID= 8,				//8
   POST_CLEANING_ID, 				//9(Collected)
   ERROR_ID,					//10
   MANUAL_CLEANING = 18,
   MC_SETTINGS = 32,					//7
}msg_id_t;

typedef enum
{
	diskSyncError = 0,
	shortWokSensorError,
	shortIGBTError,
	highVoltageError,
	lowVotlageError,
	highTemperatureOvenError,
	highTemperatureIGBTError,
	noWokError,
	interBrdCommnError,
	speedSensorError = 11,
	temperatureSensorError,
}errorID_t;

typedef enum motorDC
{
	drumDCMotor = 0,
	spareDCMotor,
}dcMotor_t;


typedef enum direction
{
	ANTICLOCKWISE = 0,
	CLOCKWISE,
}motorDirection_t;

typedef enum{
	dcMotorIdle = 0,
	dcMotorInit,
	dcMotorWaitForTimeComlete,
	dcMotorCompleted
}motorState_t;
motorState_t drumMotor, motorRotateCleaning;

/***********************ENUMS***********************/

/***********************STRUCT***********************/
typedef struct
{
	uint8_t onOffPin : 1;
	uint8_t wattagePin : 1;
	uint8_t temperaturePin : 1;
	uint8_t incrementPin : 1;
	uint8_t decrementPin : 1;
}pinCtl;
pinCtl inductionPins;

typedef struct
{
	uint8_t temperatureMode : 1;
	uint8_t wattageMode : 1;
}mode;
mode inductionMode;

typedef struct
{
	uint16_t valueFromAndroid;
	uint16_t valueToSet;
	uint16_t valueSetOnInduction;
}inductionControlModeParams;
inductionControlModeParams temperatureMode, wattageMode;


typedef struct{
	uint8_t segmentACnt;
	uint8_t segmentBCnt;
	uint8_t segmentCCnt;
	uint8_t segmentDCnt;
	uint16_t segmentDtempCnt;
	uint8_t segmentDtempvalue;
	uint8_t segmentECnt;
	uint8_t segmentFCnt;
	uint8_t segmentGCnt;
}count;

typedef struct{
	uint8_t speedSensorErrorCnt;
	uint8_t temperatureSensorErrorCnt;
	uint8_t inductionBoardErrorCnt;
	uint8_t inductionErrorCnt;
	uint16_t stopErrorCnt;
//	uint8_t endOfCookingCnt;
}Cnt;
Cnt timerCnt;

typedef struct{
	uint8_t speedSensorError : 1;
	uint8_t temperatureSensorError : 1;
	uint8_t inductionBoardError : 1;
	uint8_t errorNumberAndroid;
	uint8_t processStopError : 1;
}errorState;
errorState processError;

typedef struct{
uint8_t startofCooking : 1;	//soc_flag
uint8_t temperatureAutoMode : 1;
uint8_t autoMode : 1;
uint8_t sendTempAckOnce : 1;
uint8_t manualMode : 1;//cooking_started
//uint8_t endOfCooking : 1;//eoc_flag
}andStruct_t;
andStruct_t androidProcessStruct;


typedef struct
{
	uint16_t deltaTemperature;
	uint16_t deltaLevel1;
	uint16_t deltaLevel2;
	uint16_t deltaLevel3;
	uint16_t deltaLevel4;
	uint16_t deltaLevel5;
	uint16_t deltaLevel6;
	uint16_t deltaLevel7;
	uint16_t deltaLevel8;
	uint16_t deltaLevel9;
	uint16_t deltaWattageLevel1;
	uint16_t deltaWattageLevel2;
	uint16_t deltaWattageLevel3;
	uint16_t deltaWattageLevel4;
	uint16_t deltaWattageLevel5;
	uint16_t deltaWattageLevel6;
	uint16_t deltaWattageLevel7;
	uint16_t deltaWattageLevel8;
	uint16_t deltaWattageLevel9;
}temperatureCurveStruct_t;
temperatureCurveStruct_t deltaTempStruct;

typedef struct{
	uint16_t temperatureLevel1;
	uint16_t temperatureLevel2;
	uint16_t temperatureLevel3;
	uint16_t temperatureLevel4;
	uint16_t temperatureLevel5;
	uint16_t temperatureLevel6;
	uint16_t temperatureLevel7;
	uint16_t temperatureLevel8;
	uint16_t temperatureLevel9;
	uint16_t wattageLevel1;
	uint16_t wattageLevel2;
	uint16_t wattageLevel3;
	uint16_t wattageLevel4;
	uint16_t wattageLevel5;
	uint16_t wattageLevel6;
	uint16_t wattageLevel7;
	uint16_t wattageLevel8;
	uint16_t wattageLevel9;
}androidParam_t;
androidParam_t machineSettings;
typedef struct
{
	uint16_t positiveTemperatureOffsetValue;
	uint16_t negativeTemperatureOffsetValue;
}temperatureSense_t;
temperatureSense_t temperatureSensorOffset;
typedef struct
{
	uint16_t speedSensorCorrectionFactor;
	uint16_t defaultMotorDutyCycleAuto;
	uint16_t defaultMotorDutyCycleManual;
	uint16_t defaultTemperatureAckAuto;
	uint16_t defaultTemperatureAckManual;
	uint16_t speedSensorEnable;
	uint16_t endOfRecipeCleaningTime;
	uint16_t endOfDayCleaningTime;
	uint16_t cleanMotorSpeed;
}miscSetting_t;
miscSetting_t miscellaneousSetting;
/***********************STRUCT***********************/
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/************COMPILER ENABLING************/
#define 	UART_DEBUG_EN					0

#define		OTA_EN 							0

/************MACHINE TYPE ENABLING************/
#define WOKIE_GREEN_BRD_MC_EN				0

#define WOKIE_GREEN_V_1_5_EN				0

#define	ACT_MACHINE_EN						0

#define WOKIE_BLACK_BRD_EN					1
/************MACHINE TYPE ENABLING************/
#if WOKIE_GREEN_V_1_5_EN

	#define	TM1668_DRIVE_ENABLE				1

	#define	BS84C12A_DRIVE_ENABLE			0

	#define	ANDROID_INTERFACE_ENABLE		1

	#define	IR_TEMPERATURE_SENSOR_ENABLE	0

	#define SPEED_SENSOR_ENABLE				0

	#define	DC_MOTOR_TASK_ENABLE			0

	#define SPARE_DC_PWM_EN					0

	#define	DRUM_DC_PWM_EN					0

	#define	WATTAGE_MODE_ENABLE				1

	#define TEMPERATURE_CONTROL_MODE		0

	#define TEMPERATURE_CURVE  				0

	#define	DELTA_METHOD					0

	#define GET_DISPLAY_WITH_ADC			0

	#define PROCESS_ERROR_ENABLE			1

	#define ANDROID_MC_SETTINGS				0
#elif WOKIE_GREEN_BRD_MC_EN

	#define	TM1668_DRIVE_ENABLE				1

	#define	BS84C12A_DRIVE_ENABLE			0

	#define	ANDROID_INTERFACE_ENABLE		1

	#define	IR_TEMPERATURE_SENSOR_ENABLE	1

	#define SPEED_SENSOR_ENABLE				1

	#define	DC_MOTOR_TASK_ENABLE			1

	#define SPARE_DC_PWM_EN					0

	#define	DRUM_DC_PWM_EN					1

	#define	WATTAGE_MODE_ENABLE				1

	#define TEMPERATURE_CONTROL_MODE		0

	#define TEMPERATURE_CURVE  				1

	#define	DELTA_METHOD					1

	#define GET_DISPLAY_WITH_ADC			0

	#define PROCESS_ERROR_ENABLE			1

	#define ANDROID_MC_SETTINGS				0
#elif ACT_MACHINE_EN == 1

	#define	TM1668_DRIVE_ENABLE				0

	#define	BS84C12A_DRIVE_ENABLE			1

	#define	ANDROID_INTERFACE_ENABLE		1

	#define	IR_TEMPERATURE_SENSOR_ENABLE	0

	#define SPEED_SENSOR_ENABLE				0

	#define	DC_MOTOR_TASK_ENABLE			0

	#define SPARE_DC_PWM_EN					0

	#define	DRUM_DC_PWM_EN					0

	#define	WATTAGE_MODE_ENABLE				1

	#define TEMPERATURE_CONTROL_MODE		0

	#define TEMPERATURE_CURVE  				0

	#define	DELTA_METHOD					0

	#define GET_DISPLAY_WITH_ADC			0

	#define PROCESS_ERROR_ENABLE			0

	#define ANDROID_MC_SETTINGS				0

#elif WOKIE_BLACK_BRD_EN == 1

	#define	TM1668_DRIVE_ENABLE				0

	#define	BS84C12A_DRIVE_ENABLE			1

	#define	ANDROID_INTERFACE_ENABLE		1

	#define	IR_TEMPERATURE_SENSOR_ENABLE	1

	#define SPEED_SENSOR_ENABLE				1

	#define	DC_MOTOR_TASK_ENABLE			1

	#define SPARE_DC_PWM_EN					0

	#define	DRUM_DC_PWM_EN					1

	#define	WATTAGE_MODE_ENABLE				1

	#define TEMPERATURE_CONTROL_MODE		1

	#define TEMPERATURE_CURVE  				1

	#define	DELTA_METHOD					1

	#define GET_DISPLAY_WITH_ADC			0

	#define PROCESS_ERROR_ENABLE			1

	#define ANDROID_MC_SETTINGS				1

#endif

#define		LED_BUZZER_OTA					1			//0 for LED(Discovery) & 1 for Buzzer(PCB)
/************COMPILER ENABLING************/

/************PROCESS DEFAULTS*************/
#define 	DEFAULT_DELTA_TEMP1				175
#define 	DEFAULT_DELTA_TEMP2				120
#define 	DEFAULT_DELTA_TEMP3				90
#define 	DEFAULT_DELTA_TEMP4				70
#define 	DEFAULT_DELTA_TEMP5				50
#define 	DEFAULT_DELTA_TEMP6				25
#define 	DEFAULT_DELTA_TEMP7				12
#define 	DEFAULT_DELTA_TEMP8				6
#define 	DEFAULT_DELTA_TEMP9				3

#define 	DEFAULT_DELTA_WATT1				8
#define 	DEFAULT_DELTA_WATT2				8
#define 	DEFAULT_DELTA_WATT3				8
#define 	DEFAULT_DELTA_WATT4 			7
#define 	DEFAULT_DELTA_WATT5				7
#define 	DEFAULT_DELTA_WATT6				7
#define 	DEFAULT_DELTA_WATT7				6
#define 	DEFAULT_DELTA_WATT8				6
#define 	DEFAULT_DELTA_WATT9				4

#define 	DEFAULT_TEMP_ACK_AUTO			60
#define 	DEFAULT_TEMP_ACK_MANUAL			75
#define		DEFAULT_TEMP_POS_OFFSET			20
#define		DEFAULT_TEMP_NEG_OFFSET			0
#define 	DEFAULT_SPEED_CORRECTION		5				//0.5
#define		DEFAULT_SPEED_SENSOR_ENABLE		1
#define		DEFAULT_AUTO_DC_LEVEL			3
#define		DEFAULT_MANUAL_DC_LEVEL			3
#define		DEFAULT_EOD_TIME				45
#define		DEFAULT_EOR_TIME				30
#define		DEFAULT_ROTATE_TIME				1
#define		DEFAULT_MOTOR_CLEANING			3
/************PROCESS DEFAULTS*************/

/************ANDROID MACROS************/

#define USB_PACKET_LENGTH 				64
#define USB_FRAME_LEN 					60
#define USB_SOF 						0x13
#define USB_EOF 						0x12

#define	MAX_TEMPERATURE_CAN_SET			300
#define	TEMP_ACK_PERCENT			((float)50/(float)100)


#define HEATER_ON_WATT 					1
#define HEATER_ON_TEMP 					2
#define ROTATE_DRUM 					3
#define HEATER_OFF 						4
#define ROTATION_OFF 					5
#define TOSS 							6
/************ANDROID MACROS************/

/************TIME MACROS************/
#define	DWT_DELAY_MICRO_US				25000
/************TIME MACROS************/

/************BUFFER SIZE MACROS************/
#define	MAX_ARRAY_SIZE				150
#define	VALID_PACKET_SIZE			47
/************BUFFER SIZE MACROS************/

/************PACKET VALIDITY MACROS************/
#define	DISPLAY_SOF					0xC0
#define	DISPLAY_EOF					0x91
/************PACKET VALIDITY MACROS************/


/************DISPLAY VALUE MACROS************/
#define	ZERO_VALUE					0xEB
#define	ONE_VALUE					0x82
#define	TWO_VALUE					0xB9
#define	THREE_VALUE					0xBA
#define	FOUR_VALUE					0xD2
#define	FIVE_VALUE					0x7A
#define	SIX_VALUE					0x7B
#define SEVEN_VALUE					0xA2
#define	EIGHT_VALUE					0xFB
#define	NINE_VALUE					0xFA
#define	E_VALUE						0x79
#define	N_VALUE						0xE3
#define	F_VALUE						0x71
#if WOKIE_GREEN_V_1_5_EN == 1
	#define DASH_VALUE					0x10
	#define A_VALUE						0xF3
#endif
/************DISPLAY VALUE MACROS************/

/************WATTAGE VALUE MACROS************/
#define	WATT_500					500
#define	WATT_800					800
#define WATT_1000					1000
#define	WATT_1300					1300
#define	WATT_1500					1500
#define	WATT_1900					1900
#define	WATT_2300					2300
#define	WATT_2700					2700
#define WATT_3100					3100
#define	WATT_3500					3500
#define	WATT_4000					4000
#define	WATT_4500					4500
#define	WATT_5000					5000
/************WATTAGE VALUE MACROS************/

/************TEMPERATURE VALUE MACROS************/
#define	TEMP_60					60
#define	TEMP_80					80
#define	TEMP_100				100
#define	TEMP_120				120
#define	TEMP_130				130
#define	TEMP_140				140
#define	TEMP_150				150
#define TEMP_160				160
#define	TEMP_170				170
#define	TEMP_180				180
#define	TEMP_200				200
#define	TEMP_230				230
#define	TEMP_260				260
/************TEMPERATURE VALUE MACROS************/

/************BS84C12A WATTAGE DAC VALUE MACROS************/

#define	VTG_TO_ADC_COUNTS		((float)4096/(float)5)
#define ADC_COUNTS_TO_VTG		((float)5/(float)4096)

#define	ON_OFF_VTG				0.5				//0.4
#define LEVEL1_VTG				0.95			//0.83
#define LEVEL2_VTG				1.35			//1.24
#define LEVEL3_VTG				1.6//1.8				//1.6
#define LEVEL4_VTG				1.95//2.25			//2.02
#define LEVEL5_VTG				2.38//2.6				//2.38
#define LEVEL6_VTG				2.73//3.0				//2.73
#define LEVEL7_VTG				3.0//3.25       		//3.0
#define LEVEL8_VTG				3.53//3.9				//3.53
#define LEVEL9_VTG				3.98//4.4				//3.98


#define	INDUCTION_OFF_COUNT		(ON_OFF_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL1_COUNT			(LEVEL1_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL2_COUNT			(LEVEL2_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL3_COUNT			(LEVEL3_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL4_COUNT			(LEVEL4_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL5_COUNT			(LEVEL5_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL6_COUNT			(LEVEL6_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL7_COUNT			(LEVEL7_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL8_COUNT			(LEVEL8_VTG * VTG_TO_ADC_COUNTS)
#define LEVEL9_COUNT			(LEVEL9_VTG * VTG_TO_ADC_COUNTS)

/************BS84C12A Display VALUE MACROS************/
#define	DISP_SEGMENT_HIGH_VAL	2.5

//D Segment is being ignored since there are not much conflicts with D and cracking D segment isnt completed
#define	ZERO_BS84C12A 			0x3F
#define	ONE_BS84C12A 			0X06
#define	TWO_BS84C12A 			0X53		//0X5B
#define	THREE_BS84C12A 			0X47		//0X4F
#define	FOUR_BS84C12A 			0X66
#define	FIVE_BS84C12A 			0X65		//0X6D
#define	SIX_BS84C12A 			0X75		//0X7D
#define	SEVEN_BS84C12A 			0X07
#define	EIGHT_BS84C12A 			0X77		//0X7F//
#define	NINE_BS84C12A 			0x67		//0X6F
#define	ERROR_BS84C12A 			0X71		//0X79
#define	DASH_BS84C12A 			0X40
/************BS84C12A Display VALUE MACROS************/

/************BS84C12A WATTAGE DAC VALUE MACROS************/

/************VALID PACKET MACROS************/
#define	FIRST_SEG_VALID_INDEX		9
#define	FIRST_SEG_VALUE				0x03
#define	FIRST_SEG_DATA_INDEX		10

#define	SECOND_SEG_VALID_INDEX		15
#define	SECOND_SEG_VALUE			0x43
#define	SECOND_SEG_DATA_INDEX		16

#define	THIRD_SEG_VALID_INDEX		21
#define	THIRD_SEG_VALUE				0x23
#define	THIRD_SEG_DATA_INDEX		22

#define	FOURTH_SEG_VALID_INDEX		27
#define	FOURTH_SEG_VALUE			0x63
#define	FOURTH_SEG_DATA_INDEX		28
/************VALID PACKET MACROS************/

/************DC MOTOR MACROS************/
#define DC_MOTOR_ACCELERATION_CNT  	5
/************DC MOTOR MACROS************/

#define UINT16_MAX_VALUE							  65535

#define TEMPERATURE_CUTOFF_MIN							3
#define TEMPERATURE_CUTOFF_MAX							3
#define	GEAR_TOOTH_NUMBER								20

#define TIME_500MS_IN_SEC								0.5

#define SENSOR_AVG_CNT									10
#define	SENSOR_ERROR_CHECK_CNT							150

#define	PERCENT_CALCULATION(SET,CURRENT)	((float)(SET-CURRENT)/(float)SET)


/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INDUCTION_WATTAGE_Pin GPIO_PIN_2
#define INDUCTION_WATTAGE_GPIO_Port GPIOE
#define INDUCTION_WATTAGE_EXTI_IRQn EXTI2_IRQn
#define INDUCTION_INCREMENT_Pin GPIO_PIN_3
#define INDUCTION_INCREMENT_GPIO_Port GPIOE
#define INDUCTION_INCREMENT_EXTI_IRQn EXTI3_IRQn
#define INDUCTION_DECREMENT_Pin GPIO_PIN_4
#define INDUCTION_DECREMENT_GPIO_Port GPIOE
#define INDUCTION_DECREMENT_EXTI_IRQn EXTI4_IRQn
#define INDUCTION_SPARE_Pin GPIO_PIN_5
#define INDUCTION_SPARE_GPIO_Port GPIOE
#define INDUCTION_SPARE_EXTI_IRQn EXTI9_5_IRQn
#define INDUCTION_K1_Pin GPIO_PIN_6
#define INDUCTION_K1_GPIO_Port GPIOE
#define DEBUG_LED_Pin GPIO_PIN_13
#define DEBUG_LED_GPIO_Port GPIOC
#define USB_PowerSwitchOn_Pin GPIO_PIN_0
#define USB_PowerSwitchOn_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_1
#define BUZZER_GPIO_Port GPIOC
#define A_SEG_Pin GPIO_PIN_1
#define A_SEG_GPIO_Port GPIOA
#define B_SEG_Pin GPIO_PIN_2
#define B_SEG_GPIO_Port GPIOA
#define C_SEG_Pin GPIO_PIN_3
#define C_SEG_GPIO_Port GPIOA
#define D_SEG_Pin GPIO_PIN_4
#define D_SEG_GPIO_Port GPIOA
#define DAC_SLIDER_Pin GPIO_PIN_5
#define DAC_SLIDER_GPIO_Port GPIOA
#define E_SEG_Pin GPIO_PIN_6
#define E_SEG_GPIO_Port GPIOA
#define F_SEG_Pin GPIO_PIN_7
#define F_SEG_GPIO_Port GPIOA
#define G_SEG_Pin GPIO_PIN_4
#define G_SEG_GPIO_Port GPIOC
#define SPARE_ADC1_Pin GPIO_PIN_5
#define SPARE_ADC1_GPIO_Port GPIOC
#define SPARE_ADC2_Pin GPIO_PIN_0
#define SPARE_ADC2_GPIO_Port GPIOB
#define SPARE_ADC3_Pin GPIO_PIN_1
#define SPARE_ADC3_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define DRUM_DC_INA_Pin GPIO_PIN_9
#define DRUM_DC_INA_GPIO_Port GPIOE
#define DRUM_DC_INB_Pin GPIO_PIN_10
#define DRUM_DC_INB_GPIO_Port GPIOE
#define DRUM_DC_PWM_Pin GPIO_PIN_11
#define DRUM_DC_PWM_GPIO_Port GPIOE
#define SPARE_DC_EN_Pin GPIO_PIN_12
#define SPARE_DC_EN_GPIO_Port GPIOE
#define SPARE_DC_ENB_Pin GPIO_PIN_13
#define SPARE_DC_ENB_GPIO_Port GPIOE
#define SPARE_DC_RPWM_Pin GPIO_PIN_14
#define SPARE_DC_RPWM_GPIO_Port GPIOE
#define SPI_CS2_Pin GPIO_PIN_15
#define SPI_CS2_GPIO_Port GPIOE
#define IR_TEMP_SCL_Pin GPIO_PIN_10
#define IR_TEMP_SCL_GPIO_Port GPIOB
#define IR_TEMP_SDA_Pin GPIO_PIN_11
#define IR_TEMP_SDA_GPIO_Port GPIOB
#define SPI_CS1_Pin GPIO_PIN_12
#define SPI_CS1_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_13
#define SPI_SCK_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_15
#define SPI_MOSI_GPIO_Port GPIOB
#define SOLENOID_1_Pin GPIO_PIN_8
#define SOLENOID_1_GPIO_Port GPIOD
#define SOLENOID_2_Pin GPIO_PIN_9
#define SOLENOID_2_GPIO_Port GPIOD
#define OIL_FLOW_SENSOR_Pin GPIO_PIN_10
#define OIL_FLOW_SENSOR_GPIO_Port GPIOD
#define WATER_FLOW_SENSOR_Pin GPIO_PIN_11
#define WATER_FLOW_SENSOR_GPIO_Port GPIOD
#define GREEN_LED_STATUS_Pin GPIO_PIN_12
#define GREEN_LED_STATUS_GPIO_Port GPIOD
#define RED_LED_STATUS_Pin GPIO_PIN_13
#define RED_LED_STATUS_GPIO_Port GPIOD
#define SPARE_INPUT_Pin GPIO_PIN_14
#define SPARE_INPUT_GPIO_Port GPIOD
#define OTA_LED_DEBUG_Pin GPIO_PIN_15
#define OTA_LED_DEBUG_GPIO_Port GPIOD
#define SPEED_MEASURE_Pin GPIO_PIN_6
#define SPEED_MEASURE_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define OIL_STEP_EN_Pin GPIO_PIN_15
#define OIL_STEP_EN_GPIO_Port GPIOA
#define OIL_STEP_DIR_Pin GPIO_PIN_10
#define OIL_STEP_DIR_GPIO_Port GPIOC
#define OIL_STEP_PULSE_Pin GPIO_PIN_11
#define OIL_STEP_PULSE_GPIO_Port GPIOC
#define WATER_STEP_EN_Pin GPIO_PIN_12
#define WATER_STEP_EN_GPIO_Port GPIOC
#define WATER_STEP_DIR_Pin GPIO_PIN_0
#define WATER_STEP_DIR_GPIO_Port GPIOD
#define WATER_STEP_PULSE_Pin GPIO_PIN_1
#define WATER_STEP_PULSE_GPIO_Port GPIOD
#define SPARE_STEP_EN_Pin GPIO_PIN_2
#define SPARE_STEP_EN_GPIO_Port GPIOD
#define SPARE_STEP_DIR_Pin GPIO_PIN_3
#define SPARE_STEP_DIR_GPIO_Port GPIOD
#define SPARE_STEP_PULSE_Pin GPIO_PIN_4
#define SPARE_STEP_PULSE_GPIO_Port GPIOD
#define USB_OTG_Overcurrent_Pin GPIO_PIN_5
#define USB_OTG_Overcurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define UART_DE_Pin GPIO_PIN_5
#define UART_DE_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_6
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_7
#define UART_RX_GPIO_Port GPIOB
#define INDUCTION_ON_OFF_Pin GPIO_PIN_0
#define INDUCTION_ON_OFF_GPIO_Port GPIOE
#define INDUCTION_ON_OFF_EXTI_IRQn EXTI0_IRQn
#define INDUCTION_TEMPERATURE_Pin GPIO_PIN_1
#define INDUCTION_TEMPERATURE_GPIO_Port GPIOE
#define INDUCTION_TEMPERATURE_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
