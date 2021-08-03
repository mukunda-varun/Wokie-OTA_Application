/*
 * dcMotor.c
 *
 *  Created on: Jun 1, 2021
 *      Author: Varun
 */
#include "dcMotor.h"

	/* @brief: Initialize the PWM Channels
	 * @return: NOTHING*/
void PWM_Initialize(dcMotor_t motor)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		//Drum Motor
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);		//Spare Motor
	TIM1->CCR2 = 0;
	TIM1->CCR4 = 0;
	HAL_GPIO_WritePin(DRUM_DC_INB_GPIO_Port, DRUM_DC_INB_Pin, RESET);
	HAL_GPIO_WritePin(DRUM_DC_INA_GPIO_Port, DRUM_DC_INA_Pin, RESET);
	HAL_GPIO_WritePin(SPARE_DC_EN_GPIO_Port, SPARE_DC_EN_Pin, RESET);
	HAL_GPIO_WritePin(SPARE_DC_ENB_GPIO_Port, SPARE_DC_ENB_Pin, RESET);

/*
	if(motor == drumDCMotor)
	{
#if BTN7960_DRIVER_EN == 1
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);		//Drum Motor
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		//Drum Motor
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, RESET);
#elif VNH_DRIVER_ENABLE == 1
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		//Drum Motor
		TIM1->CCR2 = 0;
//		HAL_GPIO_WritePin(DRUM_DC_INA_GPIO_Port, DRUM_DC_INA_Pin, RESET);
		HAL_GPIO_WritePin(DRUM_DC_INB_GPIO_Port, DRUM_DC_INB_Pin, RESET);
#endif
	}
	else if(motor == spareDCMotor)
	{
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);		//Spare Motor
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);		//Spare Motor
		TIM1->CCR4 = 0;
//		TIM1->CCR3 = 0;
		HAL_GPIO_WritePin(SPARE_DC_EN_GPIO_Port, SPARE_DC_EN_Pin, RESET);
//		HAL_GPIO_WritePin(SPARE_DC_ENB_GPIO_Port, SPARE_DC_ENB_Pin, RESET);
	}*/
}

/* @brief: This function is to run the DC motor
 * @params:  pwm_val->PWM duty cycle at which the motor must run
 * 			 direction->The direction of the motor
 * 			 motor-> Motor to rotate, options are drumDCMotor motor, spareDCMotor motor
 * @return: NOTHING
 * */
void dcMotorSetPWM(float pwmValue, dcMotor_t motor,  motorDirection_t direction)
{
	if(direction == CLOCKWISE)
	{
		if(motor == drumDCMotor)
		{
#if BTN7960_DRIVER_EN == 1
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, SET);
			TIM1->CCR1 = 0;
			TIM1->CCR2 = pwmValue;
#elif VNH_DRIVER_ENABLE == 1
			HAL_GPIO_WritePin(DRUM_DC_INA_GPIO_Port, DRUM_DC_INA_Pin, RESET);
			HAL_GPIO_WritePin(DRUM_DC_INB_GPIO_Port, DRUM_DC_INB_Pin, SET);
			TIM1->CCR2 = pwmValue;
#endif
		}
		else if(motor == spareDCMotor)
		{
			HAL_GPIO_WritePin(SPARE_DC_EN_GPIO_Port, SPARE_DC_EN_Pin, RESET);
//			HAL_GPIO_WritePin(SPARE_DC_ENB_GPIO_Port, SPARE_DC_ENB_Pin, SET);
//			TIM1->CCR3 = 0;
			TIM1->CCR4 = pwmValue;
		}
	}
	else if(direction == ANTICLOCKWISE)
	{
		if(motor == drumDCMotor)
		{
#if BTN7960_DRIVER_EN == 1
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, SET);
			TIM1->CCR1 = pwmValue;
			TIM1->CCR2 = 0;
#elif VNH_DRIVER_ENABLE == 1
			HAL_GPIO_WritePin(DRUM_DC_INA_GPIO_Port, DRUM_DC_INA_Pin, SET);
			HAL_GPIO_WritePin(DRUM_DC_INB_GPIO_Port, DRUM_DC_INB_Pin, RESET);
			TIM1->CCR2 = pwmValue;
#endif
		}
		else if(motor == spareDCMotor)
		{
			HAL_GPIO_WritePin(SPARE_DC_EN_GPIO_Port, SPARE_DC_EN_Pin, SET);
//			HAL_GPIO_WritePin(SPARE_DC_ENB_GPIO_Port, SPARE_DC_ENB_Pin, RESET);
//			TIM1->CCR3 = pwmValue;
			TIM1->CCR4 = pwmValue;
		}
	}
}

void PWM_Channels_Stop(dcMotor_t motor)
{
	if(motor == drumDCMotor)
	{
#if BTN7960_DRIVER_EN == 1
		TIM1->CCR1 = 0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);		//Drum Motor
		TIM1->CCR2 = 0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);		//Drum Motor
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, RESET);
#elif VNH_DRIVER_ENABLE ==  1
		TIM1->CCR2 = 0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);		//Drum Motor
		HAL_GPIO_WritePin(DRUM_DC_INA_GPIO_Port, DRUM_DC_INA_Pin, RESET);
		HAL_GPIO_WritePin(DRUM_DC_INB_GPIO_Port, DRUM_DC_INB_Pin, RESET);
#endif
	}
	else if(motor == spareDCMotor)
	{
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);		//Spare Motor
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);		//Spare Motor
		TIM1->CCR3 = 0;
		TIM1->CCR4 = 0;
		HAL_GPIO_WritePin(SPARE_DC_EN_GPIO_Port, SPARE_DC_EN_Pin, RESET);
	}
}

