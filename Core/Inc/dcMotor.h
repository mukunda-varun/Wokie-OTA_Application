/*
 * dcMotor.h
 *
 *  Created on: Jun 1, 2021
 *      Author: Varun
 */

#ifndef INC_DCMOTOR_H_
#define INC_DCMOTOR_H_

#include "main.h"

#define VNH_DRIVER_ENABLE 		1
#define BTN7960_DRIVER_EN		0

extern TIM_HandleTypeDef htim1;

void PWM_Initialize(dcMotor_t motor);
void dcMotorSetPWM(float pwmValue, dcMotor_t motor,  motorDirection_t direction);
void PWM_Channels_Stop(dcMotor_t motor);


#endif /* INC_DCMOTOR_H_ */
