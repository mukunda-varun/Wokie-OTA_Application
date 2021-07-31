/*
 * mcp4725.h
 *
 *  Created on: Jun 2, 2021
 *      Author: Varun
 */

#ifndef INC_MCP4725_H_
#define INC_MCP4725_H_

#include "stm32f4xx_hal.h"

void dacSetVoltage( uint16_t output, char writeEEPROM );


#endif /* INC_MCP4725_H_ */
