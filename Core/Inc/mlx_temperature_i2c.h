/*
 * mlx_temperature_i2c.h
 *
 *  Created on: 26 May 2021
 *      Author: Varun Reddy
 */

#ifndef MLX90614_H_
#define MLX90614_H_


// C Standard Libraries

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "stm32f4xx_hal.h"

/* Private defines -----------------------------------------------------------*/
#define I2C_DEFINITION		&hi2c2
#define	UART_DEFINITION		&huart1

/* DEFAULT SLAVE ADDRESS */
#define MLX90614_DEFAULT_SA 0x5A
/* OPCODE DEFINES */
#define MLX90614_OP_RAM		0x00
#define MLX90614_OP_EEPROM	0x20
#define MLX90614_OP_SLEEP	0xFF

/* RAM offsets with 16-bit data, MSB first */
#define MLX90614_RAW1		(MLX90614_OP_RAM | 0x04) /* raw data IR channel 1 */
#define MLX90614_RAW2		(MLX90614_OP_RAM | 0x05) /* raw data IR channel 2 */
#define MLX90614_TAMB 		(MLX90614_OP_RAM | 0x06) /* ambient temperature */
#define MLX90614_TOBJ1 		(MLX90614_OP_RAM | 0x07) /* object 1 temperature */
#define MLX90614_TOBJ2 		(MLX90614_OP_RAM | 0x08) /* object 2 temperature */
/* EEPROM offsets with 16-bit data, MSB first */
#define MLX90614_TOMIN 		(MLX90614_OP_EEPROM | 0x00) /* object temperature min register */
#define MLX90614_TOMAX 		(MLX90614_OP_EEPROM | 0x01) /* object temperature max register */
#define MLX90614_PWMCTRL 	(MLX90614_OP_EEPROM | 0x02) /* pwm configuration register */
#define MLX90614_TARANGE 	(MLX90614_OP_EEPROM | 0x03) /* ambient temperature register */
#define MLX90614_EMISSIVITY (MLX90614_OP_EEPROM | 0x04) /* emissivity correction register */
#define MLX90614_CFG1 		(MLX90614_OP_EEPROM | 0x05) /* configuration register */
#define MLX90614_SA 		(MLX90614_OP_EEPROM | 0x0E) /* slave address register */
#define MLX90614_ID1 		(MLX90614_OP_EEPROM | 0x1C) /*[read-only] 1 ID register */
#define MLX90614_ID2 		(MLX90614_OP_EEPROM | 0x1D) /*[read-only] 2 ID register */
#define MLX90614_ID3 		(MLX90614_OP_EEPROM | 0x1E) /*[read-only] 3 ID register */
#define MLX90614_ID4 		(MLX90614_OP_EEPROM | 0x1F) /*[read-only] 4 ID register */

#define MLX90614_DBG_OFF 0
#define MLX90614_DBG_ON 1
#define MLX90614_DBG_MSG_W 0
#define MLX90614_DBG_MSG_R 1

/* Exported functions prototypes ---------------------------------------------*/
uint8_t CRC8_Calc (uint8_t *p, uint8_t len);
void MLX90614_WriteReg(uint8_t devAddr, uint8_t regAddr, uint16_t data);
uint16_t MLX90614_ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t dbg_lvl) ;
float MLX90614_ReadTemp(uint8_t devAddr, uint8_t regAddr);
void MLX90614_ScanDevices (void);
void MLX90614_SendDebugMsg(uint8_t op_type, uint8_t devAddr, uint8_t regAddr, uint16_t data, uint8_t crc_in, uint8_t crc_calc) ;



#endif /* MLX90614_H_ */
