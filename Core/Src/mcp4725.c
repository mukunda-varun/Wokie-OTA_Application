/*
 * mcp4725.c
 *
 *  Created on: Jun 2, 2021
 *      Author: Varun
 */
#include "mcp4725.h"
//------------------------------------------------
extern I2C_HandleTypeDef hi2c2;

#define		MCP4725_ADDRESS		0xC0
#define		MCP4725_EEPROM		0x60
#define 	MCP_4725_WRITE		0x40
#define		MCP_4725_READ		0x01

//uint8_t mcp4725_address = 0xC0;  //0xC4 - other address
//uint8_t mcp4725_read = 0x01;
//uint8_t mcp4725_dac = 0x40;         // Writes data to the DAC
//uint8_t mcp4725_dac_eeprom = 0x60;  // Writes data to the DAC and the EEPROM (persisting the assigned value after reset)

uint8_t buffer[3]={0x00};

//------------------------------------------------
/**
  * @brief  Function to set a voltage for Black induction board using DAC
  * @params	output: Output voltage in terms of counts.
  * 		writeEEPROM : 0 - Normal Write | 1 - EEPROM WRITEs
  * @retval returns the dutycycle to be set
  */
void dacSetVoltage( uint16_t output, char writeEEPROM )
{
	HAL_StatusTypeDef status;
  // Clear write buffer
  uint32_t i;
  for ( i = 0; i < 3; i++ )
  {
    buffer[i] = 0x00;
  }

  if (writeEEPROM == 1)  // command and config bits  (C2.C1.C0.x.x.PD1.PD0.x)
  {
    buffer[0] = MCP4725_EEPROM;
  }
  else
  {
    buffer[0] = MCP_4725_WRITE;
  }
  buffer[1] = (output / 16);       // Upper data bits     (D11.D10.D9.D8.D7.D6.D5.D4)
  buffer[2] = (output % 16) << 4;  // Lower data bits     (D3.D2.D1.D0.x.x.x.x)
  status= HAL_I2C_Master_Transmit(&hi2c2, MCP4725_ADDRESS, buffer, 3, 1000);
}

