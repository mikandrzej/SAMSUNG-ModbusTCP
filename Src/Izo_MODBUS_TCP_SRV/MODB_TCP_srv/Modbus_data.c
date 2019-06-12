/*
 * Modbus_data.c
 *
 *  Created on: 06.07.2018
 *      Author: Miki
 */

#include "Modbus_data.h"


uint8_t ModbusRegInsertData(uint16_t reg, uint16_t val)
{
	if (reg > MODBUS_REGS)
		return 1;
	Modbus_regs[reg] = val;
	return 0;
}

uint16_t ModbusRegReadData(uint16_t reg)
{
	if (reg > MODBUS_REGS)
		return 0xFFFF;
	return Modbus_regs[reg];
}
