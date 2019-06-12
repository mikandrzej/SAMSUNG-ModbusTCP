/*
 * Modbus_data.h
 *
 *  Created on: 06.07.2018
 *      Author: Miki
 */

#ifndef IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_DATA_H_
#define IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_DATA_H_

#include "main.h"
#include <stdio.h>

#include "../../common/Modbus_Addresses.h"

#define MODBUS_REGS	1000
uint16_t Modbus_regs[MODBUS_REGS];

uint8_t ModbusRegInsertData(uint16_t reg, uint16_t val);
uint16_t ModbusRegReadData(uint16_t reg);

#endif /* IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_DATA_H_ */
