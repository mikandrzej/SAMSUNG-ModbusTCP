/*
 * MODBUS_TCP_functions.c
 *
 *  Created on: 06.07.2018
 *      Author: Miki
 */

#include "MODBUS_TCP_srv.h"
#include "Modbus_data.h"

//#include "../../I2C_interr/temp_avg.h"
//#include "../../I2C_interr/i2c_interr.h"

err_t modb_func_0x03 (char *data, uint16_t len, char *rep, uint16_t *rep_len)
{
	err_t err = ERR_OK;
	uint16_t _len = 0;
	uint16_t reg,cnt,val;

	reg = data[0]<<8 | data[1];
	cnt = data[2]<<8 | data[3];

	if (reg>=4000)
		reg -= 4000;
	rep[_len++] = (cnt*2) & 0xFF;

	for (uint16_t i = 0; i < cnt; i++)
	{
		val = ModbusRegReadData(reg+i);
		rep[_len++] = (val>>8) & 0xFF;
		rep[_len++] = val & 0xFF;
	}
/*
	int16_t *ptr;
	for(int k = 0; k < 16; k++){
		avgResetAct(k);
		ptr = &temperature_raw[k];
	  	avgAddValue(k, *ptr);
	}
*/
	*rep_len = _len;
	return err;
}

err_t modb_func_0x06 (char *data, uint16_t len, char *rep, uint16_t *rep_len)
{
	err_t err = ERR_OK;
	uint16_t _len = 0;
	uint16_t reg,val;

	reg = data[0]<<8 | data[1];
	val = data[2]<<8 | data[3];

	ModbusRegInsertData(reg, val);

	rep[_len++] = (reg>>8) & 0xFF;
	rep[_len++] = reg & 0xFF;

	rep[_len++] = (val>>8) & 0xFF;
	rep[_len++] = val & 0xFF;

	*rep_len = _len;
	return err;
}

err_t modb_func_0x10 (char *data, uint16_t len, char *rep, uint16_t *rep_len)
{
	err_t err = ERR_OK;
	uint16_t _len = 0;
	uint16_t reg,cnt,_cnt,val;

	reg = data[0]<<8 | data[1];
	cnt = data[2]<<8 | data[3];
	_cnt = 5;

	for (uint16_t i = 0; i < cnt; i++)
	{
		val = (data[_cnt++]<<8) & 0xFF;
		val |= data[_cnt++] & 0xFF;
		ModbusRegInsertData(reg+i, val);
	}

	rep[_len++] = (reg>>8) & 0xFF;
	rep[_len++] = reg & 0xFF;

	rep[_len++] = (cnt>>8) & 0xFF;
	rep[_len++] = cnt & 0xFF;

	*rep_len = _len;
	return err;
}

