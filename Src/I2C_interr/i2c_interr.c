/*
 * i2c_interr.c
 *
 *  Created on: 03.10.2018
 *      Author: Miki
 */

#include "i2c_interr.h"

#include "stm32f4xx_hal.h"
#include "i2c_selector.h"

#include "../common/IOports.h"

#include "temp_avg.h"

extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

uint16_t act_sens[2] = {0,0};

uint8_t i2c_snd_dataA[20];
uint8_t i2c_snd_dataB[20];
uint8_t i2c_rec_dataA[20];
uint8_t i2c_rec_dataB[20];

uint16_t err_cnt_i2c = 0;

uint8_t dump[10];

void bit_bang(I2C_HandleTypeDef *hi2c)
{
	err_cnt_i2c++;

	  GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	  if(hi2c == &hi2c2)
	  {
		  GPIO_InitStruct.Pin = GPIO_PIN_1;
		  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		  for(int k=0; k<20; k++)
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, k%2);
	  }

	  if(hi2c == &hi2c3)
	  {
		  GPIO_InitStruct.Pin = GPIO_PIN_8;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		  for(int k=0; k<20; k++)
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, k%2);
	  }

	  HAL_I2C_MspInit(hi2c);
}


void RST_i2c(I2C_HandleTypeDef *hi2c)
{
	bit_bang(hi2c);
    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
	if(hi2c == &hi2c2)
	{
		RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
		__DSB();
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
		__DSB();
		hi2c->Instance->CR1 &= ~I2C_CR1_PE;
		hi2c->Instance->CR1 |= I2C_CR1_SWRST;
		__DSB();
		hi2c->Instance->CR1 &= ~I2C_CR1_SWRST;
		__DSB();

		HAL_I2C_Init(hi2c);

		i2c_err_cnt2++;
	}
	else if(hi2c == &hi2c3)
	{
		RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
		__DSB();
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
		__DSB();
		hi2c->Instance->CR1 &= ~I2C_CR1_PE;
		hi2c->Instance->CR1 |= I2C_CR1_SWRST;
		__DSB();
		hi2c->Instance->CR1 &= ~I2C_CR1_SWRST;
		__DSB();
		HAL_I2C_Init(hi2c);

		i2c_err_cnt1++;
	}
}

void nextSens(uint8_t port)
{
	if(++act_sens[port] >= 8)
		act_sens[port]=0;
	selectInputI2C(port, (act_sens[port]/4));
}

uint8_t getAddr(uint16_t a_sens)
{
	a_sens %=4;
	switch(a_sens)
	{
	case 0:
		return ADT_ADDRESS1;
	case 1:
		return ADT_ADDRESS2;
	case 2:
		return ADT_ADDRESS3;
	case 3:
		return ADT_ADDRESS4;
	}
	return 0;
}

uint8_t getEEPROMAddr(uint16_t a_sens)
{
	a_sens %=4;
	switch(a_sens)
	{
	case 0:
		return EEPROM_ADDRESS1;
	case 1:
		return EEPROM_ADDRESS2;
	case 2:
		return EEPROM_ADDRESS3;
	case 3:
		return EEPROM_ADDRESS4;
	}
	return 0;
}

double conv_temperature(uint8_t *data)
{
	int16_t temp;
	double temp_d;

	temp = data[0];
	temp <<= 8;
	temp |= data[1];
	temp_d = (double) temp;
	temp_d /= 128;

	return temp_d;
}

void i2c_interr_init(void)
{
	selectInputI2C(0,0);
	selectInputI2C(1,0);

	for(int k=0;k<2;k++)
		for(int j=0;j<8;j++)
			sens[k][j].state = I2C_STATE_IDLE;
	for(int k=0;k<16;k++)
		sid[k] = 0xFFFF;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t port;
	s_i2c_sens *sns;
	if(hi2c == &hi2c3)
		port = 0;
	else if(hi2c == &hi2c2)
		port = 1;
	else
		return;
	sns = &sens[port][act_sens[port]];
	switch(sns->state){
	case I2C_STATE_CONF_SEND:
		sns->state = I2C_STATE_CONF_SEND_DONE;
		break;
	default:
		sns->state = I2C_STATE_ERROR;
	}
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t port;
	uint8_t *rec_data;
	s_i2c_sens *sns;
	if(hi2c == &hi2c3)
	{
		port = 0;
		rec_data = i2c_rec_dataA;
	}
	else if(hi2c == &hi2c2)
	{
		port = 1;
		rec_data = i2c_rec_dataB;
	}
	else
		return;

	sns = &sens[port][act_sens[port]];

	uint8_t temper_no = (port*8)+act_sens[port];
	switch(sns->state){
	case I2C_STATE_CONF_CHECK_QUERY:
		if(rec_data[0] == 0x80)
			sns->state = I2C_STATE_CONF_CHECK_RECEIVED_OK;
		else
			sns->state = I2C_STATE_CONF_CHECK_RECEIVED_FAIL;
		break;
	case I2C_STATE_READ_TEMP_QUERY:
		temperature[temper_no] = conv_temperature(rec_data);
		temperature_raw[temper_no] = ((rec_data[0])<<8) | rec_data[1];
		sns->state = I2C_STATE_READ_TEMP_RECEIVE;
		break;
	case I2C_STATE_READ_ID_QUERY:
		sns->state = I2C_STATE_READ_ID_REC;
		HAL_GPIO_TogglePin(LAMP_I2C_GPIO_Port, LAMP_I2C_Pin);
		break;
	default:
		sns->state = I2C_STATE_IDLE;
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t port;
	s_i2c_sens *sns;
	if(hi2c == &hi2c3)
		port = 0;
	else if(hi2c == &hi2c2)
		port = 1;
	else
		return;
	sns = &sens[port][act_sens[port]];
	sns->state = I2C_STATE_ERROR;
	RST_i2c(hi2c);
}

void task_Temperature(void)
{
	static uint16_t tout1,tout2;
	HAL_StatusTypeDef status = HAL_OK;
	s_i2c_sens *sns;
	uint8_t addr;
	sns = &sens[0][act_sens[0]];
	addr = getAddr(act_sens[0]);
	if (sns->state == I2C_STATE_IDLE && meas_perm1>0)
	{
		meas_perm1 = 0;
		tout1 = 0;
		status = HAL_I2C_IsDeviceReady(&hi2c3, addr | ADT_READ, 1, 1);
		if(status == HAL_OK)
		{
			status = HAL_I2C_Mem_Read_IT(&hi2c3, addr | ADT_READ, ADT_REG_CONFIG, 1, i2c_rec_dataA, 1);
			if(status == HAL_OK)
					sns->state = I2C_STATE_CONF_CHECK_QUERY;
		}
		else
			sns->state = I2C_STATE_ERROR;
	}
	if (sns->state == I2C_STATE_CONF_CHECK_RECEIVED_FAIL)
	{
		tout1 = 0;
		status = HAL_I2C_IsDeviceReady(&hi2c3, addr | ADT_READ, 1, 1);
		if(status == HAL_OK)
		{
			i2c_snd_dataA[0] = 0x80;
			status = HAL_I2C_Mem_Write_IT(&hi2c3, addr, ADT_REG_CONFIG, 1, i2c_snd_dataA, 1);
			if(status == HAL_OK)
				sns->state = I2C_STATE_CONF_SEND;
		}
		else
			sns->state = I2C_STATE_ERROR;
	}
	if ( (sns->state == I2C_STATE_CONF_SEND_DONE) || (sns->state == I2C_STATE_CONF_CHECK_RECEIVED_OK) )
	{
		tout1 = 0;
		status = HAL_I2C_IsDeviceReady(&hi2c3, addr | ADT_READ, 1, 1);
		if(status == HAL_OK)
		{
			status = HAL_I2C_Mem_Read_IT(&hi2c3, addr | ADT_READ, ADT_REG_TEMP_MSB, 1, i2c_rec_dataA, 2);
			if(status == HAL_OK)
				sns->state = I2C_STATE_READ_TEMP_QUERY;
		}
		else
			sns->state = I2C_STATE_ERROR;
	}
	if (sns->state == I2C_STATE_READ_TEMP_RECEIVE)
	{
		tout1 = 0;
		sns->state = I2C_STATE_READ_ID;
	}
	if (sns->state == I2C_STATE_READ_ID)
	{
		tout1 = 0;
		addr = getEEPROMAddr(act_sens[0]);
		status = HAL_I2C_IsDeviceReady(&hi2c3, addr | ADT_READ, 1, 1);
		if(status == HAL_OK)
		{
			if(sns->meas_cnt > 10)
				status = HAL_I2C_Mem_Read_IT(&hi2c3, addr | ADT_READ, 0, 1, (uint8_t*)&sid[act_sens[0]], 2);
			else
				status = HAL_I2C_Mem_Read_IT(&hi2c3, addr | ADT_READ, 0, 1, dump, 2);
			if(status == HAL_OK)
				sns->state = I2C_STATE_READ_ID_QUERY;
		}
		else
			sns->state = I2C_STATE_ERROR;
	}
	if (sns->state == I2C_STATE_READ_ID_REC)
	{
		tout1 = 0;
		sns->state = I2C_STATE_IDLE;
		sns->err_cnt = 0;
		if(sns->meas_cnt <= 100)	sns->meas_cnt++;
		nextSens(0);
	}
	if (sns->state == I2C_STATE_ERROR)
	{
		tout1 = 0;
		if (status == HAL_BUSY || (I2C3->CR1 & I2C_CR1_STOP))
			RST_i2c(&hi2c3);
		sns->state = I2C_STATE_IDLE;
			uint8_t temper_no = act_sens[0];
			//temperature[temper_no] = 0;
			//temperature_raw[temper_no] = 0;
			if(sns->err_cnt<10)
				sns->err_cnt++;
			else
				sid[temper_no] = 0xFFFF;
			sns->meas_cnt = 0;
			//avgResetAct(temper_no);
			nextSens(0);

	}
	if(++tout1 > 3)
		sns->state = I2C_STATE_ERROR;

	status = HAL_OK;

	sns = &sens[1][act_sens[1]];
	addr = getAddr(act_sens[1]);
	if (sns->state == I2C_STATE_IDLE && meas_perm2>0)
	{
		meas_perm2 = 0;
		tout2=0;
		status = HAL_I2C_IsDeviceReady(&hi2c2, addr | ADT_READ, 1, 1);
		if(status == HAL_OK)
		{
			status = HAL_I2C_Mem_Read_IT(&hi2c2, addr | ADT_READ, ADT_REG_CONFIG, 1, i2c_rec_dataB, 1);
			if(status == HAL_OK)
				sns->state = I2C_STATE_CONF_CHECK_QUERY;
		}
		else
			sns->state = I2C_STATE_ERROR;
	}
	if (sns->state == I2C_STATE_CONF_CHECK_RECEIVED_FAIL)
	{
		tout2=0;
		status = HAL_I2C_IsDeviceReady(&hi2c2, addr | ADT_READ, 1, 1);
		if(status == HAL_OK)
		{
			i2c_snd_dataB[0] = 0x80;
			status = HAL_I2C_Mem_Write_IT(&hi2c2, addr, ADT_REG_CONFIG, 1, i2c_snd_dataB, 1);
			if(status == HAL_OK)
				sns->state = I2C_STATE_CONF_SEND;

		}
		else
			sns->state = I2C_STATE_ERROR;
	}
	if ( (sns->state == I2C_STATE_CONF_SEND_DONE) || (sns->state == I2C_STATE_CONF_CHECK_RECEIVED_OK) )
	{
		tout2=0;
		status = HAL_I2C_IsDeviceReady(&hi2c2, addr | ADT_READ, 1, 1);
		if(status == HAL_OK)
		{
			status = HAL_I2C_Mem_Read_IT(&hi2c2, addr | ADT_READ, ADT_REG_TEMP_MSB, 1, i2c_rec_dataB, 2);
			if(status == HAL_OK)
				sns->state = I2C_STATE_READ_TEMP_QUERY;

		}
		else
			sns->state = I2C_STATE_ERROR;
	}
	if (sns->state == I2C_STATE_READ_TEMP_RECEIVE)
	{
		tout2=0;
		sns->state = I2C_STATE_READ_ID;
	}
	if (sns->state == I2C_STATE_READ_ID)
	{
		tout2=0;
		addr = getEEPROMAddr(act_sens[1]);
		status = HAL_I2C_IsDeviceReady(&hi2c2, addr | ADT_READ, 1, 1);
		if(status == HAL_OK)
		{
			if(sns->meas_cnt > 10)
				status = HAL_I2C_Mem_Read_IT(&hi2c2, addr | ADT_READ, 0, 1, (uint8_t*)&sid[act_sens[1]+8], 2);
			else
				status = HAL_I2C_Mem_Read_IT(&hi2c2, addr | ADT_READ, 0, 1, dump, 2);
			if(status == HAL_OK)
				sns->state = I2C_STATE_READ_ID_QUERY;
		}
		else
			sns->state = I2C_STATE_ERROR;
	}
	if (sns->state == I2C_STATE_READ_ID_REC)
	{
		tout2=0;
		sns->state = I2C_STATE_IDLE;
		sns->err_cnt = 0;
		if(sns->meas_cnt <= 100)	sns->meas_cnt++;
		nextSens(1);
	}
	if (sns->state == I2C_STATE_ERROR)
	{
		tout2 = 0;
		if (status == HAL_BUSY || (I2C2->CR1 & I2C_CR1_STOP))
			RST_i2c(&hi2c2);
		sns->state = I2C_STATE_IDLE;
		uint8_t temper_no = 8 + act_sens[1];
		//temperature[temper_no] = 0;
		//temperature_raw[temper_no] = 0;
		if(sns->err_cnt<10)
			sns->err_cnt++;
		else
			sid[temper_no] = 0xFFFF;
		sns->meas_cnt = 0;


		//avgResetAct(temper_no);
		nextSens(1);
	}
	if(++tout2 > 3)
		sns->state = I2C_STATE_ERROR;

}

