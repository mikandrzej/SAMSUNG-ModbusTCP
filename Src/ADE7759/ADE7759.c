/*
 * ADE7759.c
 *
 *  Created on: 04.09.2018
 *      Author: Miki
 */

#include "main.h"
#include "stm32f4xx_hal.h"

#include "ADE7759.h"

#include "../Izo_MODBUS_TCP_SRV/MODB_TCP_srv/Modbus_data.h"
#include "../common/IOports.h"

extern SPI_HandleTypeDef hspi5;

int16_t APOS = -8700;
int16_t CH1OFFS = 0;
int16_t CH2OFFS = 0;
int8_t PHCAL = 0;
uint8_t CH1GN = 0;
uint8_t CH2GN = 0;
uint8_t CH1SCALE = 0;

uint8_t ade_ready = 0;

double POWER;

void ADE7759_WriteReg8(uint8_t reg, uint8_t data)
{
	uint8_t raw[2];
	raw[0] = reg | ADE_WRITE;
	raw[1] = data;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 2, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
}

void ADE7759_ReadReg8(uint8_t reg, uint8_t *data)
{
	uint8_t raw[1];
	raw[0] = reg;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 1, 1000);
	HAL_SPI_Receive(&hspi5, raw, 1, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	if (data)
		*data = raw[0];
}

void ADE7759_WriteReg16(uint8_t reg, uint16_t data)
{
	uint8_t raw[3];
	raw[0] = reg | ADE_WRITE;
	raw[1] = (data>>8) & 0xFF;
	raw[2] = data & 0xFF;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 3, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
}

void ADE7759_ReadReg16(uint8_t reg, uint16_t *data)
{
	uint8_t raw[2];
	raw[0] = reg;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 1, 1000);
	HAL_SPI_Receive(&hspi5, raw, 2, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	uint16_t tmp;
	tmp = raw[0];
	tmp <<= 8;
	tmp |= raw[1];
	if(data)
		*data = tmp;
}

void ADE7759_ReadWave(int32_t *CH1, int32_t *CH2)
{
	uint8_t raw[5];
	raw[0] = ADE_REG_WAVEFORM;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 1, 1000);
	HAL_SPI_Receive(&hspi5, raw, 5, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);

	static volatile int32_t temp;

	temp = raw[0] & 0x0F;
	temp <<=8;
	temp |= raw[1];
	temp <<=8;
	temp |= raw[2];
	temp <<=12;
	temp /= 4096;
	if(CH2)
		*CH2 = temp;

	temp = raw[0] & 0xF0;
	temp <<=4;
	temp |= raw[3];
	temp <<=8;
	temp |= raw[4];
	temp <<=12;
	temp /= 4096;
	if(CH1)
		*CH1 = temp;
}

void ADE7759_Read1Wave(int32_t *CH1)
{
	uint8_t raw[3];
	raw[0] = ADE_REG_WAVEFORM;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 1, 1000);
	HAL_SPI_Receive(&hspi5, raw, 3, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);

	static volatile int32_t temp;


	temp = raw[0];
	temp <<=8;
	temp |= raw[1];
	temp <<=8;
	temp |= raw[2];

	if(CH1)
		*CH1 = temp;
}



void ADE7759_ReadLineCycleEnergy(uint64_t *energy)
{
	uint8_t raw[5];
	raw[0] = ADE_REG_LENERGY;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 1, 1000);
	HAL_SPI_Receive(&hspi5, raw, 5, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);

	static volatile int64_t temp = 0;
	temp = raw[0];
	temp <<= 8;
	temp |= raw[1];
	temp <<= 8;
	temp |= raw[2];
	temp <<= 8;
	temp |= raw[3];
	temp <<= 8;
	temp |= raw[4];
	temp <<=24;
	temp /= 256;
	temp /= 256;
	temp /= 256;
	temp += 3000000;

	temp /= 100;

	if(temp < 0)
		temp = -temp;

	double temp_pow;
	temp_pow = temp;
	temp_pow /= POW_CONST_DIV;

	POWER = temp_pow;
	if(energy)
		*energy = temp;
}

void ADE7759_ReadActiveEnergy(uint64_t *energy)
{
	uint8_t raw[5];
	raw[0] = ADE_REG_AENERGY;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 1, 1000);
	HAL_SPI_Receive(&hspi5, raw, 5, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);

	static volatile int64_t temp = 0;
	temp = raw[0];
	temp <<= 8;
	temp |= raw[1];
	temp <<= 8;
	temp |= raw[2];
	temp <<= 8;
	temp |= raw[3];
	temp <<= 8;
	temp |= raw[4];
	temp <<=24;
	temp /= 256;
	temp /= 256;
	temp /= 256;
	if(energy)
		*energy = temp;
}

void ADE7759_ReadRstActiveEnergy(uint64_t *energy)
{
	uint8_t raw[5];
	raw[0] = ADE_REG_RSTENERGY;
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 0);
	HAL_SPI_Transmit(&hspi5, raw, 1, 1000);
	HAL_SPI_Receive(&hspi5, raw, 5, 1000);
	HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port,SPI5_CSS_Pin, 1);

	static volatile int64_t temp = 0;
	temp = raw[0];
	temp <<= 8;
	temp |= raw[1];
	temp <<= 8;
	temp |= raw[2];
	temp <<= 8;
	temp |= raw[3];
	temp <<= 8;
	temp |= raw[4];
	temp <<=24;
	temp /= 256;
	temp /= 256;
	temp /= 256;
	if(energy)
		*energy = temp;
}

void ADE7759_Init(void)
{
	ADE_tim = 0;
	ade_ready = 0;

	ADE7759_WriteReg16(ADE_REG_MODE,
			ADE_SWRST);
	HAL_Delay(10);
	uint8_t status;
	do
	{
		ADE7759_ReadReg8(ADE_REG_RSTSTATUS, &status);
	}while(status & ADE_RESET);

	ADE7759_WriteReg16(ADE_REG_MODE,
			ADE_DTRT_3k6 | ADE_WAV_CH1CH2 | ADE_CYCMODE);
	ADE7759_WriteReg8(ADE_REG_GAIN, ((CH2GN&0b111)<<5) | ((CH1SCALE&0b11)<<3) | (CH1GN&0b111));
	ADE7759_WriteReg16(ADE_REG_APOS, APOS);
	ADE7759_WriteReg16(ADE_REG_LINECYC, 10);
	ADE7759_WriteReg8(ADE_REG_CH1OS, ADE_CH1INTEGR);
	ADE7759_ReadReg8(ADE_REG_RSTSTATUS, NULL);
	ADE7759_WriteReg8(ADE_REG_IRQEN,
			ADE_CYCEND);


}


void ADE7759_Process(void)
{
	if ((ADE_tim) >= 1000)
	{
		HAL_GPIO_TogglePin(LAMP_READY_GPIO_Port, LAMP_READY_Pin);
		ADE7759_Init();
	}
}


void ADE7759_Interrupt(void)
{
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	ade_ready = 1;
	ADE_tim = 0;
	uint8_t regval;
	ADE7759_ReadReg8(ADE_REG_RSTSTATUS, &regval);
	if (regval & ADE_AEHF)
	{

	}
	if (regval & ADE_SAG)
	{

	}
	if (regval & ADE_CYCEND)
	{
		ADE7759_ReadLineCycleEnergy(&LENERGY);
		LENERGY_32 = (int32_t) LENERGY;
		uint16_t temp;
		if (LENERGY > 0 || 1)
			temp = (uint32_t) (LENERGY/POW_CONST_DIV);
		else
			temp = 0; //(uint32_t) ((0-LENERGY)/WATT_CONST);
		ModbusRegInsertData(MODB_REG_POWER, temp);
	}
	if (regval & ADE_WSMP)
	{
		ADE7759_ReadWave(&WAVE1, &WAVE2);
	}
	if (regval & ADE_ZX)
	{

	}
	if (regval & ADE_TEMP)
	{

	}
	if (regval & ADE_RESET)
	{

	}
	if (regval & ADE_AEOF)
	{

	}
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}
