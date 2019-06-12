/*
 * i2c_selector.c
 *
 *  Created on: 03.10.2018
 *      Author: Miki
 */

#include "i2c_selector.h"


#include "main.h"
#include "stm32f4xx_hal.h"

void selectInputI2C(uint8_t port, uint8_t inp)
{
	switch (port)
	{
	case 0:
		switch (inp)
		{
		case 0:
			HAL_GPIO_WritePin(I2C_SELECTOR1_S1_GPIO_Port, I2C_SELECTOR1_S1_Pin, 0);
			HAL_GPIO_WritePin(I2C_SELECTOR1_S0_GPIO_Port, I2C_SELECTOR1_S0_Pin, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(I2C_SELECTOR1_S1_GPIO_Port, I2C_SELECTOR1_S1_Pin, 0);
			HAL_GPIO_WritePin(I2C_SELECTOR1_S0_GPIO_Port, I2C_SELECTOR1_S0_Pin, 1);
			break;
		}
		break;
	case 1:
		switch (inp)
		{
		case 0:
			HAL_GPIO_WritePin(I2C_SELECTOR2_S1_GPIO_Port, I2C_SELECTOR2_S1_Pin, 0);
			HAL_GPIO_WritePin(I2C_SELECTOR2_S0_GPIO_Port, I2C_SELECTOR2_S0_Pin, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(I2C_SELECTOR2_S1_GPIO_Port, I2C_SELECTOR2_S1_Pin, 0);
			HAL_GPIO_WritePin(I2C_SELECTOR2_S0_GPIO_Port, I2C_SELECTOR2_S0_Pin, 1);
			break;
		}
		break;
	}
}
