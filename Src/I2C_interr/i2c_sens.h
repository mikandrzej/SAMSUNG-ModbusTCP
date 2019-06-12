/*
 * i2c_sens.h
 *
 *  Created on: 03.10.2018
 *      Author: Miki
 */

#ifndef I2C_INTERR_I2C_SENS_H_
#define I2C_INTERR_I2C_SENS_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define I2C_STATE_IDLE 1
#define I2C_STATE_CONF_CHECK_QUERY	2
#define I2C_STATE_CONF_CHECK_RECEIVE	3
#define I2C_STATE_CONF_CHECK_RECEIVED_FAIL	4
#define I2C_STATE_CONF_CHECK_RECEIVED_OK	5
#define I2C_STATE_CONF_SEND	6
#define I2C_STATE_CONF_SEND_DONE	7
#define I2C_STATE_READ_TEMP_QUERY	10
#define I2C_STATE_READ_TEMP_RECEIVE 15
#define I2C_STATE_READ_ID	16
#define I2C_STATE_READ_ID_QUERY	17
#define I2C_STATE_READ_ID_REC	18
#define I2C_STATE_TRANSMIT_END	20
#define I2C_STATE_ERROR		25

#define ADT_REG_TEMP_MSB			0x00
#define ADT_REG_TEMP_LSB			0x01
#define ADT_REG_STATUS				0x02
#define ADT_REG_CONFIG				0x03
#define ADT_REG_TEMP_HIG_SP_MSB		0x04
#define ADT_REG_TEMP_HIG_SP_LSB		0x05
#define ADT_REG_TEMP_LOW_SP_MSB		0x06
#define ADT_REG_TEMP_LOW_SP_LSB		0x07
#define ADT_REG_TEMP_CRIT_SP_MSB	0x08
#define ADT_REG_TEMP_CRIT_SP_LSB	0x09
#define ADT_REG_TEMP_HYST_SP		0x0A
#define ADT_REG_ID					0x0B
#define ADT_REG_SW_RESET			0x0F

typedef struct{
	uint16_t state;
	uint16_t err_cnt;
	uint16_t meas_cnt;
}s_i2c_sens;

#endif /* I2C_INTERR_I2C_SENS_H_ */
