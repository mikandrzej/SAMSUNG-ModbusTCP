/*
 * i2c_interr.h
 *
 *  Created on: 03.10.2018
 *      Author: Miki
 */

#ifndef I2C_INTERR_I2C_INTERR_H_
#define I2C_INTERR_I2C_INTERR_H_

#include "stm32f4xx_hal.h"

#include "i2c_sens.h"
#define ADT_ADDRESS1	0b10010000
#define ADT_ADDRESS2	0b10010010
#define ADT_ADDRESS3	0b10010100
#define ADT_ADDRESS4	0b10010110
#define EEPROM_ADDRESS1	0b10100000
#define EEPROM_ADDRESS2	0b10100010
#define EEPROM_ADDRESS3	0b10100100
#define EEPROM_ADDRESS4	0b10100110
#define ADT_READ	1
#define EEPROM_READ	1


double temperature[16];
uint16_t temperature_raw[16];
uint16_t sid[16];
uint8_t meas_perm1;
uint8_t meas_perm2;



uint16_t err_cnt_i2c;

uint16_t i2c_err_cnt1, i2c_err_cnt2;

s_i2c_sens sens[2][8];
void i2c_interr_init(void);
void task_Temperature(void);

#endif /* I2C_INTERR_I2C_INTERR_H_ */
