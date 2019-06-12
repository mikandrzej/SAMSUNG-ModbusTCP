/*
 * temp_avg.h
 *
 *  Created on: 14.11.2018
 *      Author: Miki
 */

#ifndef I2C_INTERR_TEMP_AVG_H_
#define I2C_INTERR_TEMP_AVG_H_


int16_t avgVals[16][101];
uint16_t avgActVal[16];
uint16_t avgValuesInReg[16];
uint16_t avg_values;

void avgInit(void);
void avgAddValue(uint16_t sens, int16_t value);
int16_t avgGetValue(uint16_t sens);
void avgResetAct(uint16_t sens);


#endif /* I2C_INTERR_TEMP_AVG_H_ */
