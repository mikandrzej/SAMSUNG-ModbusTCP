/*
 * temp_avg.c
 *
 *  Created on: 14.11.2018
 *      Author: Miki
 */

#include <stdio.h>

#include "temp_avg.h"


void avgInit(void){
	for(int k=0; k<avg_values; k++)	{
		avgValuesInReg[k] = 0;
		avgActVal[k] = 0;
	}
	avg_values = 100;
}

void avgAddValue(uint16_t sens, int16_t value){
	if (sens > 15)
		return;
	avgVals[sens][avgActVal[sens]] = value;

	if( avgActVal[sens] > avgValuesInReg[sens] )
		avgValuesInReg[sens] = avgActVal[sens]+1;
	else if(avgValuesInReg[sens] > avg_values)
		avgValuesInReg[sens] = avg_values;

	if ( (avgActVal[sens] + 1) >= avg_values)	{
		avgActVal[sens] = 0;
	}
	else	{
		avgActVal[sens]++;
	}
}


int16_t avgGetValue(uint16_t sens){
	int32_t temp = 0;
	for(int k=0; k<avgValuesInReg[sens]; k++)
		temp += avgVals[sens][k];
	if((avgValuesInReg[sens]) != 0)
		temp /= avgValuesInReg[sens];
	else
		temp = 0;
	return (int16_t)temp;
}

void avgResetAct(uint16_t sens){
	avgActVal[sens] = 0;
	avgValuesInReg[sens]=0;
}
