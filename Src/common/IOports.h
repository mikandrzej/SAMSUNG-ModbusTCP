/*
 * IOports.h
 *
 *  Created on: 15.10.2018
 *      Author: Miko³aj Andrzejewski
 */

#include "main.h"

#ifndef COMMON_IOPORTS_H_
#define COMMON_IOPORTS_H_

#define LAMP_FRIDGE_POWER_GPIO_Port	Q0_0_GPIO_Port
#define LAMP_FRIDGE_POWER_Pin		Q0_0_Pin

#define FRIDGE_POWER_GPIO_Port		Q0_2_GPIO_Port
#define FRIDGE_POWER_Pin			Q0_2_Pin

#define LAMP_READY_GPIO_Port		Q0_1_GPIO_Port
#define LAMP_READY_Pin				Q0_1_Pin

#define LAMP_I2C_GPIO_Port			USER_LED1_GPIO_Port
#define LAMP_I2C_Pin				USER_LED1_Pin
#define LAMP_AC_GPIO_Port			USER_LED2_GPIO_Port
#define LAMP_AC_Pin					USER_LED2_Pin
#define LAMP_RUN_GPIO_Port			USER_LED3_GPIO_Port
#define LAMP_RUN_Pin				USER_LED3_Pin

#endif /* COMMON_IOPORTS_H_ */
