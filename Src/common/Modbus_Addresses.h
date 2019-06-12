/*
 * Modbus_Addresses.h
 *
 *  Created on: 15.10.2018
 *      Author: Miko³aj Andrzejewski
 */

#ifndef COMMON_MODBUS_ADDRESSES_H_
#define COMMON_MODBUS_ADDRESSES_H_

#define	MODB_REG_CONTROL		0
#define	MODB_REG_STATUS			1
#define	MODB_REG_EVENT_ID		2
#define	MODB_REG_POWER			3
#define	MODB_REG_CYCLE			4

#define MODB_REG_SENS1_SID		10
#define MODB_REG_SENS1_VAL		11
#define MODB_REG_SENS_OFFSET	2
#define MODB_REG_SENSORS		16

#define MODB_CONTROL_RELAY_MASK (1<<0)
#define MODB_STATUS_FUSE_230	((uint16_t)(1<<1))



#endif /* COMMON_MODBUS_ADDRESSES_H_ */
