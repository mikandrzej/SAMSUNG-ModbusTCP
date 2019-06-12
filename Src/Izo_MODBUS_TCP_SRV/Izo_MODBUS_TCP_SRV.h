/*
 * Izo_MODBUS_TCP_SRV.h
 *
 *  Created on: 06.07.2018
 *      Author: Miki
 */

#ifndef IZO_MODBUS_TCP_SRV_IZO_MODBUS_TCP_SRV_H_
#define IZO_MODBUS_TCP_SRV_IZO_MODBUS_TCP_SRV_H_

#include "main.h"
#include "lwip.h"

typedef struct{
	struct tcp_pcb *pcb;
	uint8_t active;
}s_pcby;
s_pcby pcby[MEMP_NUM_TCP_PCB];
uint8_t act_conns;

err_t TCP_Modbus_init(uint16_t port);
void TCP_Modbus_Process();

#endif /* IZO_MODBUS_TCP_SRV_IZO_MODBUS_TCP_SRV_H_ */
