/*
 * MODBUS_TCP_srv.h
 *
 *  Created on: 06.07.2018
 *      Author: Miki
 */

#ifndef IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_TCP_SRV_H_
#define IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_TCP_SRV_H_

#include "lwip/tcp.h"

err_t TCP_Modbus_Parse(struct tcp_pcb *pcb, struct pbuf *p);

#endif /* IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_TCP_SRV_H_ */
