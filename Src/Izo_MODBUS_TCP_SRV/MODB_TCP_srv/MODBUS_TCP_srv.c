/*
 * MODBUS_TCP_srv.c
 *
 *  Created on: 06.07.2018
 *      Author: Miki
 */

#include "MODBUS_TCP_srv.h"
#include "lwip/tcp.h"

#include "MODBUS_TCP_functions.h"

extern int8_t modb_timeout;

err_t TCP_Modbus_Parse(struct tcp_pcb *pcb, struct pbuf *p)
{
		modb_timeout = 0;


	uint16_t len = p->len;
	char *data = (char *)p->payload;
	err_t err = ERR_OK;

	if (!len) return err;

	char rep[1500];
	uint16_t rep_len;


	if (len < 8) return err;

	uint16_t ti;	//transaction identifier - byte 0-1
	uint16_t pi;	//protocol identifier - byte 2-3
	uint16_t lf;	//length field
	uint8_t ui;		//unit identifier
	uint8_t cmd;	//cmd field

	ti	= data[0]<<8 | data[1];
	pi	= data[2]<<8 | data[3];
	lf	= data[4]<<8 | data[5];
	ui	= data[6];
	cmd = data[7];
	switch(cmd)
	{
	case 0x03:	modb_func_0x03(data+8, len-8, rep, &rep_len); break;	//Odczyt n rejestrow
	//case 0x04:	modb_func_0x04(data+8, len-8, rep, &rep_len); break;	//Odczyt n rejestrow wejsciowych
	case 0x06:	modb_func_0x06(data+8, len-8, rep, &rep_len); break;	//Zapis rejestru
	//case 0x07:	modb_func_0x07(data+8, len-8, rep, &rep_len); break;	//Odczyt statusu
	//case 0x08:	modb_func_0x08(data+8, len-8, rep, &rep_len); break;	//Odczyt statusu
	case 0x10:	modb_func_0x10(data+8, len-8, rep, &rep_len); break;	//Odczyt n rejestrow
	//case 0x11:	modb_func_0x11(data+8, len-8, rep, &rep_len); break;	//Identyfikacja urzadzenia
	default:
		rep_len = 0;
	}

	lf = rep_len + 2;
	data[4] = (lf>>8) & 0xFF;
	data[5] = lf & 0xFF;

	if (rep_len)
	{
		err = tcp_write(pcb, data, 8, 0);
		err = tcp_write(pcb, rep, rep_len, 0);
	}
	else
	{
		rep_len = sprintf(rep, "NO DATA");
		err = tcp_write(pcb, rep, rep_len, 0);
	}
	tcp_output(pcb);

	return err;
}

