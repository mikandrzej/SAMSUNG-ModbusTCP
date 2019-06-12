/*
 * MODBUS_TCP_functions.h
 *
 *  Created on: 06.07.2018
 *      Author: Miki
 */

#ifndef IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_TCP_FUNCTIONS_H_
#define IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_TCP_FUNCTIONS_H_

err_t modb_func_0x03 (char *data, uint16_t len, char *rep, uint16_t *rep_len);
err_t modb_func_0x04 (char *data, uint16_t len, char *rep, uint16_t *rep_len);
err_t modb_func_0x06 (char *data, uint16_t len, char *rep, uint16_t *rep_len);
err_t modb_func_0x07 (char *data, uint16_t len, char *rep, uint16_t *rep_len);
err_t modb_func_0x08 (char *data, uint16_t len, char *rep, uint16_t *rep_len);
err_t modb_func_0x10 (char *data, uint16_t len, char *rep, uint16_t *rep_len);
err_t modb_func_0x11 (char *data, uint16_t len, char *rep, uint16_t *rep_len);


#endif /* IZO_MODBUS_TCP_SRV_MODB_TCP_SRV_MODBUS_TCP_FUNCTIONS_H_ */
