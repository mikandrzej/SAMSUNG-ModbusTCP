/*
 * Izo_MODBUS_TCP_SRV.c
 *
 *  Created on: 06.07.2018
 *      Author: Miki
 */

#include "Izo_MODBUS_TCP_SRV.h"
#include "MODB_TCP_srv/MODBUS_TCP_srv.h"

#include "main.h"
#include "lwip/tcp.h"

// Local variables
struct tcp_pcb *pMod_pcb;

// Local function declarations
void TCP_close_conn(struct tcp_pcb *pcb);
err_t TCP_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
err_t TCP_accept(void *arg, struct tcp_pcb *pcb, err_t err);
err_t TCP_poll(void *arg, struct tcp_pcb *pcb);

// Function definitions
err_t TCP_Modbus_init(uint16_t port)
{
	err_t err = ERR_OK;

	for(int k=0; k<MEMP_NUM_TCP_PCB; k++)
	{
		pcby[k].active = 0;
		pcby[k].pcb = NULL;
	}

    pMod_pcb = tcp_new();
    err = tcp_bind(pMod_pcb, IP_ADDR_ANY, port);

	return err;
}


void TCP_Modbus_Process()
{
	pMod_pcb = tcp_listen(pMod_pcb);
	tcp_accept(pMod_pcb, TCP_accept);
}


void TCP_close_conn(struct tcp_pcb *pcb)
{
  	for(int k=0; k<MEMP_NUM_TCP_PCB; k++)
  	{
  		if(pcby[k].pcb == pcb)
  		{
  			pcby[k].active = 0;
  			pcby[k].pcb = NULL;
  			act_conns--;
  			break;
  		}
  	}

    tcp_arg(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_close(pcb);
}


err_t TCP_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
      int len;

      if (err == ERR_OK && p != NULL) {
            /* Inform TCP that we have taken the data. */
            tcp_recved(pcb, p->tot_len);

            TCP_Modbus_Parse(pcb, p);



           //Free the packet buffer
            pbuf_free(p);

            //check output buffer capacity
            if (len >tcp_sndbuf(pcb)) len= tcp_sndbuf(pcb);
            //Send out the data
            tcp_sent(pcb, NULL); /* No need to call back */
      } else {
            pbuf_free(p);
      }

      if (err == ERR_OK && p == NULL) {
    	  TCP_close_conn(pcb);
      }
      return ERR_OK;
}



err_t TCP_accept(void *arg, struct tcp_pcb *pcb, err_t err){
      LWIP_UNUSED_ARG(arg);
      LWIP_UNUSED_ARG(err);


  	for(int k=0; k<MEMP_NUM_TCP_PCB; k++)
  	{
  		if(! pcby[k].active)
  		{
  			pcby[k].active = 1;
  			pcby[k].pcb = pcb;
  			act_conns++;
  			break;
  		}
  	}

      tcp_setprio(pcb, TCP_PRIO_MIN);
      tcp_recv(pcb, TCP_recv);
      tcp_err(pcb, NULL); //Don't care about error here
      tcp_poll(pcb, NULL, 4); //No polling here
      return ERR_OK;
}

err_t TCP_poll(void *arg, struct tcp_pcb *pcb)
{
	err_t err = ERR_OK;

	return err;
}
