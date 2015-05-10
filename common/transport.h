#ifndef _TRANSPORT_H_
#define _TRANSPORT_H_

#include <inttypes.h>
#include <defines.h>

#define TCP_TIMEOUT		7000 // 1000 ms

#define	TCP_ERROR		0
#define TCP_OK			1

uint8_t tcp_init(pcfg_t);
uint8_t tcp_listen();
uint16_t tcp_send(uint8_t addr, uint8_t *buffer, uint8_t size);
uint8_t tcp_recv(uint8_t *addr, void *buffer, uint8_t size);

#endif
