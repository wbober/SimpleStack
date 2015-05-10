#include "transport.h"
#include "timers.h"
#include "event.h"
#include "network.h"

void tcp_net_rxc(ptr_t p);
void tcp_net_txc(ptr_t p);
void tcp_net_fail(ptr_t p);

uint8_t tcp_init(pcfg_t cfg)
{
	net_init(cfg);

    event_register(EVENT_NET_TXC, tcp_net_txc, false);
    event_register(EVENT_NET_RXC, tcp_net_rxc, false);
    event_register(EVENT_NET_FAIL, tcp_net_fail, false);

	return TCP_OK;
}

void tcp_net_fail(ptr_t p)
{
    event_activate(EVENT_TCP_FAIL, p);
}

void tcp_net_rxc(ptr_t p)
{
    event_activate(EVENT_TCP_RXC, NULL);
}

void tcp_net_txc(ptr_t p)
{
    event_activate(EVENT_TCP_TXC, p);
}

/*!
 */
uint16_t tcp_send(uint8_t addr, uint8_t *buffer, uint8_t size) 
{
	return net_send(addr, buffer, size);
}

/*!
 */
uint8_t tcp_recv(uint8_t *addr, void *buffer, uint8_t size)
{
	net_recv(addr, buffer, size);
	return NET_OK;
}
