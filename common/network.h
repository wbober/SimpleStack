#ifndef _NETWORK_H
#define _NETWORK_H

#include <inttypes.h>
#include <stdlib.h>
#include "cmd.h"
#include "defines.h"

/*!
	Net layer result codes
 */
#define NET_ERROR	0x00
#define NET_OK		0x01

/*!
 */
#define NET_CLOCK_RES           100

/*!
 */
#define NET_PACKET_LOAD_SIZE    10

/*!
 */
#define NET_ROUTE_TABLE_SIZE    5
#define NET_ROUTE_RETRIES       2

/*!
 */
#define NET_ROUTE_VALID         0
#define NET_ROUTE_UNKNOWN_SEQ   1
#define NET_ROUTE_REQ_TTL       (5000/NET_CLOCK_RES)

#define NET_HOP_CNT(p)          ((p)->r_flags & 0xF0)
#define NET_INC_HOP_CNT(p)      ((p)->r_flags += 0x10)
#define NET_IS_ROUTE_VALID(nid)     (net_ctx.n_route[nid].r_flags & _BV(NET_ROUTE_VALID))

#define NET_BROADCAST_NID       0

/*!
 */
#ifdef NET_ROUTING
typedef struct _net_route_entry_t {
    uint8_t r_flags;
    uint16_t r_seq;
    uint16_t r_req_id;
    uint16_t r_req_ttl;
    uint8_t r_next_hop;
    uint8_t r_prev_list;
} net_route_entry_t, *pnet_route_entry_t;
#endif

#define NET_RREQ_UNKNOWN_SEQ     0
#define NET_RREQ_TIMEOUT         30000

/*!
    Route request packet
 */
#ifdef NET_ROUTING
typedef struct _net_rreq_t {
    uint8_t r_flags;
    uint16_t r_req_id;
    uint8_t r_dest_addr;
    uint16_t r_dest_seq;
    uint8_t r_org_addr;
    uint16_t r_org_seq;
} net_rreq_t, *pnet_rreq_t;
#endif

/*! 
    Route reply packet
 */
#ifdef NET_ROUTING
typedef struct _net_rrep_t {
    uint8_t r_flags;
    uint8_t r_dest_addr;
    uint16_t r_dest_seq;
    uint8_t r_org_addr;
} net_rrep_t, *pnet_rrep_t;
#endif

/*!
    Packet type
 */
typedef enum _net_packet_type_t {
    NET_DTA,
    NET_RREQ,
    NET_RREP,
    NET_RERR
} net_packet_type_t;


/*!
 */
typedef struct _net_packet_t {
    net_packet_type_t p_type;
    uint8_t p_src_addr;
    uint8_t p_dest_addr;
    uint8_t p_size;
    uint8_t p_load[NET_PACKET_LOAD_SIZE];
} net_packet_t, *pnet_packet_t;

/*!
 */
#define net_packet_size(pp) (sizeof(net_packet_t) - NET_PACKET_LOAD_SIZE + (pp)->p_size)

/*!
	Net layer functions
 */
void 	net_init(pcfg_t cfg);
uint8_t net_listen();
uint16_t net_send(uint8_t addr, uint8_t *buffer, uint8_t size);
uint8_t	net_recv(uint8_t *addr, void *buffer, uint8_t size);


#endif
