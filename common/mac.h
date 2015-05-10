#ifndef _MAC_H_
#define _MAC_H_

#include "link.h"

/**
    \defgroup mac <mac.h> MAC sublayer 
 */
#define MAC_CARRIER_LVL     -80
#define MAC_ACK_TIMEOUT     150
#define MAC_DTA_RETRIES     3
#define MAC_DTA_DEFER       200
#define MAC_DTA_MIN_DEFER   100

#define MAC_BROADCAST_NID   0

#define MAC_PREAMBLE 		0xAA
#define MAC_PREAMBLE_TX	    0x80
#define MAC_PREAMBLE_RX	    0x40
#define MAC_SOF1			0x33
#define MAC_SOF2			0xCC

#define MAC_MTU             sizeof(link_frame_t)
#define MAC_HAS             5

typedef enum _mac_tcvr_state_t {
    MAC_TCVR_PD,                
    MAC_TCVR_RX,
    MAC_TCVR_TX,
    MAC_TCVR_LISTEN,
    MAC_TCVR_LOCK,
} mac_tcvr_state_t;

typedef enum _mac_trx_state_ {
    MAC_TRX_PREAMBLE,
    MAC_TRX_SOF1,
    MAC_TRX_SOF2,
    MAC_TRX_SYNC,
    MAC_TRX_DONE
} mac_trx_state_t;

typedef enum _mac_state_t {
    MAC_IDLE = 0,
    MAC_WFACK,
    MAC_DEFER,
    MAC_SUPRISE
} mac_state_t;

typedef enum _mac_event_t {
    MAC_EVENT_RECV,
    MAC_EVENT_TXC,
    MAC_EVENT_TIMEOUT
} mac_event_t;

typedef struct _mac_history_t {
    uint8_t h_addr;
    uint16_t h_seq;
} mac_history_t;

typedef struct _mac_debug_t {
    int8_t   d_rssi;    
    uint16_t d_received;
    uint16_t d_sent;
    uint16_t d_lost;
    uint16_t d_corrupted;
    uint16_t d_ack_fail;
} mac_debug_t, *pmac_debug_t;

typedef uint8_t (*mac_state_handler_t)(mac_event_t event, plink_frame_t f);

uint8_t mac_send(plink_frame_t fp);
void mac_timeout(uint8_t t_id);
void mac_init(pcfg_t);
void mac_irq_handler();
pmac_debug_t mac_debug_info();

#endif
