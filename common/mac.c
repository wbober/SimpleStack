#include <avr/io.h>
#include <util/crc16.h>	
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <assert.h>
#include "event.h"
#include "link.h"
#include "rssi.h"
#include "cc1000.h"
#include "cc1000settings.h"
#include "defines.h"
#include "list.h"
#include "timers.h"
#include "mac.h"



#ifdef BASE
    #include "../Base/usart.h"

#ifdef DEBUG_MAC
    PGM_S(mac_event_recv_s, "MAC_EVENT_RECV");
    PGM_S(mac_event_timeout_s, "MAC_EVENT_TIMEOUT");
    PGM_S(mac_event_txc_s, "MAC_EVENT_TXC");

    PGM_P mac_event_string[4] = {mac_event_recv_s, mac_event_send_s, mac_event_txc_s, mac_event_timeout_s};

    PGM_S(mac_event_idle_s, "MAC_IDLE");
    PGM_S(mac_event_wfack_s,"MAC_WFACK");
    PGM_S(mac_event_defer_s, "MAC_DEFER");
    PGM_P mac_state_string[3] = {mac_event_idle_s, mac_event_wfack_s, mac_event_defer_s};
#endif
#endif

static struct {
    uint8_t             m_nid;
    uint8_t             m_counter;
    mac_trx_state_t     m_trx_state;           //<** state machine phase >
    mac_tcvr_state_t    m_tcvr_state;          //<** transceiver mode>
    uint8_t             m_buffer[MAC_MTU];     //<** currently transmited or received frame *>
    uint8_t             m_bp;
    mac_state_t         m_state;
    uint16_t            m_timeout;
    uint8_t             m_timer;               //<** timeout timer >  
    uint8_t             m_retries;
    uint16_t            m_backoff_timeout;
    plink_frame_t       m_tx_fp;               //<** pointer to currently tx frame *>
    link_frame_t        m_rx_f;                
    mac_history_t       m_history[MAC_HAS];    
    mac_debug_t         m_debug;

    #ifdef ACCEPT_ONE_NODE
    uint8_t m_accept_nid;
    #endif
} mac_ctx;

uint8_t mac_idle(mac_event_t, plink_frame_t);
uint8_t mac_wfack(mac_event_t, plink_frame_t);
uint8_t mac_defer(mac_event_t, plink_frame_t);

static uint8_t mac_handle_event(mac_event_t event, plink_frame_t f);
static uint8_t mac_send_frame(plink_frame_t f);
static uint8_t mac_switch_mode(mac_tcvr_state_t);
static volatile mac_state_handler_t mac_state_handlers[] = {mac_idle, mac_wfack, mac_defer};

/*!
    \brief Reset transceiver and transmission states
 */
static inline void mac_reset(void)
{
    LED_OFF(LED_GREEN);
    LED_OFF(LED_RED);
    mac_ctx.m_trx_state = MAC_TRX_PREAMBLE;
    mac_ctx.m_counter = MAC_PREAMBLE_RX;
    mac_ctx.m_bp = 0;
}

/*!
	\brief Receive bit from transceiver
 */
static inline void mac_recv_bit()
{
    // received data
    static uint8_t data = 0;

    data <<= 1;

	if (CC1000_DIO)
		data |= 0x01;
    else
        data &= ~0x01;

	// check for every bit
	switch (mac_ctx.m_trx_state) {
       
            // receiver is waiting for preamble
			case MAC_TRX_PREAMBLE: {
				if (mac_ctx.m_counter > 0) {
					// collect apropriate bits of preamble
					if ((data & 0x01) ^ ((data & 0x02) >> 1))
						mac_ctx.m_counter--;
					else
						mac_ctx.m_counter = MAC_PREAMBLE_RX;
				} else {
					// received enough preamble bits, now wait for SOF
					mac_ctx.m_trx_state = MAC_TRX_SOF1;
					mac_ctx.m_counter = MAC_PREAMBLE_TX - MAC_PREAMBLE_RX + 8;
				}
				break;
			}

            // receiver is waiting for first synchronization character
			case MAC_TRX_SOF1: {
                // if received data looks like first synch byte, then go to the next phase
			    if (data == MAC_SOF1) {
                    mac_ctx.m_trx_state = MAC_TRX_SOF2;
                    mac_ctx.m_counter = 0;
                } else {
                    // if not wait a little bit more
                    if (mac_ctx.m_counter == 0)
                        mac_reset();
                    else
                        mac_ctx.m_counter--;
                }
				break;
			}

            // waiting for second synch byte
			case MAC_TRX_SOF2: {
				// check if second byte of SOF word matches
                if (mac_ctx.m_counter == 7) {
                    if (data == MAC_SOF2) {
                        // the receiver is synchronized
                        mac_ctx.m_trx_state = MAC_TRX_SYNC;
                        mac_ctx.m_tcvr_state = MAC_TCVR_RX;
                        mac_ctx.m_counter = 0;
                        // sample rssi
                        rssi_measure();
                    } else
                        mac_reset(); // error, the byte after SOF1 is not the SOF2 byte
   
                } else 
                    mac_ctx.m_counter++;
				break;
			}

            // the receiver is synchronized
			case MAC_TRX_SYNC:
            	if (mac_ctx.m_counter == 7) {
		            mac_ctx.m_counter = 0;
                    LED_TOGGLE(LED_GREEN);
                    
                    if (mac_ctx.m_bp < MAC_MTU) {
                        mac_ctx.m_buffer[mac_ctx.m_bp] = data;
                        //
                        if (mac_ctx.m_bp > sizeof(link_frame_header_t)) {
                            if (mac_ctx.m_bp == sizeof(link_frame_header_t) + mac_ctx.m_buffer[offsetof(link_frame_header_t, f_size)] + 2) {
                                CC1000_DCLK_DISABLE;
                                mac_ctx.m_trx_state = MAC_TRX_DONE;
                                event_activate(EVENT_RADIO_RXC, mac_ctx.m_buffer);
                            }
                        }
                        mac_ctx.m_bp++;
                     } else 
                        mac_reset();
	            } else
            		mac_ctx.m_counter++;
				break;
            case MAC_TRX_DONE:
                CC1000_DCLK_DISABLE;                
            break;
	}
}

/*!
	Send bit from buffer to RF interface
 */
//static inline void mac_send_bit() __attribute__((always_inline));
static inline void mac_send_bit()
{
	static uint8_t cnt = 0x01;
    static uint8_t data = 0;

    if (cnt == 0x01) {
		cnt = 0x80;
        switch (mac_ctx.m_trx_state) {
                // transmit preamble
    			case MAC_TRX_PREAMBLE:
                    data = MAC_PREAMBLE;
    				if (--mac_ctx.m_counter == 0)
        				mac_ctx.m_trx_state = MAC_TRX_SOF1;
    				break;

                // transmit first synch byte
    			case MAC_TRX_SOF1:
                    data = MAC_SOF1;
                    mac_ctx.m_trx_state = MAC_TRX_SOF2;
    				break;

                // transmit second synch byte
    			case MAC_TRX_SOF2:
                    data = MAC_SOF2;
                    mac_ctx.m_trx_state = MAC_TRX_SYNC;
                    mac_ctx.m_counter = mac_ctx.m_bp;
                    mac_ctx.m_bp = 0;
                    break;

                // transmiter is synchronized
    			case MAC_TRX_SYNC:
                    LED_TOGGLE(LED_RED);
                    if (mac_ctx.m_bp < mac_ctx.m_counter) {
                        data = mac_ctx.m_buffer[mac_ctx.m_bp];
                        mac_ctx.m_bp++;
                    } else {
                        CC1000_DCLK_DISABLE;
                        mac_switch_mode(MAC_TCVR_PD);
                        event_activate(EVENT_RADIO_TXC, mac_ctx.m_buffer);
                        mac_ctx.m_trx_state = MAC_TRX_DONE;
                    }
                break;
                case MAC_TRX_DONE:
                break;
    	};       
	} else 
		cnt >>= 1;

    if (data & cnt)
		CC1000_DIO_HIGH;
	else
		CC1000_DIO_LOW;
}

/*!
    \brief Receive interrupt bottom half handler.
 */
void mac_rxc(ptr_t p)
{
    uint16_t crc = 0;
    uint8_t i, *dp;
    uint8_t listen = 1;
    plink_frame_t r = (plink_frame_t)p;

    LED_OFF(LED_GREEN);
    
    // compute header crc
    for (i = 0, dp = (uint8_t *)&r->f_header; i < sizeof(link_frame_header_t); i++, dp++) {
        crc = _crc_xmodem_update(crc, *dp);
    }

    // compute load's crc
    for (i = 0, dp = r->f_load; i < r->f_header.f_size; i++, dp++) {
        crc = _crc_xmodem_update(crc, *dp);
    }

    // get crc value from buffer
    r->f_crc = r->f_load[i + 1] << 8 | r->f_load[i]; 

    // if crc is correct handle received frame
	if (r->f_crc == crc) {
        if ((r->f_header.f_dest_addr == mac_ctx.m_nid) || (r->f_header.f_dest_addr == MAC_BROADCAST_NID)) {
            mac_ctx.m_debug.d_rssi = rssi_get_value();
            mac_ctx.m_debug.d_received++;
            DBG_MAC(usart_printf(PSTR("MAC RXC %S dst:%d src:%d\r\n"), r->f_header.f_flags == LINK_DTA?PSTR("DTA"):PSTR("ACK"), r->f_header.f_dest_addr, r->f_header.f_src_addr));
            
            #ifdef ACCEPT_ONE_NODE
                if (mac_ctx.m_accept_nid) {
                    if (r->f_header.f_src_addr == mac_ctx.m_accept_nid) {
                        mac_handle_event(MAC_EVENT_RECV, r);      
                        listen = 0;                    
                    } else {
                        DBG_MAC(usart_printf(PSTR("MAC ignoring from %d\r\n"), r->f_header.f_src_addr));
                        listen = 1;
                    }
                } else {
                   DBG_MAC(usart_printf(PSTR("MAC accepting only from %d\r\n"), r->f_header.f_src_addr));
                   mac_ctx.m_accept_nid = r->f_header.f_src_addr;
                   mac_handle_event(MAC_EVENT_RECV, r);      
                   listen = 0;
                }
            #else 
               mac_handle_event(MAC_EVENT_RECV, r);      
               listen = 0;
            #endif
        } else {
            DBG_MAC(usart_printf(PSTR("MAC intercepted %S dst:%d src:%d\r\n"), r->f_header.f_flags == LINK_DTA?PSTR("DTA"):PSTR("ACK"), r->f_header.f_dest_addr, r->f_header.f_src_addr));
        }
	} else
        mac_ctx.m_debug.d_corrupted++;


    if (listen)
        mac_switch_mode(MAC_TCVR_LISTEN);
}

/*!
    \breif Transmit interrupt bottom half handler
 */
void mac_txc(ptr_t p)
{
    plink_frame_t fp = (plink_frame_t)p;
    LED_OFF(LED_RED);
    mac_ctx.m_debug.d_sent++;
    
    DBG_MAC(usart_printf(PSTR("MAC TXC %S fp:%p dst:%d src:%d\r\n"), fp->f_header.f_flags == LINK_DTA?PSTR("DTA"):PSTR("ACK"), fp, fp->f_header.f_dest_addr, fp->f_header.f_src_addr));
    
    if (fp->f_header.f_flags != LINK_ACK && fp->f_header.f_dest_addr == MAC_BROADCAST_NID) {
        event_activate(EVENT_MAC_TXC, fp);
    }
    
    mac_handle_event(MAC_EVENT_TXC, fp);
}

/*!
	\brief Switch device between RX, TX and PD mode.
 */
static uint8_t mac_switch_mode(mac_tcvr_state_t m_tcvr_state)
{
    uint8_t result = 0;

    LED_OFF(LED_GREEN);
    LED_OFF(LED_RED);

    if (mac_ctx.m_tcvr_state == m_tcvr_state)
        return 1;

	switch (m_tcvr_state) {
        case MAC_TCVR_TX:
            if (mac_ctx.m_tcvr_state == MAC_TCVR_RX) {
                DBG_MAC(usart_printf(PSTR("MAC unable to transmit, reception in progress\r\n")));
                return 0;
            }

            CC1000_DCLK_DISABLE;
			// set DIO port as output
			CC1000_DIO_OUTPUT;
			// enable interrupt on falling edge
			CC1000_DCLK_FALLING;

			if ((result = cc1000_switch_mode(CC1000_TX))) {
                mac_ctx.m_counter = MAC_PREAMBLE_TX / 8;
				mac_ctx.m_tcvr_state = MAC_TCVR_TX;
                mac_ctx.m_trx_state = MAC_TRX_PREAMBLE;
                // enable interrupt
                CC1000_DCLK_ENABLE;
                // debug
                DBG_MAC(usart_printf(PSTR("MAC switch tx\r\n")));
            }
	            
		break;
		
        case MAC_TCVR_LISTEN:
            if (mac_ctx.m_tcvr_state == MAC_TCVR_RX && mac_ctx.m_trx_state != MAC_TRX_DONE) {
                DBG_MAC(usart_printf(PSTR("MAC unable to listen, frame present in buffer\r\n")));
                return 0;
            }

            // disable interrupt
            CC1000_DCLK_DISABLE;
			// set DIO port as input
			CC1000_DIO_HIGH;
			CC1000_DIO_INPUT;

			// enable interrupt on rising edge
			CC1000_DCLK_RISING;

			cc1000_switch_mode(CC1000_RX);

            mac_reset();            
            mac_ctx.m_tcvr_state = MAC_TCVR_LISTEN;

            CC1000_DCLK_ENABLE;
            result++;
        break;

        case MAC_TCVR_RX:
            mac_ctx.m_tcvr_state = MAC_TCVR_RX;
            result++;
		break;

        case MAC_TCVR_LOCK:
            if ((mac_ctx.m_tcvr_state == MAC_TCVR_TX || mac_ctx.m_tcvr_state == MAC_TCVR_RX) && mac_ctx.m_trx_state != MAC_TRX_DONE) {
                DBG_MAC(usart_printf(PSTR("MAC unable to lock, buffer is engaged\r\n")));                
                return 0;
            }
            CC1000_DCLK_DISABLE;
            
            // we need to turn on the receiver in order to sample rssi
			cc1000_switch_mode(CC1000_RX);
                        
            mac_ctx.m_tcvr_state = MAC_TCVR_LOCK;
            result++;

            // debug
            DBG_MAC(usart_printf(PSTR("MAC switch lock\r\n")));
        break;

		case MAC_TCVR_PD:
            // switch cc1000 to power down mode
            cc1000_sleep();
            // set mode
			mac_ctx.m_tcvr_state = MAC_TCVR_PD;
            result++;
            // debug
            DBG_MAC(usart_printf(PSTR("MAC switch pd\r\n")));
		break;
    }
    
    return result;
}

/*!
    Send frame.

    Function makes a copy of given frame to internal buffer and switches
    the transceiver state to the TX mode.

    \param f pointer to frame to send
    \return positive value when transceiver has been successfully switched to TX mode, 0 otherwise
 */
uint8_t mac_send_frame(plink_frame_t f)
{
    uint8_t *bp = mac_ctx.m_buffer;
    int16_t rssi = 0;

    if (!f || link_frame_size(f) > MAC_MTU)
        return 0;

    // block the buffer
    if (!mac_switch_mode(MAC_TCVR_LOCK))
        return 0;

    if (f->f_header.f_flags == LINK_DTA) {
        // sense carrier only for data frames
        rssi_measure();
        rssi = rssi_get_value();
        DBG_MAC(usart_printf(PSTR("MAC rssi:%d dBm\r\n"), rssi));

        // if carrier detected then stop
        if (rssi > MAC_CARRIER_LVL) {
            DBG_MAC(usart_printf(PSTR("MAC carrier present, stop\r\n")));
            return 0;
        }
    }

    // copy frame header
    memcpy(bp, &f->f_header, sizeof(link_frame_header_t));
    bp += sizeof(link_frame_header_t);

    // copy frame load
    memcpy(bp, f->f_load, f->f_header.f_size);
    bp += f->f_header.f_size;

    // copy frame crc
    memcpy(bp, &f->f_crc, 2);
    bp += 2;

    // amount of bytes to send
    mac_ctx.m_bp = bp - mac_ctx.m_buffer;

    DBG_MAC(usart_printf(PSTR("MAC send %S f:%p dst:%d src:%d\r\n"), f->f_header.f_flags == LINK_DTA?PSTR("DTA"):PSTR("ACK"), f, f->f_header.f_dest_addr, f->f_header.f_src_addr));

    // switch to tx mode
	return mac_switch_mode(MAC_TCVR_TX);
}

/*!
    \brief Send control frame

    Function prepares a control frame and sends it immediately.
 */
uint8_t mac_send_ack(plink_frame_t fack)
{
    uint8_t i, *dp;
    link_frame_t frame;
    plink_frame_t fp = &frame;
    
    fp->f_header.f_flags = LINK_ACK;
    fp->f_header.f_seq = fack->f_header.f_seq;
    fp->f_header.f_src_addr = mac_ctx.m_nid;
    fp->f_header.f_dest_addr = fack->f_header.f_src_addr;
    fp->f_header.f_size = 0;
    fp->f_crc = 0;

    // copy frame header
    for (i = 0, dp = (uint8_t *)&fp->f_header; i < sizeof(link_frame_header_t); i++, dp++) {
        fp->f_crc = _crc_xmodem_update(fp->f_crc, *dp);
    }
    
    return mac_send_frame(fp);
}

/*!
	\brief Interrupt handler for RF interface.
 */
void mac_irq_handler()
{
	switch (mac_ctx.m_tcvr_state) {
		case MAC_TCVR_TX:
			mac_send_bit();
			break;
		case MAC_TCVR_RX: 
        case MAC_TCVR_LISTEN:
			mac_recv_bit();		
			break;
        default:
            break;
  	}
}

/*!
    \brief Search for frame in received frames history table.

    Function looks for a lately received frame with the same source address and
    sequence number as given frame. If such a frame is present in the table 
    function exits with positive value result, otherwise the frame is inserted
    to history table on last position shifting out oldest entry.

    \param Frame which should be looked up.
    \return 1 when similar frame has been found, 0 otherwise
 */
uint8_t mac_search_history(plink_frame_t fp)
{
    uint8_t i, result = 0;

    assert(fp != 0);

    // DIRTY HACK IN ORDER TO SKIP FIRST FRAME SEND AFTER RESET
    if (fp->f_header.f_seq == 0 || fp->f_header.f_seq == 1)
        return result;

    for (i = 0; i < MAC_HAS && !result; i++) {
        if (mac_ctx.m_history[i].h_addr == fp->f_header.f_src_addr && 
            mac_ctx.m_history[i].h_seq == fp->f_header.f_seq) {
            result = 1;
        }
    }

    if (!result) {
        for (i = 1; i < MAC_HAS; i++) {
            mac_ctx.m_history[i - 1] = mac_ctx.m_history[i];
        }
        mac_ctx.m_history[MAC_HAS - 1].h_addr = fp->f_header.f_src_addr;
        mac_ctx.m_history[MAC_HAS - 1].h_seq = fp->f_header.f_seq;
        
    }

    return result;
}

uint8_t mac_handle_received(plink_frame_t fp)
{
    bool listen = true;

    if (fp && fp->f_header.f_flags == LINK_DTA) {
        bool duplicate = false;

        #ifdef MAC_DISCARD_DUPLICATE
        duplicate = mac_search_history(fp) ? true : false;
        #endif

        // pass to link layer only if the frame is not a duplicate                
        if (!duplicate) {
            // stop processing if link layer rejected frame
            if (link_rxc(fp) && fp->f_header.f_dest_addr != MAC_BROADCAST_NID) {
                if (mac_send_ack(fp)) {
                    listen = false;
                } else {
                    mac_ctx.m_debug.d_ack_fail++;
                }
            }        
        } else {
            DBG_MAC(usart_printf(PSTR("MAC drop, duplicated seq:%d\r\n"), fp->f_header.f_seq));
        }
    }

    if (listen) {
        mac_switch_mode(MAC_TCVR_LISTEN);  
    }
    return 1;
}

void mac_backoff(void) {
    mac_ctx.m_state = MAC_DEFER;
    mac_ctx.m_retries += 1;
    mac_ctx.m_backoff_timeout = MAC_DTA_MIN_DEFER + rand() % (MAC_DTA_DEFER + 1);
    timer_set(mac_ctx.m_timer, mac_ctx.m_backoff_timeout, _BV(TIMER_ENABLED), mac_ctx.m_tx_fp);
    DBG_MAC(usart_printf(PSTR("MAC defering for %d\r\n"), mac_ctx.m_backoff_timeout));
}

/*!
    Handle event when MAC layer is in idle state.

    \param event
    \param fp

    \return
 */
uint8_t mac_idle(mac_event_t event, plink_frame_t fp)
{
    uint8_t result = 1;

    DBG_MAC(usart_printf(PSTR("MAC idle %S fp:%p\r\n"), mac_event_string[event], fp));
    switch (event) {
        case MAC_EVENT_TXC:
            mac_switch_mode(MAC_TCVR_LISTEN);
            break;

        case MAC_EVENT_RECV:
            mac_handle_received(fp);
            break;

        default:
            break;
    }

    DBG_MAC(usart_printf(PSTR("MAC idle: result %d next %S\r\n"), result, mac_state_string[mac_ctx.m_state]));
    return result;
}

/*!
    Handle event when MAC sublayer waits for acknowledgment from adjacent node.

    \param event
    \param fp

    \return 
 */
uint8_t mac_wfack(mac_event_t event, plink_frame_t fp)
{
    uint8_t result = 1;
    DBG_MAC(usart_printf(PSTR("MAC wfack %S fp:%p\r\n"), mac_event_string[event], fp));
    
    switch (event) {
        case MAC_EVENT_TXC:
            if (fp && fp->f_header.f_flags == LINK_DTA) {
                timer_set(mac_ctx.m_timer, MAC_ACK_TIMEOUT, _BV(TIMER_ENABLED), mac_ctx.m_tx_fp); 
            }
            break;

        case MAC_EVENT_RECV: {
            // reject all frames except ACK
            if (fp && fp->f_header.f_flags == LINK_ACK) {
                // reject frame if the sequence numbers do not match
                if (fp->f_header.f_seq != mac_ctx.m_tx_fp->f_header.f_seq)
                    break;

                event_activate(EVENT_MAC_TXC, mac_ctx.m_tx_fp);
                
                // disable timer and go to idle state
                timer_disable(mac_ctx.m_timer);
                mac_ctx.m_state = MAC_IDLE;
            }
            break;
        }

        case MAC_EVENT_TIMEOUT: {
            if (mac_ctx.m_retries < MAC_DTA_RETRIES) {
                mac_backoff();
            } else {
                //debug
                DBG_MAC(usart_printf(PSTR("MAC no response from %d %d\r\n"), mac_ctx.m_tx_fp->f_header.f_dest_addr));
                mac_ctx.m_debug.d_lost++;
                mac_ctx.m_state = MAC_IDLE;
                event_activate(EVENT_MAC_FAIL, mac_ctx.m_tx_fp);
            }            
            break;
        }

        default:
            break;
    }

    mac_switch_mode(MAC_TCVR_LISTEN);
    
    DBG_MAC(usart_printf(PSTR("MAC wfack result:%d next %S\r\n"), result, mac_state_string[mac_ctx.m_state]));
    return result;
}

/*!
    Handle event when MAC sublayer is deffering activities

    \param event
    \param fp
 */
uint8_t mac_defer(mac_event_t event, plink_frame_t fp)
{
    uint8_t result = 1;

    DBG_MAC(usart_printf(PSTR("MAC defer %S fp:%p\r\n"), mac_event_string[event], fp));

    switch (event) {
        case MAC_EVENT_TIMEOUT: {
            if (mac_send_frame(mac_ctx.m_tx_fp)) {
                if (fp->f_header.f_flags == LINK_DTA && fp->f_header.f_dest_addr != MAC_BROADCAST_NID) {
                    mac_ctx.m_state = MAC_WFACK;
                } else {
                    mac_ctx.m_state = MAC_IDLE;
                }
            } else {
                if (mac_ctx.m_retries < MAC_DTA_RETRIES) {
                    mac_backoff();
                } else {
                    //debug
                    DBG_MAC(usart_printf(PSTR("MAC unable to send, won't try any more\r\n")));
                    event_activate(EVENT_MAC_FAIL, mac_ctx.m_tx_fp);
                    mac_ctx.m_state = MAC_IDLE;                        
                }
                mac_switch_mode(MAC_TCVR_LISTEN);
            }
            break;
        }

        case MAC_EVENT_RECV: {
            mac_handle_received(fp);
            break;
        }

        case MAC_EVENT_TXC: {
            mac_switch_mode(MAC_TCVR_LISTEN);
            break;
        }

        default:
            break;
    }
        

    DBG_MAC(usart_printf(PSTR("MAC defer: result:%d next %S\r\n"), result, mac_state_string[mac_ctx.m_state]));
    return result;
}


/**
    \brief MAC layer timeout handler.

    This handler is invoked when a timer set by MAC state handler expires.
 */
void mac_timeout(uint8_t t_id)
{
    mac_handle_event(MAC_EVENT_TIMEOUT, (plink_frame_t)timer_get_pdata(t_id));    
} 

/**
    \brief Switch MAC states.
 */
uint8_t mac_handle_event(mac_event_t event, plink_frame_t fp)
{
    uint8_t result = 0;
    mac_state_handler_t handler = mac_state_handlers[mac_ctx.m_state];

    if (handler)
        result = handler(event, fp);

    return result;
}

/*!
    \brief Get MAC context debug info
 */
pmac_debug_t mac_debug_info()
{
    return &mac_ctx.m_debug;
}

uint8_t mac_send(plink_frame_t fp)
{
    if (mac_ctx.m_state == MAC_IDLE) {
        if (fp)  {
            mac_ctx.m_tx_fp = fp;
            mac_ctx.m_retries = 0;
            mac_backoff();
            /*
            // if it isn't possible to send immediately, then schedule and try later
            if (mac_send_frame(mac_ctx.m_tx_fp)) {
                // if not broadcast then wait for ACK
                if (mac_ctx.m_tx_fp->f_header.f_dest_addr != MAC_BROADCAST_NID) {
                    mac_ctx.m_state = MAC_WFACK;
                }
            } else {
                
                //mac_switch_mode(MAC_TCVR_LISTEN);   
            }
            */
        }

    }
    return 0;
}

/*!
    \brief Initialize MAC sublayer

    Function initializes underlaying transceiver and switches
    to LISTEN mode.
 */
void mac_init(pcfg_t cfg)
{
    uint8_t i;

    cc1000_init();
    mac_ctx.m_nid = cfg->cfg_nid_addr;
    #ifdef ACCEPT_ONE_NODE
    mac_ctx.m_accept_nid = 0;
    #endif

    for (i = 0; i < MAC_HAS; i++) {
        mac_ctx.m_history[i].h_addr = 0;
        mac_ctx.m_history[i].h_seq = 0;
    }
    
    rssi_measure();
    srandom(mac_ctx.m_nid);
    mac_ctx.m_timer = timer_add(mac_timeout);

    event_register(EVENT_RADIO_RXC, mac_rxc, true);
    event_register(EVENT_RADIO_TXC, mac_txc, true);
            
    mac_switch_mode(MAC_TCVR_LISTEN);
}
