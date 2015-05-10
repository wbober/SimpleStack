#include <avr/io.h>
#include <util/crc16.h>	
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "event.h"
#include "link.h"
#include "defines.h"
#include "list.h"
#include "mac.h"
#ifdef BASE
#include "../Base/usart.h"
#endif

/*!
    Link layer contex structure.
 */
static struct {
    uint8_t l_nid;
    uint16_t l_seq;             //<** sequence counter >
    plist_t l_rx_queue;
    plist_t l_tx_queue;
    
    link_frame_t l_frame;
    
    struct {
        uint8_t d_frames;
        uint16_t d_discarded;
    } l_debug;

} link_ctx;


/*!
    Allocate frame
 */
plink_frame_t link_frame_alloc(link_frame_type_t type)
{
    plink_frame_t f = NULL;
    size_t size = sizeof(link_frame_t);

    // determine frame size
    if (type != LINK_DTA) {
        size -= LINK_FRAME_LOAD_SIZE;
    } 

    f = (plink_frame_t) malloc(size);
    if (!f) {
        DBG_LINK(usart_printf(PSTR("LNK frame alloc failed %p\r\n"), f));
        return NULL;
    }

    memset(f, 0xBB, size);

    link_ctx.l_debug.d_frames++;

    f->f_header.f_flags = 0;
    //f->f_header.f_size = size;   
    f->f_crc = 0;

    DBG_LINK(usart_printf(PSTR("LNK alloc %S %p\r\n"), (type == LINK_ACK)?PSTR("ACK"):PSTR("DTA"), (void *)f));
    
    return f;
}

/*!
    Copy frame
*/
plink_frame_t link_frame_copy(plink_frame_t dest, plink_frame_t src)
{
    if (!(dest && src))
        return NULL;

    if (src->f_header.f_size < LINK_FRAME_LOAD_SIZE) {
        memcpy(dest, src, sizeof(link_frame_header_t) + src->f_header.f_size + sizeof(src->f_crc));
        return dest; 
    } else
        return NULL;
}

/*!
    Free frame
 */
plink_frame_t link_frame_free(plink_frame_t *f)
{
    if (*f) {
        DBG_LINK(usart_printf(PSTR("LNK frame free %p\r\n"), (void *)*f));
        memset(*f, 0xCC, sizeof(link_frame_t));
        free(*f);
        f = NULL;
        link_ctx.l_debug.d_frames--;
    }

    return NULL;
}

/*!
    Handle incoming frame

    Function insterts incoming frame into incoming queue and informs
    network layer that there is a new frame waiting in the queue.

    \param fp
 */
uint8_t link_rxc(plink_frame_t fp)
{
    plink_frame_t f = link_frame_alloc(LINK_DTA);
      
    DBG_LINK(usart_printf(PSTR("LNK RXC fp:%p src:%d\r\n"), fp, fp->f_header.f_src_addr));

    if (!f)
        return 0;

    link_frame_copy(f, fp);
    
    if (!list_push(link_ctx.l_rx_queue, f)) {
        link_frame_free(&f);
        return 0;
    };

    event_activate(EVENT_LINK_RXC, NULL);

    return 1;
}

/*!
    Handle request for outgoing frame

    Function passes fist frame from outgoing queue
    to MAC sublayer.

    \param fp

 */
uint8_t link_txc(ptr_t p)
{
    plink_frame_t f = list_pop(link_ctx.l_tx_queue);

    DBG_LINK(
        plink_frame_t fp = (plink_frame_t)p; 
        usart_printf(PSTR("LNK TXC fp:%p dest:%d\r\n"), fp, fp->f_header.f_dest_addr)
    ); 
    
    //if (fp->f_header.f_dest_addr != MAC_BROADCAST_NID)
    event_activate(EVENT_LINK_TXC, (ptr_t)&link_ctx.l_frame);

    link_frame_copy(&(link_ctx.l_frame), f);
    link_frame_free(&f);

    f = list_head(link_ctx.l_tx_queue);
    

    return 1;
}

/*!
    Handle transmission failure

    This function is invoked by MAC sublayer when transmission
    of given frame to adjacent node was unsucessfull.

    \param fp
 */
uint8_t link_fail(ptr_t p)
{
    plink_frame_t fp = (plink_frame_t)p;
    plink_frame_t f = list_pop(link_ctx.l_tx_queue);

    DBG_LINK(
        usart_printf(PSTR("LNK fail fp:%p dest:%d\r\n"), fp, fp->f_header.f_dest_addr)
    );

    
    if (fp->f_header.f_dest_addr != MAC_BROADCAST_NID)
        event_activate(EVENT_LINK_FAIL, (ptr_t)&link_ctx.l_frame);
    
    link_frame_copy(&(link_ctx.l_frame), f);
    link_frame_free(&f);

    // try to send next frame from queue
    f = list_head(link_ctx.l_tx_queue);
    mac_send(f);

    return 1;
}

/*!
    Get frame from incoming queue
 */
uint8_t link_receive(uint8_t *addr, uint8_t *data, uint8_t size)
{
    uint8_t result = 0;
    plink_frame_t f = list_pop(link_ctx.l_rx_queue);

    DBG_LINK(usart_printf(PSTR("LNK receive f:%p src:%d\r\n"), f, f->f_header.f_src_addr));
    
    // copy frame load if sufficient space in the buffer
    if (size >= f->f_header.f_size) {
        *addr = f->f_header.f_src_addr;    
        memcpy(data, f->f_load, f->f_header.f_size);
        result = 1;
    }

    link_frame_free(&f);

    // if there is any frame left in the rx buffer, activate and event to process it
    if (link_ctx.l_rx_queue->l_count > 0)
        event_activate(EVENT_LINK_RXC, NULL);

    return result;
}

/*!
	Insert frame to outgoing queue

    Function creates a link frame and inserts it into outcoming buffer.

    \param addr adjacent node address
	\param data pointer to data
	\param size size of data
 */
uint16_t link_send(uint8_t addr, uint8_t *data, uint8_t size)
{	
	uint8_t i = 0, *dp;
    plink_frame_t f;

    if (!(f = link_frame_alloc(LINK_DTA)))
        return 0;    

    // set header fields
    f->f_header.f_seq = link_ctx.l_seq;
    f->f_header.f_flags = LINK_DTA;
    f->f_header.f_dest_addr = addr;
    f->f_header.f_src_addr = link_ctx.l_nid;
    f->f_header.f_size = size;
    f->f_crc = 0;

    // calculate header's crc
    for (i = 0, dp = (uint8_t *)&f->f_header; i < sizeof(link_frame_header_t); i++, dp++)
        f->f_crc = _crc_xmodem_update(f->f_crc, *dp);

	// copy bytes and calculate crc
    for (i = 0, dp = data; i < size; i++, dp++) {
 		f->f_load[i] = *dp;
		f->f_crc = _crc_xmodem_update(f->f_crc, *dp);
	}

    if (!list_push(link_ctx.l_tx_queue, f)) {
        link_frame_free(&f);
        return 0;
    }

    DBG_LINK(usart_printf(PSTR("LNK send dest:%d\r\n"), f->f_header.f_dest_addr));
    mac_send(f);

	return ++link_ctx.l_seq;
}

/*!
	\brief Initialize link layer.
 */
void link_init(pcfg_t cfg)
{
    mac_init(cfg);

    link_ctx.l_nid = cfg->cfg_nid_addr;
    link_ctx.l_seq = 1;
    link_ctx.l_rx_queue = list_create(LINK_RX_QUEUE_SIZE);
    link_ctx.l_tx_queue = list_create(LINK_TX_QUEUE_SIZE);
   
    event_register(EVENT_MAC_TXC, link_txc, false);
    event_register(EVENT_MAC_FAIL, link_fail, false);
}




