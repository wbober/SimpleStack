#include <string.h>
#include "network.h"
#include "link.h"
#include "list.h"
#include "timers.h"
#include "event.h"

#ifdef BASE
    #include "../Base/usart.h"
#endif

/*!
    Network layer context.
 */
static struct {
    uint8_t n_nid;
    uint8_t n_flags;
    net_packet_t n_current;

    #ifdef NET_ROUTING
        uint16_t n_seq;
        uint16_t n_req_id;

        uint8_t n_retries;
    	uint8_t n_timeout;
        uint8_t n_clock;

        plist_t n_in_queue;
        plist_t n_out_queue;

        net_route_entry_t n_route[NET_ROUTE_TABLE_SIZE];
    #endif
    
    struct {
        uint8_t d_packet_count;
    } n_debug;
} net_ctx;


void net_link_rxc(ptr_t);
void net_link_txc(ptr_t);
void net_link_fail(ptr_t);



/*!
    Allocate frame
 */
#ifdef NET_ROUTING
pnet_packet_t net_packet_alloc()
{
    pnet_packet_t p;

    p = (pnet_packet_t) malloc(sizeof(net_packet_t));
    if (!p) {
        DBG_NET(usart_printf(PSTR("NET packet alloc failed %p\r\n"), p));
        return NULL;
    }

    memset((void *)p, 0xAA, sizeof(net_packet_t));
    net_ctx.n_debug.d_packet_count++;

    DBG_NET(usart_printf(PSTR("NET frame alloc %p\r\n"), (void *)p));
    
    return p;
}
#endif


/*!
    Free frame
 */
#ifdef NET_ROUTING
pnet_packet_t net_packet_free(pnet_packet_t *f)
{
    if (*f) {
        DBG_NET(usart_printf(PSTR("NET frame free %p\r\n"), (void *)*f));
        memset(*f, 0xFF, sizeof(net_packet_t));
        free(*f);
        f = NULL;
        net_ctx.n_debug.d_packet_count--;
    }

    return NULL;
}
#endif

/*!
    Copy frame
*/
pnet_packet_t net_packet_copy(pnet_packet_t dest, pnet_packet_t src)
{
    if (!(dest && src))
        return NULL;

    if (src->p_size < NET_PACKET_LOAD_SIZE) {
        memcpy(dest, src, sizeof(net_packet_t) - NET_PACKET_LOAD_SIZE + src->p_size);
        return dest; 
    } else
        return NULL;
}

/*!
 */
#ifdef NET_ROUTING
uint16_t net_route(uint8_t addr, pnet_packet_t p)
{
    pnet_route_entry_t re = &net_ctx.n_route[addr];
    pnet_route_entry_t me = &net_ctx.n_route[net_ctx.n_nid];
    uint16_t result = 0;

    DBG_NET(usart_printf(PSTR("NET route p:%p src:%d dst:%d\r\n"), p, p->p_src_addr, p->p_dest_addr));
    
    if (p) {
        // check if the route is valid
        if (re->r_flags & _BV(NET_ROUTE_VALID)) {
            DBG_NET(usart_printf(PSTR("NET next hop:%d\r\n"), re->r_next_hop));
            // send packet to next hop
            result = link_send(re->r_next_hop, (uint8_t *)p, net_packet_size(p));
        } else {
            // send rreq if queue has only one element
            net_packet_t pp;
            pnet_rreq_t rreq = (pnet_rreq_t)pp.p_load;
            pp.p_src_addr = net_ctx.n_nid;
            pp.p_dest_addr = 0;
            pp.p_type = NET_RREQ;
            pp.p_size = sizeof(net_rreq_t);
            rreq->r_flags = 0;
            rreq->r_dest_addr = addr;
            rreq->r_dest_seq = re->r_seq;
            rreq->r_org_addr = net_ctx.n_nid;
            rreq->r_org_seq = ++net_ctx.n_seq;
            rreq->r_req_id = ++net_ctx.n_req_id;
            if (re->r_flags & _BV(NET_ROUTE_UNKNOWN_SEQ)) {
                rreq->r_flags |= _BV(NET_RREQ_UNKNOWN_SEQ);
            }

            me->r_req_id = net_ctx.n_req_id;
            me->r_req_ttl = NET_ROUTE_REQ_TTL;

            DBG_NET(usart_printf(PSTR("NET no route, send rreq\r\n")));
            // bcast rreq
            result = link_send(0, (uint8_t *)&pp, net_packet_size(&pp));
            if (result) {
                timer_enable(net_ctx.n_timeout);
            }
        }      
    }
    return result; 
}
#endif

/*!
 */
#ifdef NET_ROUTING
void net_timeout(uint8_t t)
{
    // remove packet from the outgoing queue
    pnet_packet_t pp = list_pop(net_ctx.n_out_queue);
    DBG_NET(usart_printf(PSTR("NET timeout p:%p src:%d dst:%d\r\n"), pp, pp->p_src_addr, pp->p_dest_addr));
    
    net_packet_free(&pp);
    
    // if there is anything in the queue, try to send it
    if (net_ctx.n_out_queue->l_count > 0) {
        pp = list_head(net_ctx.n_out_queue);
        net_route(pp->p_dest_addr, pp);
    }

    // report failure   
    event_activate(EVENT_NET_FAIL, NULL);
}
#endif

/*!
 */
#ifdef NET_ROUTING
void net_handle_rrep(uint8_t hop, pnet_packet_t p)
{
    pnet_rrep_t rrep = (pnet_rrep_t)p->p_load;  
    bool update = false;    

    // get route to the previous hop
    pnet_route_entry_t hop_re = &net_ctx.n_route[hop];
    // get route to the destination
    pnet_route_entry_t dst_re = &net_ctx.n_route[rrep->r_dest_addr];


    DBG_NET(usart_printf(PSTR("NET RREP p:%p org:%d\r\n"), p, rrep->r_org_addr));

    // create route toward previous hop if desired
    if (!(hop_re->r_flags & _BV(NET_ROUTE_VALID))) {
        hop_re->r_flags |= _BV(NET_ROUTE_VALID) | _BV(NET_ROUTE_UNKNOWN_SEQ);
        hop_re->r_next_hop = hop;
    }

    // increment hop count
    NET_INC_HOP_CNT(rrep);
     
    // update route if invalid or newer
    if (!(dst_re->r_flags & _BV(NET_ROUTE_VALID)) || (dst_re->r_flags & _BV(NET_ROUTE_UNKNOWN_SEQ))) {
        update = true;
    } else if ((int16_t)rrep->r_dest_seq > (int16_t)dst_re->r_seq) {
        update = true;
    } else if (NET_HOP_CNT(rrep) < NET_HOP_CNT(dst_re)) {
        update = true;
    }

    // update forward route
    if (update) {
        // debug
        DBG_NET(usart_printf(PSTR("NET RREP updating route to %d via %d \r\n"), rrep->r_dest_addr, hop));
        // mark route as valid
        dst_re->r_flags = _BV(NET_ROUTE_VALID);
        // set the hop count
        dst_re->r_flags |= rrep->r_flags;
        // set the next hop
        dst_re->r_next_hop = hop;
        // set the dest seq number
        dst_re->r_seq = rrep->r_dest_seq;
    }

    // if intermediate rrep node then forward rrep
    if (rrep->r_org_addr != net_ctx.n_nid) {
        // get route to the orignator 
        pnet_route_entry_t org_re = &net_ctx.n_route[rrep->r_org_addr];
        // debug
        DBG_NET(usart_printf(PSTR("NET forward RREP to %d by %d\r\n"), rrep->r_org_addr, org_re->r_next_hop));        
        // update precursor list for the destination
        dst_re->r_prev_list |= 1 << org_re->r_next_hop;
        // unicast rrep to originator
        link_send(org_re->r_next_hop, (uint8_t *)p, net_packet_size(p));
    } else {
        // disable timer
        timer_disable(net_ctx.n_timeout);
        // get packet from the queue
        pnet_packet_t pp = list_head(net_ctx.n_out_queue);
        if (pp) {
            // debug
            DBG_NET(usart_printf(PSTR("NET sending from queue p:%p\r\n"), pp));
            // send packet 
            net_route(pp->p_dest_addr, pp);
        } else {
            DBG_NET(usart_printf(PSTR("NET nothing to route, strange...\r\n")));        
        }
    }
}
#endif

/*!
    Handle route request.

    \praam hop previous hop, form which RREQ was received
    \param p packet with RREQ
 */
#ifdef NET_ROUTING
void net_handle_rreq(uint8_t hop, pnet_packet_t p)
{
    net_packet_t r;
    pnet_rreq_t rreq = (pnet_rreq_t)p->p_load;  
    pnet_rrep_t rrep = (pnet_rrep_t)r.p_load;

    pnet_route_entry_t hop_re = &net_ctx.n_route[hop];
    pnet_route_entry_t org_re = &net_ctx.n_route[rreq->r_org_addr];
    pnet_route_entry_t dst_re = &net_ctx.n_route[rreq->r_dest_addr]; 

    DBG_NET(usart_printf(PSTR("NET RREQ p:%p org:%d dst:%d dst_seq:%d req_id%d\r\n"), p, rreq->r_org_addr, rreq->r_dest_addr, rreq->r_dest_seq, rreq->r_req_id));

    // create route toward previous hop if desired
    if (!(hop_re->r_flags & _BV(NET_ROUTE_VALID))) {
        hop_re->r_flags |= _BV(NET_ROUTE_VALID) | _BV(NET_ROUTE_UNKNOWN_SEQ);
        hop_re->r_next_hop = hop;
    }

    // if the request was already processed
    if ((org_re->r_req_id == rreq->r_req_id && org_re->r_req_ttl > 0)) {
        DBG_NET(usart_printf(PSTR("NET drop, rreq_id already received:%d\r\n"), rreq->r_org_addr));
        return;
    }
    
    r.p_type = NET_RREP;
    r.p_size = sizeof(net_rrep_t);
    r.p_src_addr = net_ctx.n_nid;
    r.p_dest_addr = rreq->r_org_addr;

    // increase hop count
    NET_INC_HOP_CNT(rreq);

    // set reverse path
    if (!(hop_re->r_flags & _BV(NET_ROUTE_VALID)) || org_re->r_flags & _BV(NET_ROUTE_UNKNOWN_SEQ) || org_re->r_seq < rreq->r_org_seq) {
        org_re->r_seq = rreq->r_org_seq;
        org_re->r_flags = NET_HOP_CNT(org_re) | _BV(NET_ROUTE_VALID);
        org_re->r_next_hop = hop;
        DBG_NET(usart_printf(PSTR("NET reverse path via %d to %d set\r\n"), hop, rreq->r_org_addr));
    }
    org_re->r_req_id = rreq->r_req_id;
    org_re->r_req_ttl = NET_ROUTE_REQ_TTL;


    if (net_ctx.n_nid == rreq->r_dest_addr) {
        // debug
        DBG_NET(usart_printf(PSTR("NET i am dst, send rrep org:%d\r\n"), rreq->r_org_addr));
        // update own sequence number to max of own and rreq seq number
        if (rreq->r_dest_seq > net_ctx.n_seq) {
            net_ctx.n_seq = rreq->r_dest_seq;
            DBG_NET(usart_printf(PSTR("NET update seq:%d\r\n"), net_ctx.n_seq));
        }
        // set the dest seq
        rrep->r_dest_seq = net_ctx.n_seq;
        // copy the dest addr
        rrep->r_dest_addr = rreq->r_dest_addr;
        // set hop count to 0
        rrep->r_flags = 0;
        // set the originator addres
        rrep->r_org_addr = rreq->r_org_addr;
        // unicast to originator
        link_send(org_re->r_next_hop, (uint8_t *)&r, net_packet_size(&r));
    }  
    else if (dst_re->r_flags & _BV(NET_ROUTE_VALID) && (int16_t)dst_re->r_seq >= (int16_t)rreq->r_dest_seq) {           
        // debug
        DBG_NET(usart_printf(PSTR("NET i have route, send rrep org:%d\r\n"), rreq->r_org_addr));
        // copy the known dest seq no
        rrep->r_dest_seq = dst_re->r_seq;
        // copy the dest addr
        rrep->r_dest_addr = rreq->r_dest_addr;
        // set the hop count from the destination
        rrep->r_flags |= NET_HOP_CNT(dst_re);
        // set the originator addres
        rrep->r_org_addr = rreq->r_org_addr;
        // set the prcursor for the forward route
        dst_re->r_prev_list |= 1 << hop;
        // set the precursor for the reverse route
        org_re->r_prev_list |= 1 << dst_re->r_next_hop;
        // unicast to originator
        link_send(org_re->r_next_hop, (uint8_t *)&r, net_packet_size(&r));
    } else {
        // debug
        DBG_NET(usart_printf(PSTR("NET no route, bcast rreq org:%d\r\n"), rreq->r_org_addr));
        // else broadcast rreq
        link_send(0, (uint8_t *)p, net_packet_size(p));
    }
}
#endif

/*!
 */
#ifdef NET_ROUTING
void net_handle_dta(uint8_t hop, pnet_packet_t p)
{
    // allocate memory for packet
    pnet_packet_t pp = net_packet_alloc();

    if (pp) {            
        // copy packet
        net_packet_copy(pp, p);
        // print debug info
        DBG_NET(usart_printf(PSTR("NET DTA org:%d\r\n"), pp->p_src_addr));
       
        if (p->p_dest_addr == net_ctx.n_nid) {
            if (!NET_IS_ROUTE_VALID(hop)) {
                pnet_route_entry_t hop_re = &net_ctx.n_route[hop];
                // invalidate route toward previous hop
                hop_re->r_flags |= 0x10 | _BV(NET_ROUTE_VALID) | _BV(NET_ROUTE_UNKNOWN_SEQ);
                hop_re->r_next_hop = hop;
                // debug
                DBG_NET(usart_printf(PSTR("NET DTA route towards hop %d updated \r\n"), hop));
            }

            if (!NET_IS_ROUTE_VALID(pp->p_src_addr)) {
                pnet_route_entry_t org_re = &net_ctx.n_route[pp->p_src_addr];
                // invalidate route toward src
                org_re->r_flags |= _BV(NET_ROUTE_VALID) | _BV(NET_ROUTE_UNKNOWN_SEQ);
                org_re->r_next_hop = hop;
                // debug
                DBG_NET(usart_printf(PSTR("NET DTA route towards src %d updated \r\n"), pp->p_src_addr));
            }    

            // insert to input queue
            list_push(net_ctx.n_in_queue, pp);       
            // rise event
            event_activate(EVENT_NET_RXC, NULL);
        } else {
            // push to outgoing queue
            list_push(net_ctx.n_out_queue, pp);

            // if no packets waiting in out queue, send it
            if (net_ctx.n_out_queue->l_count == 1) {
                if (!net_route(pp->p_dest_addr, pp)) {
                    list_pop(net_ctx.n_out_queue);
                    net_packet_free(&pp);                
                }
            }
        }
    }
}
#endif

/*!
 */
#ifdef NET_ROUTING
void net_clock(uint8_t tid)
{
    uint8_t i;

    // decrease ttl's in route table
    for (i = 0; i < NET_ROUTE_TABLE_SIZE; i++) {
        if (net_ctx.n_route[i].r_req_ttl > 0) {
            net_ctx.n_route[i].r_req_ttl--;
            #if DEBUG_NET
            if (net_ctx.n_route[i].r_req_ttl == 0) {
                usart_printf(PSTR("NET rreq_id expired dst:%d\r\n"), i);
            }
            #endif
        }
    }
}
#endif

/*!
    Handle recieve event from link layer.
 */
void net_link_rxc(ptr_t p)
{
    uint8_t hop, result;
    net_packet_t pp;

    result = link_receive(&hop, (uint8_t *)&pp, sizeof(net_packet_t));

    DBG_NET(usart_printf(PSTR("NET RXC p:%p\r\n"), pp)); 

    #ifdef NET_ROUTING
        switch (pp.p_type) {
            case NET_RREQ:
                net_handle_rreq(hop, &pp);
            break;
            case NET_RREP:
                net_handle_rrep(hop, &pp);
            break;
            case NET_DTA:
                net_handle_dta(hop, &pp);
            break;
            case NET_RERR:
            break;
        }
    #else   
        if (result) {
            net_packet_copy(&net_ctx.n_current, &pp);
            event_activate(EVENT_NET_RXC, NULL);
        }
    #endif
}

/*!
    Handle fail event from link layer.
 */
void net_link_fail(ptr_t p)
{
#ifdef NET_ROUTING    
    plink_frame_t fp = (plink_frame_t)p;
    pnet_packet_t pp = (pnet_packet_t)fp->f_load;

    DBG_NET(usart_printf(PSTR("NET fail p:%p src:%d dst:%d\r\n"), pp, pp->p_src_addr, pp->p_dest_addr));


        // invalidate route toward next hop
        net_ctx.n_route[fp->f_header.f_dest_addr].r_flags &= ~_BV(NET_ROUTE_VALID);
        // invalidate route toward dest
        net_ctx.n_route[pp->p_dest_addr].r_flags &= ~_BV(NET_ROUTE_VALID);    

        if (pp->p_type == NET_DTA) {
            // try to find other way
            if (net_ctx.n_retries++ < NET_ROUTE_RETRIES) {
                // debug
                DBG_NET(usart_printf(PSTR("NET find other route, ret: %d\r\n"), net_ctx.n_retries));
                pp = list_head(net_ctx.n_out_queue);
                net_route(pp->p_dest_addr, pp);
            } else {
                // debug
                DBG_NET(usart_printf(PSTR("NET no other route\r\n")));
                net_ctx.n_retries = 0;
                // remove packet from the outgoing queue
                pp = list_pop(net_ctx.n_out_queue);
                net_packet_free(&pp);
                // report failure   
                event_activate(EVENT_NET_FAIL, p);
            }
        }
    #else
        event_activate(EVENT_NET_FAIL, p);
    #endif

}

/*!
    Handle successfully sent frame.
    
 */
void net_link_txc(ptr_t p)
{
    #ifdef NET_ROUTING
        if (((plink_frame_t)p)->f_header.f_dest_addr != NET_BROADCAST_NID) {
            pnet_packet_t pp = list_pop(net_ctx.n_out_queue);
            net_packet_free(&pp);

            DBG_NET(usart_printf(PSTR("NET TXC p:%p\r\n"), pp));
    
            // reset number of retries
            net_ctx.n_retries = 0;

            if (net_ctx.n_out_queue->l_count > 0) {
                pp = list_head(net_ctx.n_out_queue);
                net_route(pp->p_dest_addr, pp);
            }
        }
    #endif

    event_activate(EVENT_NET_TXC, p);
}



/*!
	Asynchronous send.

 	\param addr receiver's address
	\param buffer data to send
	\param size size of data to send
 
    \return frame seq number <> 0 if successfull
 */
uint16_t net_send(uint8_t addr, uint8_t *buffer, uint8_t size) 
{
    uint16_t result = 0;
    pnet_packet_t pp = NULL;


    if (size > NET_PACKET_LOAD_SIZE)
        return 0;

    DBG_NET(usart_printf(PSTR("NET send dst:%d\r\n"), addr));  	
    
    #ifdef NET_ROUTE
        // allocate packet
        pp = net_packet_alloc();        
        if (pp) {
            // set packet header
            pp->p_type = NET_DTA;
            pp->p_dest_addr = addr; 
            pp->p_src_addr = net_ctx.n_nid;
            pp->p_size = size;
            // copy load
            memcpy(pp->p_load, buffer, size);
            // queue packet
            list_push(net_ctx.n_out_queue, pp);
            
            result = 1;
            
            // if no packets waiting in out queue, send it
            if (net_ctx.n_out_queue->l_count == 1) {
                if (!net_route(addr, pp)) {
                    list_pop(net_ctx.n_out_queue);
                    net_packet_free(&pp);
                    result = 0;
                } 
            }
        }
    #else
        {
            net_packet_t p; 
            p.p_type = NET_DTA;
            p.p_dest_addr = addr; 
            p.p_src_addr = net_ctx.n_nid;
            p.p_size = size;
            memcpy(p.p_load, buffer, size);
            result = link_send(addr, (uint8_t *)&p, net_packet_size((&p)));
        }
    #endif
        
    return result;
}


/*!
	Asynchronous receive.

	This function copies received data to buffer.

	\param addr sender's address 
	\param buffer buffer for data
	\param size size of received data
 */
uint8_t net_recv(uint8_t *addr, void *buffer, uint8_t size)
{
    pnet_packet_t p = NULL;
    
    #ifdef NET_ROUTING
        // get packet from in queue
        p = list_pop(net_ctx.n_in_queue);
        if (size < p->p_size) {
            net_packet_free(&p);
            return NET_ERROR;
        }
        *addr = p->p_src_addr;
        memcpy(buffer, p->p_load, p->p_size);
        net_packet_free(&p);
    #else
        if (size < net_ctx.n_current.p_size)
            return NET_ERROR;
        *addr = net_ctx.n_current.p_src_addr;
        memcpy(buffer, net_ctx.n_current.p_load, size);
	#endif

    DBG_NET(usart_printf(PSTR("NET recv src:%d\r\n"), *addr));  

    return NET_OK;	
}


/*!
	Initialize network layer.
	Function registers bottom half handlers for link layer events
*/
void net_init(pcfg_t cfg)
{
    uint8_t i = 0;

	link_init(cfg);
    net_ctx.n_nid = cfg->cfg_nid_addr;

    #ifdef NET_ROUTING
        DBG_NET(PSTR("Routing is enabled\r\n"));

        net_ctx.n_in_queue = list_create(5);
        net_ctx.n_out_queue = list_create(5);
        net_ctx.n_timeout = timer_add(net_timeout);
        net_ctx.n_clock = timer_add(net_clock);
    
        timer_set(net_ctx.n_timeout, NET_RREQ_TIMEOUT, 0, NULL);
        timer_set(net_ctx.n_clock, NET_CLOCK_RES, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);


        for (i = 0; i < NET_ROUTE_TABLE_SIZE; i++) {
            net_ctx.n_route[i].r_flags |= _BV(NET_ROUTE_UNKNOWN_SEQ);
        }
    #else
        DBG_NET(PSTR("Routing is disabled\r\n"));
    #endif    
        
    event_register(EVENT_LINK_TXC, net_link_txc, false);
    event_register(EVENT_LINK_RXC, net_link_rxc, false);
    event_register(EVENT_LINK_FAIL, net_link_fail, false);
}
