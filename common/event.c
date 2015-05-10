#include "event.h"
#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>

#ifdef BASE
#include "../Base/usart.h"
#endif

/* Internal array of events */
static volatile event_t events[EVENTS_MAX];

/* Last object in event array */
static volatile const pevent_t event_last = (pevent_t) events + EVENTS_MAX;

/**
	\brief Initalize events structures.
 */
void event_init() 
{
    pevent_t e = (pevent_t)events;

    while (e != event_last) {
        e->e_flags = 0;
        e->e_callback = NULL;
        e->e_pdata = NULL;
        e++;
    }
}

/**
 */
void event_handle_queue() 
{
    pevent_t hp_e, e;

    for (e = (pevent_t)events; e < events + EVENTS_MAX; e++) {
        // handle high priority events
        for (hp_e = (pevent_t)events; hp_e < events + EVENTS_MAX; hp_e++) {
            if (hp_e->e_flags & _BV(EVENT_ACTIVE) && hp_e->e_flags & _BV(EVENT_HIGH_PRIORITY) && hp_e->e_callback) {
                DBG_EVT(usart_printf(PSTR("event_activate: e:%d\r\n"), e-events));
                hp_e->e_flags &= ~_BV(EVENT_ACTIVE);        
                hp_e->e_callback(hp_e->e_pdata);                
                DBG_EVT(usart_printf(PSTR("event_handle_queue: r:%d"), 1));
            }
            wdt_reset();
        }
          // handle next low prio event
        if (e->e_flags & _BV(EVENT_ACTIVE) && !(e->e_flags & _BV(EVENT_HIGH_PRIORITY)) && e->e_callback) {
            DBG_EVT(usart_printf(PSTR("event_activate: e:%d\r\n"), e-events));
            e->e_flags &= ~_BV(EVENT_ACTIVE);        
            e->e_callback(e->e_pdata);
            DBG_EVT(usart_printf(PSTR("event_handle_queue: r:%d"), 1));
        }
        wdt_reset();
    }
}

/**
 */
uint8_t event_register(event_id_t e_id, event_callback_t e_callback, bool e_hp)
{
    if (e_id >= EVENTS_MAX) 
        return 0;
    
    events[e_id].e_callback = e_callback;
    events[e_id].e_flags = e_hp ? _BV(EVENT_HIGH_PRIORITY) : 0;

    return 1;
}

/**
    Activate event

    \param e_id event id
    \param e_data pointer to user data
 */
void event_activate(event_id_t e_id, ptr_t e_data)
{
    if (e_id >= EVENTS_MAX)
        return;
    events[e_id].e_flags |= _BV(EVENT_ACTIVE);
    events[e_id].e_pdata = e_data;
    //DBG({if (events[e_id].e_flags & _BV(EVENT_DBG_LOG)) usart_printf(PSTR("event_activate: e:%d\r\n"), e_id);});
}

