#ifndef _EVENTS_H_
#define _EVENTS_H_

/** \defgroup event <event.h> Event handling */
#include <inttypes.h>
#include <defines.h>

#define EVENTS_MAX     	20
#define EVENTS_OK		0xFF
#define EVENTS_NO_FREE	0xFE
#define EVENTS_ERROR	0xFD

/** \name Event flags definitions */
/*@{*/

/** \ingroup event 
    The event is marked as active */
#define EVENT_ACTIVE            0
#define EVENT_HIGH_PRIORITY     1
#define EVENT_DBG_LOG           2

/*@}*/

/** Events */
typedef enum event_ids {
    EVENT_TIMER,
    EVENT_USART_RXC,
    EVENT_USART_TXC,
    EVENT_MAC_TXC,
    EVENT_MAC_FAIL,
    EVENT_LINK_TXC,
    EVENT_LINK_RXC,
    EVENT_LINK_FAIL,
    EVENT_NET_RXC,
    EVENT_NET_TXC,
    EVENT_NET_FAIL,
    EVENT_TCP_RXC,
    EVENT_TCP_TXC,
    EVENT_TCP_FAIL,
    EVENT_APP_MAIN,
	EVENT_APP_STATUS,
    EVENT_RADIO_RXC,
    EVENT_RADIO_TXC
} event_id_t;

/** Event callback function type definition */
typedef void (*event_callback_t)(ptr_t) ;

/** Event structure */
typedef struct {
    uint8_t e_flags;                /**< Event flags */
    ptr_t   e_pdata;                /**< Pointer to user data */
    event_callback_t e_callback;    /**< Event callback function */
} event_t, *pevent_t;

/*@{*/

/**
	\brief Initalize events structures.
 */
void event_init();

/**
    \brief Handle triggered events queue.

    Function loops over the array of registered events and checks if the 
    event was activated. If so, the callback function associated with 
    the event is being invoked and the event is deactivated.
*/
void event_handle_queue();

/**
    \brief Register callback function for event

    \param e_id Event id number
    \param e_callback Pointer to callback function
    \param e_hp True if event should have high priority, false otherwise

    \return EVENTS_OK if successfull, EVENTS_ERROR otherwise
*/
uint8_t event_register(event_id_t e_id, event_callback_t e_callback, bool e_hp);

/**
    \brief Activate event

    \param e_id Event id number
    \param e_data Pointer to user data which should be associated with event.
*/
void event_activate(event_id_t e_id, ptr_t e_data);

/*@}*/

#endif
