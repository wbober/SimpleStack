#include "timers.h"
#include "event.h"
#include <avr/io.h>
#include <stdlib.h>

void	timer_interrupt();

static volatile timer_t timers[TIMERS_MAX];

volatile clock_t clock;


/*!
	Initalize timers to default values and registeres
	BH_TIMER bottom half handler
 */
void timer_init()
{
	ptimer_t t;

    #ifdef BASE
	    TCCR0 = _BV(WGM01) | _BV(CS00) | _BV(CS01) | _BV(CS02);
    #else
 	    TCCR0 = _BV(WGM01) | _BV(CS00) | _BV(CS02);
    #endif

	OCR0 = 0x0C;
	TIMSK = _BV(OCIE0);
    	
	for (t = (ptimer_t)timers; t < timers + TIMERS_MAX; ++t) {
        t->flags = 0;
		t->ticks = 0;
        t->counter = 0;
        t->pdata = NULL;
		t->callback = NULL;
	}

    clock.s = 0;
    clock.ms = 0;

	event_register(EVENT_TIMER, timer_event_handler, true);
}

/*!
	Timers module interrupt handler.

	Decrease all timers' tick counter by one. Activates BH_TIMER bottom half handler
 */
void timer_interrupt()
{
    ptimer_t t;

    for (t = (ptimer_t)timers; t < timers + TIMERS_MAX; t++) {
		if ((t->flags & _BV(TIMER_ENABLED)) && t->counter)
			t->counter--;
    }

    if (clock.ms++ == 1000) {
        clock.s++;
        clock.ms = 0;
    }

	event_activate(EVENT_TIMER, NULL);	
}

/*!
	Bottom half handler for timer interrupt.

	If timer's tick count is zero then function calls 
    callback function associated with timer. 
    
    Function disables timer after it reaches zero.
 */
void timer_event_handler(ptr_t p) 
{
	ptimer_t t;

	for (t = (ptimer_t)timers; t < timers + TIMERS_MAX; t++) {
		if ((t->flags & _BV(TIMER_ENABLED)) && !t->counter) {
            if (t->flags & _BV(TIMER_AUTO))
                t->counter = t->ticks;
            else
                t->flags &= ~_BV(TIMER_ENABLED);
			t->callback((uint8_t)(t - timers));
		}
    }
}

/*!
	Add new timer.

	The timer is automatically enabled.

	\param ticks ticks ticks to trigger timer
	\param callback function which should be called when timer reaches zero

	\return id of timer
	\return TIMERS_NO_FREE when no free timers present
 */
uint8_t timer_add(timer_callback_t callback) 
{
	ptimer_t t;

	for (t = (ptimer_t)timers; t < timers + TIMERS_MAX; ++t) {
		if (!(t->flags & _BV(TIMER_VALID))) {
            t->flags |= _BV(TIMER_VALID);
			t->ticks = 0;
            t->counter = 0;
			t->callback = callback;
            t->pdata = NULL;
			return (uint8_t)(t - timers);
		}
	}

	return t < timers + TIMERS_MAX ? TIMERS_OK : TIMERS_NO_FREE;
	
}

/*!
	Set timer ticks value.

	Timer is automatically enabled.

	\param ticks to trigger the timer
	\id timer id

	\return TIMERS_OK value set and timer enabled
	\return TIMERS_ERROR no timer with given id
 */
uint8_t timer_set(uint8_t id, uint16_t ticks, uint8_t flags, void *pdata) 
{
	if (id < TIMERS_MAX && (timers[id].flags & _BV(TIMER_VALID))) {
		timers[id].ticks = ticks;
		timers[id].counter = ticks;
        timers[id].flags = flags | _BV(TIMER_VALID);
        timers[id].pdata = pdata;
 		return TIMERS_OK;
	}
	return TIMERS_ERROR;
}

/*!
	Disables timer.

	\param id timer id
	
	\return TIMERS_OK timer disabled
	\return TIMERS_ERROR no timer with given id
 */
uint8_t timer_remove(uint8_t id) 
{
	if (id < TIMERS_MAX) {
        timers[id].flags &= ~(_BV(TIMER_VALID) | _BV(TIMER_ENABLED));
		timers[id].ticks = 0;
        timers[id].counter = 0;
		return TIMERS_OK;
	}
	return TIMERS_ERROR;
}

/*!
    Enable timer.

    \param id timer id

    \return TIMERS_OK timer enabled
    \retunr TIMERS_ERROR no timer with given id
*/

uint8_t timer_enable(uint8_t id)
{
    if (id < TIMERS_MAX && timers[id].ticks) {
        timers[id].flags |= _BV(TIMER_ENABLED);
        timers[id].counter = timers[id].ticks;
        return TIMERS_OK;
    }
    return TIMERS_ERROR;
}

uint8_t timer_disable(uint8_t id)
{
    if (id < TIMERS_MAX) {
        timers[id].flags &= ~_BV(TIMER_ENABLED);
        return TIMERS_OK;
    }
    return TIMERS_ERROR;
}


/*!
    Return user data associated with the timer.

    /param id timer id

    /return pointer to user data
 */
void*   timer_get_pdata(uint8_t id)
{
    if (id < TIMERS_MAX) {
        return timers[id].pdata;
    }
    
    return NULL;
}

