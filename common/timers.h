#ifndef _TIMERS_H_
#define _TIMERS_H_

#include <inttypes.h>
#include "defines.h"

#define TIMERS_MAX 		0x0A
#define TIMERS_OK		0xFF
#define TIMERS_NO_FREE	0xFE
#define TIMERS_ERROR	0xFD

#define TIMER_ENABLED   0
#define TIMER_AUTO      1
#define TIMER_VALID     2

typedef void (*timer_callback_t)(uint8_t) ;

typedef struct {
    uint8_t flags;
    // max timer value
    uint16_t ticks;
    // current timer value
	uint16_t counter;
    // pointer to user data
    void*   pdata;
    // callback function
	timer_callback_t callback;
} timer_t, *ptimer_t;

typedef struct {
    uint32_t s;
    uint16_t ms;
} clock_t;

void	    timer_init();
void    	timer_interrupt()  __attribute__((always_inline));
void 	    timer_event_handler(ptr_t p);
uint8_t     timer_add(timer_callback_t callback);
uint8_t     timer_set(uint8_t id, uint16_t ticks, uint8_t flags, void *pdata);
uint8_t     timer_enable(uint8_t id);
uint8_t     timer_disable(uint8_t id);
void*       timer_get_pdata(uint8_t id);
uint8_t     timer_remove(uint8_t id);

extern volatile clock_t clock;

// inline void () __attribute__((always_inline));
//inline void delay_us(uint8_t t)
//{
//}

#endif
