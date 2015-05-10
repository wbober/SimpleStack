#ifndef _DEFINES_H_
#define _DEFINES_H_

#include <avr/io.h>

#define RELAY_PIN	PD0	
#define RELAY_PORT	PORTD
#define RELAY_DDR	DDRD

#define RELAY_ON	(RELAY_PORT |= _BV(RELAY_PIN))
#define RELAY_OFF	(RELAY_PORT &= ~_BV(RELAY_PIN))

#define DQ			0	// PORTB, pin 0

/* LED connections */
#define LED_PORT	PORTC
#define LED_DDR		DDRC
#define LED_RED		PORT0
#define LED_GREEN	PORT1

#define LED_ON(l)	    (LED_PORT |= _BV(l))
#define LED_OFF(l)	    (LED_PORT &= ~_BV(l))
#define LED_TOGGLE(l)   (LED_PORT ^= _BV(l))


#endif
