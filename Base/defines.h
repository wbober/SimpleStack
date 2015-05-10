#ifndef _BASE_DEFINES_H_
#define _BASE_DEFINES_H_

#include <avr/io.h>

#define RELAY_1 	PORT0
#define RELAY_2     PORT1	
#define RELAY_PORT	PORTG
#define RELAY_DDR	DDRG
#define RELAY_COUNT 2

#define RELAY_ON(r)	    (RELAY_PORT |= _BV(r))
#define RELAY_OFF(r)	(RELAY_PORT &= ~_BV(r))

#define DQ			0	// PORTB, pin 0

#define	AUX0		0	// PORTC, pin 0
#define AUX1		1	// PORTC, pin 1

/* LED connections */
#define LED_PORT	PORTA
#define LED_DDR		DDRA
#define LED_RED		PORT0
#define LED_GREEN	PORT1

#define LED_ON(l)	    (LED_PORT |= _BV(l))
#define LED_OFF(l)	    (LED_PORT &= ~_BV(l))
#define LED_TOGGLE(l)   (LED_PORT ^= _BV(l))

/* PSU connection */
#define PSU_PORT    PORTF
#define PSU_DDR     DDRF
#define PSU_PIN     PORT1

#define PSU_ON      (PSU_PORT &= ~_BV(PSU_PIN))
#define PSU_OFF     (PSU_PORT |= _BV(PSU_PIN))


#endif
