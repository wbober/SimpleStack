#include <avr/io.h>
#include "rssi.h"

static volatile uint16_t rssi_value;

/*!
    \brief Initialize RSSI measuring module.

    Function initializes ADC, switching internal ADC reference,
    selecting appropriate ADC input pin and seting ADC conversion
    clock divider.
*/
void rssi_init() 
{
   // internal 2.56V reference, ADC3 pin
    #ifdef BASE
	    ADMUX = _BV(REFS0) | _BV(REFS1) | _BV(MUX1);
    #else
    	ADMUX = _BV(REFS0) | _BV(REFS1) | _BV(MUX0) | _BV(MUX1);
    #endif
   
    // set prescaler clock
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1);

	// enable ADC module
	ADCSRA |= _BV(ADEN);

    /*
    #ifdef BASE
        // set the ADC into free running mode and init first conversion
        ADCSRA |= _BV(ADFR) | _BV(ADSC);
    #endif
    */
}

/*!
	\brief Get the RSSI value.

    Read the value of RSSI stored on last rssi_measure() invocation.

    \breif The value of RSSI in dBm.
 */
int16_t rssi_get_value()
{
    uint16_t value;
    do {
    } while (ADCSRA & _BV(ADSC));
	value = (ADCL | (ADCH << 8)) << 1;
    return (int16_t)(-0.12825*(float)(value) - 49.2); 
}

/*!
	\brief Get current rssi value.

    Routing reads current value of ADC data register and stores it
    in internal variable. It can be read by invoking rssi_get_value()
 */
void rssi_measure() 
{
    ADCSRA |= _BV(ADSC);
}

