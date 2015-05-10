#include "ds18B20.h"
#include <util/delay.h>
#include <avr/io.h>


void W1_Init()
{
	SFIOR &=  ~(1 << PUD);
}

unsigned char W1_Reset()
{
	W1_DDR |= (1 << W1_PIN); // set W1_PIN as output
	W1_OUTPORT &= ~(1 << W1_PIN); // set LOW state on bus 
	_delay_ms(0.5); 
	
	W1_OUTPORT |= (1 << W1_PIN); // set HIGH on bus
	W1_DDR &= ~(1 << W1_PIN); // set W1_PIN as input with pullup
	_delay_ms(0.1);
	if (bit_is_set(W1_INPORT, W1_PIN))
	{
		_delay_ms(0.4);// wait to end of slot
		return 1;
	}
	_delay_ms(0.4);// wait to end of slot
	return 0;
}

void W1_WriteByte(unsigned char byte)
{
	unsigned char b = byte;
	unsigned char i;
	for (i = 0; i < 8; i++)
	{
		asm volatile("ror %0" : "=r" (b) : "0" (b));
		if (bit_is_set(SREG, SREG_C))
		{
			W1_DDR |= (1 << W1_PIN); // set W1_PIN as output
			W1_OUTPORT &= ~(1 << W1_PIN); // set LOW state on bus 
			_delay_us(5.0);
		
			W1_OUTPORT |= (1 << W1_PIN); // set HIGH on bus
			W1_DDR &= ~(1 << W1_PIN); // set W1_PIN as input with pullup
			_delay_ms(0.07);		
		}
		else
		{
			W1_DDR |= (1 << W1_PIN); // set W1_PIN as output
			W1_OUTPORT &= ~(1 << W1_PIN); // set LOW state on bus 
			_delay_ms(0.06);
	
			W1_OUTPORT |= (1 << W1_PIN); // set HIGH on bus
			W1_DDR &= ~(1 << W1_PIN); // set W1_PIN as input with pullup
			_delay_us(15.0);

		}
	}
}

unsigned char W1_ReadByte(void)
{
	
	unsigned char i, byte = 0;
	for (i = 0; i < 8; i++)
	{
		W1_DDR |= (1 << W1_PIN); // set W1_PIN as output
		W1_OUTPORT &= ~(1 << W1_PIN); // set LOW state on bus 
		_delay_us(2.0);
		W1_OUTPORT |= (1 << W1_PIN); // set HIGH on bus
		W1_DDR &= ~(1 << W1_PIN); // set W1_PIN as input with pullup
		_delay_us(10.0);
		if (bit_is_set(W1_INPORT, W1_PIN))
			SREG |= (1 << SREG_C);
		else
			SREG &= ~(1 << SREG_C);
		
		asm volatile("ror %0" : "=r" (byte) : "0" (byte));
		_delay_ms(0.060);
	}

	return byte;
	
}
