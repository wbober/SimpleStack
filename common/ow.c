#include "ow.h"
#include <avr/interrupt.h>
#include <util/crc16.h>	
#include <util/delay.h>
#include <stdlib.h>

#define OW_GET_IN()   (OW_IN & _BV(OW_PIN))
#define OW_OUT_LOW()  (OW_OUT &= ~_BV(OW_PIN))
#define OW_OUT_HIGH() (OW_OUT |= _BV(OW_PIN))

#define OW_DIR_IN()   (OW_DDR &= ~_BV(OW_PIN))
#define OW_DIR_OUT()  (OW_DDR |= _BV(OW_PIN))

typedef float (*convert_t)(uint8_t, uint8_t);
uint8_t status;

/*!
	Reset 1-wire bus.
 */
uint8_t ow_reset(void)
{
	uint8_t sreg;
	
	OW_OUT_LOW(); // disable internal pull-up (maybe on from parasite)
	OW_DIR_OUT(); // pull OW-Pin low for 480us
	
	sreg = SREG & _BV(SREG_I);
	cli();
	
	_delay_ms(0.5);
		
	// set Pin as input - wait for clients to pull low
	OW_DIR_IN(); // input

	_delay_ms(0.1);
	status = OW_GET_IN();
	
	SREG |= sreg;
	
	// after a delay the clients should release the line
	// and input-pin gets back to high due to pull-up-resistor
	_delay_ms(0.5);
	
	return status;
}

/*!
	Write byte on 1-wire bus.

	\param b byte to write
 */
void ow_write(uint8_t b)
{
	uint8_t i = 8, sreg;	
	
	do {
		sreg = SREG & _BV(SREG_I);
		cli();

		OW_OUT_LOW();	
		OW_DIR_OUT(); 	// init write slot
		_delay_us(5.0);

		if (b & 1) {
			OW_DIR_IN();
		}
		
		_delay_ms(0.1);
		OW_DIR_IN();

		SREG |= sreg;
	
		_delay_ms(0.1);	// time delay between write slots

		b >>= 1;
	} while( --i );

}

/*!
	Read byte from 1-wire bus.

	\return read byte
 */
uint8_t ow_read(void)
{
	uint8_t i = 8, b = 0, sreg;
	do {
		sreg = SREG & _BV(SREG_I);
		cli();
		
		b >>= 1;

		OW_OUT_LOW();
		OW_DIR_OUT();
		_delay_us(5.0); // init read time slot

		OW_DIR_IN();
		_delay_us(5.0);
		
		if (OW_GET_IN())
			b |= 0x80;

		_delay_ms(0.1); // wait until end of slot

		SREG |= sreg;	

		_delay_ms(0.1); // time delay between read slots	
	} while (--i);
	return b;
}

/*!
	Start temerature conversion.

	This function use SKIP_ROM command, so only one sensor
	can be used.
 */
void ds18x20_start_conversion()
{
	ow_reset();
	ow_write(DS_CMD_SKIP_ROM);
	ow_write(DS_CMD_CONVERT_T);
}

/*!
	Convert from ds18s20 data format to float.
 */
float ds18s20_convert(uint8_t msb, uint8_t lsb)
{
	float value = 0;
	lsb = msb & 0x80 ? ~lsb + 1 : lsb;
	value = (lsb >> 1) + (lsb & 1 ? 0.5 : 0);
	return msb & 0x80 ? -value : value;
}

/*!
	Convert from ds18b20 data format to float.
 */
float ds18b20_convert(uint8_t msb, uint8_t lsb)
{
	lsb = (lsb >> 3) | ((msb & 0x07) << 5);
	return ds18s20_convert(msb, lsb);
}

/*!
	Read temperature from sensor.

	Fuction is able to detect the type of used sensor and performe
	aproppriate temperature conversion. Because SKIP_ROM command is used
	only one sensor can be present on a bus.
 */
float ds18x20_get_temperature()
{
	uint8_t sp[DS_SCRATCHPAD_SIZE];
	uint8_t i = 0;
	uint8_t ROM = 0;
    float value;

	// get family code
	ow_reset();
	ow_write(DS_CMD_READ_ROM);
	ROM = ow_read();

	// read scratchpad
	ow_reset();
	ow_write(DS_CMD_SKIP_ROM);
	ow_write(DS_CMD_READ_SCRATCHPAD);
	
	for (i = 0; i < DS_SCRATCHPAD_SIZE; i++) {
		sp[i] = ow_read();
	}

	// choose conversion function
	switch (ROM) {
		case DS_18S20:
			value = ds18s20_convert(sp[DS_SP_TMSB], sp[DS_SP_TLSB]);
			break;
		case DS_18B20:
			value = ds18b20_convert(sp[DS_SP_TMSB], sp[DS_SP_TLSB]);
			break;
        default:
            value = -99;
            break;
	}
	
    return value;
}
