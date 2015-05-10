#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <util/delay.h>

#include "hd44780.h"


#define HD44780_SET(bit)   (HD44780_PORT |= _BV(bit))
#define HD44780_CLEAR(bit) (HD44780_PORT &= ~_BV(bit))		

#define HD44780_DATABITS (_BV(HD44780_D4)|_BV(HD44780_D5)|_BV(HD44780_D6)|_BV(HD44780_D7))

static inline void hd44780_pulse_e(void) __attribute__((always_inline));
/**
    \brief  Pulse E bit

    Function pulse the level on E bit, what strobes data available
    on the data pins into the HD44780 driver.
 */
static inline void hd44780_pulse_e(void)
{
  HD44780_PORT |= _BV(HD44780_E);
  _delay_us(1.0);
  HD44780_PORT &= ~_BV(HD44780_E);
}

/**
    \brief  Write byte to LCD.

    Function writes a byte to the HD44780 driver using 4 bit interface. The
    rs flag determines if a data or a command byte is being written.

    \param b byte to be writen
    \param rs value of the rs bit; 1 for command; 0 for data

 */
void hd44780_write(uint8_t b, uint8_t rs)
{
    if (rs)
        HD44780_SET(HD44780_RS);
    else
        HD44780_CLEAR(HD44780_RS);

    HD44780_PORT = (HD44780_PORT & ~HD44780_DATABITS) | (b >> 4);
    hd44780_pulse_e();

    HD44780_PORT = (HD44780_PORT & ~HD44780_DATABITS) | (b & 0x0F);
    hd44780_pulse_e();
}


/**
    \brief Initialize the LCD controller.
 */
void hd44780_init(uint8_t por)
{
    // set output pins
	HD44780_DDR = _BV(HD44780_RS) | _BV(HD44780_E) | HD44780_DATABITS;
    HD44780_LED_DDR = _BV(HD44780_LED);
    
    // turn on led
    HD44780_LED_ON;

    // the lcd interface type (4bit or 8bit) must be selected only
    // once after lcd is powered on
    if (por) {
        // select 4-bit interface
        HD44780_PORT = (HD44780_PORT & ~HD44780_DATABITS) | 0x02;
        hd44780_pulse_e();
        _delay_ms(15.0);
    }
    
    // set function and turn on
    hd44780_cmd(HD44780_FSET);
    _delay_ms(15.0);
	hd44780_cmd(HD44780_ON);
    _delay_ms(15.0);
}

