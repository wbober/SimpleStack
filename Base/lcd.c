#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

#define HD44780_SET(bit)   (HD44780_PORT |= _BV(bit))
#define HD44780_CLEAR(bit) (HD44780_PORT &= ~_BV(bit))		
#define HD44780_DATABITS (_BV(HD44780_D4)|_BV(HD44780_D5)|_BV(HD44780_D6)|_BV(HD44780_D7))

/*!
    \brief  Pulse E bit

    Function pulse the level on E bit, what strobes data available
    on the data pins into the HD44780 driver.
 */
static inline void hd44780_pulse_e(void) __attribute__((always_inline));
static inline void hd44780_pulse_e(void)
{
  HD44780_PORT |= _BV(HD44780_E);
  _delay_us(50.0);
  HD44780_PORT &= ~_BV(HD44780_E);
}

/*!
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


/*!
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

/*!
 * Setup the LCD controller.  First, call the hardware initialization
 * function, then adjust the display attributes we want.
 */
void lcd_init(uint8_t por)
{
    hd44780_init(por);
    hd44780_cmd(HD44780_CLR);
}

/*!
 */
void lcd_clr()
{
    hd44780_cmd(HD44780_CLR);
}

/*!
 */
void lcd_write(const char *s)
{
    while (*s != '\0') {
        hd44780_data(*s++);
    }
}

/*!
 */
void lcd_xy(uint8_t x, uint8_t y)
{
    
    uint8_t addr = (y == 1) ? 0x40 : 0 + x;
    hd44780_cmd(addr | HD44780_ADDR);
}
