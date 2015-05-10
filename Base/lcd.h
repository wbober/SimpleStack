#ifndef _LCD_H_
#define _LCD_H_

#include <inttypes.h>

/* HD44780 LCD port connections */
#define HD44780_PORT        PORTC
#define HD44780_DDR         DDRC

/* Control bits */
#define HD44780_RS 		    PORT5
#define HD44780_E  		    PORT4

/* The data bits have to be in ascending order. */
#define HD44780_D4 		    PORT0
#define HD44780_D5 		    PORT1
#define HD44780_D6		    PORT2
#define HD44780_D7 		    PORT3

/* Display backlight port */
#define HD44780_LED_PORT    PORTB
#define HD44780_LED_DDR     DDRB
#define HD44780_LED         PORT4

#define HD44780_LED_ON      (PORTB |= _BV(HD44780_LED))
#define HD44780_LED_OFF     (PORTB &= ~_BV(HD44780_LED))

/** HD44780 Commands */
#define HD44780_CLR 	0x01
#define HD44780_HOME 	0x02
#define HD44780_FSET	0x28
#define HD44780_ADDR    0x80
#define HD44780_OFF     0x08
#define HD44780_ON		0x0C

void	hd44780_init(uint8_t);
void	hd44780_write(uint8_t b, uint8_t rs);

#define hd44780_cmd(n)	hd44780_write((n), 0)
#define hd44780_data(n)	hd44780_write((n), 1)

void lcd_init(uint8_t);
void lcd_xy(uint8_t x, uint8_t y);
void lcd_write(const char *s);
void lcd_clr();

#endif
