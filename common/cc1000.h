/* Constants defined for CC1000 */
/* Register addresses */

#include <inttypes.h>
#include <avr/io.h>
#include "cc1000defs.h"

typedef enum {
    CC1000_TX = 1,
    CC1000_RX = 2,
    CC1000_PD = 3
} cc_mode_t;

#define CC1000_CFG_POW  1

/* Pin usage definitions */
#ifdef BASE

#define CC1000_PCLK_OUTPUT      (DDRB |= _BV(PB7))
#define CC1000_PCLK_HIGH        (PORTB |= _BV(PB7))
#define CC1000_PCLK_LOW         (PORTB &= ~_BV(PB7))

#define CC1000_PDATA_OUTPUT     (DDRG |= _BV(PG3))
#define CC1000_PDATA_INPUT      (DDRG &= ~_BV(PG3))
#define CC1000_PDATA_HIGH       (PORTG |= _BV(PG3))
#define CC1000_PDATA_LOW        (PORTG &= ~_BV(PG3))
#define CC1000_PDATA            (PING & _BV(PG3))

#define CC1000_PALE_OUTPUT      (DDRG |= _BV(PG4))
#define CC1000_PALE_HIGH        (PORTG |= _BV(PG4))
#define CC1000_PALE_LOW         (PORTG &= ~_BV(PG4))

#define CC1000_DIO_OUTPUT       (DDRE |= _BV(PE6))
#define CC1000_DIO_INPUT        (DDRE &= ~_BV(PE6))
#define CC1000_DIO_HIGH         (PORTE |= _BV(PE6))
#define CC1000_DIO_LOW          (PORTE &= ~_BV(PE6))
#define CC1000_DIO              (PINE & _BV(PE6))

#define CC1000_DCLK_ENABLE      (EIMSK |= _BV(INT7))
#define CC1000_DCLK_DISABLE     (EIMSK &= ~_BV(INT7)), (EIFR |= _BV(INT7))
#define CC1000_DCLK_RISING      (EICRB |= _BV(ISC71) | _BV(ISC70))
#define CC1000_DCLK_FALLING     (EICRB &= ~_BV(ISC70)), (EICRB |= _BV(ISC71))

#else

#define CC1000_PCLK_OUTPUT      (DDRD |= _BV(PD6))
#define CC1000_PCLK_HIGH        (PORTD |= _BV(PD6))
#define CC1000_PCLK_LOW         (PORTD &= ~_BV(PD6))

#define CC1000_PDATA_OUTPUT     (DDRD |= _BV(PD5))
#define CC1000_PDATA_INPUT      (DDRD &= ~_BV(PD5))
#define CC1000_PDATA_HIGH       (PORTD |= _BV(PD5))
#define CC1000_PDATA_LOW        (PORTD &= ~_BV(PD5))
#define CC1000_PDATA            (PIND & _BV(PD5))

#define CC1000_PALE_OUTPUT      (DDRD |= _BV(PD4))
#define CC1000_PALE_HIGH        (PORTD |= _BV(PD4))
#define CC1000_PALE_LOW         (PORTD &= ~_BV(PD4))

#define CC1000_DIO_OUTPUT       (DDRD |= _BV(PD1))
#define CC1000_DIO_INPUT        (DDRD &= ~_BV(PD1))
#define CC1000_DIO_HIGH         (PORTD |= _BV(PD1))
#define CC1000_DIO_LOW          (PORTD &= ~_BV(PD1))
#define CC1000_DIO              (PIND & _BV(PD1))

#define CC1000_DCLK_ENABLE      (GICR |= _BV(INT0))
#define CC1000_DCLK_DISABLE     (GICR &= ~_BV(INT0))
#define CC1000_DCLK_RISING      (MCUCR |= _BV(ISC01) | _BV(ISC00))
#define CC1000_DCLK_FALLING     {MCUCR &= ~_BV(ISC00); MCUCR |= _BV(ISC01);} 

#endif

void    cc1000_init();
void    cc1000_write_register(uint8_t addr, uint8_t data);
uint8_t cc1000_read_register(uint8_t addr);
void    cc1000_reset(void);
uint8_t cc1000_calibrate(void);
uint8_t cc1000_switch_mode(uint8_t mode);
void    cc1000_sleep(void);
void    cc1000_set_cfg(uint8_t cfg, uint8_t msb, uint8_t lsb);

#define cc1000_power_on(p)      cc1000_write_register(CC1000_MAIN, MAIN_PD & ~_BV(p))
#define cc1000_write_word(w)    cc1000_write_register(w >> 8, w & 0xFF);

void AverageManualLockCC1000(void);
void AverageFreeRunCC1000(void);
void AverageAutoLockCC1000(void);
void ReadCurrentCalibration(char *val1, char *val2);
void OverrideCurrentCalibration(char val1, char val2);
void StopOverridingCalibration(void);
void ResetFreqSynth(void);
