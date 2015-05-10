#ifndef _CC1000_SETTINGS_H
#define _CC1000_SETTINGS_H

#include <avr/pgmspace.h>

extern const prog_uint8_t cc1000_registers[][2] PROGMEM;
extern const uint8_t cc1000_registers_count;
extern const uint8_t cc1000_rx_current;
extern const uint8_t cc1000_tx_current;
extern const uint8_t cc1000_rx_pll;
extern const uint8_t cc1000_tx_pll;



#endif
