#include <avr/pgmspace.h>
#include "cc1000.h"
#include "cc1000settings.h"

// registers' values calculated with SmartRF Studio
const prog_uint8_t cc1000_registers[][2] PROGMEM = {
	{CC1000_FREQ_2A, 	0x66}, 
	{CC1000_FREQ_1A, 	0xA0}, 
	{CC1000_FREQ_0A, 	0x00},
	{CC1000_FREQ_2B, 	0x41},
	{CC1000_FREQ_1B, 	0xF2},
	{CC1000_FREQ_0B, 	0x53},
	{CC1000_FSEP1, 		0x02},
	{CC1000_FSEP0, 		0x80},
	{CC1000_FRONT_END, 	0x12},
	{CC1000_LOCK, 		0x00}, /* LOCK pin disabled */
	{CC1000_CAL, 		0x26}, 
	{CC1000_MODEM2,		0x8D},
	{CC1000_MODEM1,		0x6F},
	{CC1000_MODEM0,		0x27}, /* 2.4kBoud */
	{CC1000_MATCH,		0x70},
	{CC1000_FSCTRL,		0x01},
	{CC1000_PRESCALER, 	0x00},
	{CC1000_TEST4,		0x25}
};

const uint8_t cc1000_registers_count = sizeof(cc1000_registers)/sizeof(cc1000_registers[0]);
/*
const uint8_t cc1000_rx_current;
const uint8_t cc1000_tx_current;
const uint8_t cc1000_rx_pll;
const uint8_t cc1000_tx_pll;
*/
