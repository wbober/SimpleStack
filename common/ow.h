#ifndef _OW_H_
#define _OW_H_

#include <avr/io.h>

#ifdef BASE
    #define OW_PIN  PF3
    #define OW_IN   PINF
    #define OW_OUT  PORTF
    #define OW_DDR  DDRF
#else
    #define OW_PIN  PB0
    #define OW_IN   PINB
    #define OW_OUT  PORTB
    #define OW_DDR  DDRB
#endif

uint8_t ow_reset(void);
uint8_t ow_read(void);
void ow_write(uint8_t b);

void ds18x20_start_conversion();
float ds18x20_get_temperature();

#define DS_CMD_SKIP_ROM 		0xCC
#define DS_CMD_CONVERT_T		0x44
#define DS_CMD_READ_SCRATCHPAD 	0xBE
#define DS_CMD_READ_ROM			0x33

#define DS_18S20				0x10
#define DS_18B20				0x28

#define DS_SP_TLSB				0x00
#define DS_SP_TMSB				0x01

#define DS_SCRATCHPAD_SIZE		9
#define DS_CONVERSION_TIME		750
#endif
