#ifndef _ds18B20_h_
#define _ds18B20_h_


#include <inttypes.h>


#define W1_OUTPORT 	PORTB
#define W1_DDR 		DDRB
#define W1_INPORT 	PINB
#define W1_PIN 		0



#define SEARCH_ROM 0xF0
#define READ_ROM 0x33
#define MATCH_ROM 0x55
#define SKIP_ROM 0xCC
#define ALARM_SEARCH 0xEC
#define CONVERT_T 0x44
#define WRITE_SCRATCHPAD 0x4E
#define READ_SCRATCHPAD 0xBE
#define COPY_SCRATCHPAD 0x48
#define RECALL_E2 0xB8
#define READ_POWER_SUPPLY 0xB4

void W1_Init(void);//enable global pullup

//Reset/Init Pulse 
// return 
// 0 if was presence pule
// 1 if was not presence pulse
unsigned char W1_Reset(void); 

void W1_WriteByte(unsigned char byte); // write bit 
unsigned char W1_ReadByte(void); // read bit

#endif
