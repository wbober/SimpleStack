#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <stdio.h>
#include "app.h"
#include "event.h"
//#include "control.h"
#include "cc1000.h"
#include "defines.h"
#include "link.h"
#include "network.h"
#include "transport.h"
#include "timers.h"
#include "cc1000settings.h"
#include "rssi.h"
#include "mac.h"

const cfg_t EEMEM ee_cfg = {
    .cfg_nid_addr = 4,
    .cfg_temp_cal = 0
};

static volatile uint8_t mcusr_mirror;


// interrupt on DCLK pin
ISR(INT0_vect)
{
	mac_irq_handler();
}


// timer interrupt
ISR(TIMER0_COMP_vect)
{
	timer_interrupt();
}

/*
void my_init(void) __attribute__((naked)) __attribute__ ((section (".init3")));
void my_init(void)
{
      extern uint8_t __bss_end; // Linker variable, used by memfill routine - filled at compile time
      uint8_t* RamLoc;
   
      for (RamLoc = (uint8_t*)&__bss_end; RamLoc < (uint8_t*)RAMEND; RamLoc++)
        *RamLoc = 0xDC; 
}
*/

/*!
    Initalize I/O
*/
void init_io(void)
{
    CC1000_PCLK_OUTPUT;
    CC1000_PALE_OUTPUT;
    CC1000_PDATA_OUTPUT;

    // set output for LEDs
    LED_DDR |= _BV(LED_RED) | _BV(LED_GREEN);

	// set relay io
    RELAY_DDR |= _BV(RELAY_PIN);
	RELAY_OFF;
}

int main(void)
{
    cfg_t cfg;
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();     

    // read cfg from eeprom
    eeprom_busy_wait();
    eeprom_read_block(&cfg, &ee_cfg, sizeof(cfg_t));

    init_io();
   
    timer_init();    
    rssi_init();
    
    tcp_init(&cfg);    
    app_init(&cfg);

    wdt_enable(WDTO_500MS);
    sei();  
	while(1) {
        event_handle_queue();
	}
}


