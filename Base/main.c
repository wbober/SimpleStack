#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <stdio.h>
#include "app.h"
#include "debug.h"
#include "event.h"
#include "usart.h"
#include "cc1000.h"
#include "defines.h"
#include "link.h"
#include "network.h"
#include "transport.h"
#include "timers.h"
#include "cc1000settings.h"
#include "rssi.h"
#include "lcd.h"
#include "link.h"
#include "mac.h"
#include "pool.h"

const cfg_t EEMEM ee_cfg = {
    .cfg_nid_addr = 1,
    .cfg_temp_cal = 0,
    .cfg_relays = 0
};

static volatile cfg_t cfg;
static volatile uint8_t mcusr_mirror;


// interrupt on DCLK pin
ISR(INT7_vect)
{
	mac_irq_handler();	
}

// timer interrupt
ISR(TIMER0_COMP_vect)
{
	timer_interrupt();
}


void init_io(void)
{
  // set output for LEDs
    LED_DDR |= _BV(LED_RED) | _BV(LED_GREEN);
}

void store_cfg(pcfg_t cfg) {
	wdt_enable(WDTO_2S);
	cli();
    eeprom_busy_wait();
    eeprom_write_block(cfg, &ee_cfg, sizeof(cfg_t));
	sei();
	wdt_enable(WDTO_500MS);
}

void load_cfg(pcfg_t cfg) {
    eeprom_busy_wait();
    eeprom_read_block(cfg, &ee_cfg, sizeof(cfg_t));	
}


int main(void)
{
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    
    wdt_enable(WDTO_2S); 
    load_cfg(&cfg);

    init_io();
    
    //MCUCSR &= ~(_BV(PORF) | _BV(JTRF));
    timer_init();    
    rssi_init();

    usart_init();
    if (mcusr_mirror & _BV(WDRF))
        usart_printf(PSTR("WATCHDOG RESET\r\n"));
    if (mcusr_mirror & _BV(JTRF)) 
        usart_printf(PSTR("JTAG RESET\r\n"));

    wdt_reset();

    tcp_init(&cfg);
    app_init(&cfg);

    wdt_enable(WDTO_500MS);
    
    sei();
    while(1) {
        event_handle_queue();
	}

}


