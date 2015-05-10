#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "cc1000.h"
#include "cc1000settings.h"
#include "defines.h"

#define CAL_TIMEOUT   0x7FFE
#define LOCK_TIMEOUT  0x7FFE

/*!
    CC1000 context.
 */
static volatile struct {
    cc_mode_t cc_mode;
    uint8_t cc_pow;
} cc1000_ctx;

/*!
    Set module configuration.
 */
void cc1000_set_cfg(uint8_t cfg, uint8_t msb, uint8_t lsb)
{
    switch (cfg) {
        // set output power
        case CC1000_CFG_POW:
            cc1000_ctx.cc_pow = lsb;
            cc1000_write_register(CC1000_PA_POW, cc1000_ctx.cc_pow);
            break;
    }
}

/**
    Initialize cc1000 transceiver.
 */
void cc1000_init()
{
	uint8_t i, addr, value;
    
    // set default output power
    cc1000_ctx.cc_pow = PA_VALUE;

    // set ports for serial configuration interface
    CC1000_PALE_OUTPUT;
    CC1000_PCLK_OUTPUT;
    CC1000_PDATA_OUTPUT;
    
    // set port for data clock
    CC1000_DCLK_RISING;

	// trun on crystal oscillator core
    cc1000_power_on(CORE_PD);
    // reset
    cc1000_reset();

	// write configuration registers
	for (i = 0; i < cc1000_registers_count; i++) {
		addr = pgm_read_byte(&cc1000_registers[i][0]);
		value = pgm_read_byte(&cc1000_registers[i][1]);
		cc1000_write_register(addr, value);
    }

    cc1000_switch_mode(CC1000_RX);
    cc1000_calibrate();
    
    cc1000_switch_mode(CC1000_TX);
    cc1000_calibrate();

	cc1000_sleep();
}

/**
    Write byte to the configuration interface.

    \param b byte to be written
 */
static void cc1000_write(uint8_t b)
{
    uint8_t c = 8;

    CC1000_PDATA_OUTPUT;

    while (c) {
      CC1000_PCLK_HIGH;
      
      if ((b & 0x80))
        CC1000_PDATA_HIGH;
      else
        CC1000_PDATA_LOW;
      b <<= 1;
  
      CC1000_PCLK_LOW;        
      c--;
    }

    CC1000_PCLK_HIGH;
}

/**
    Read byte form configuration interface.

    \return read byte
 */
static uint8_t cc1000_read(void)
{
    uint8_t c = 8;
    uint8_t b = 0;

    // turn on pull-up
    CC1000_PDATA_HIGH;
    CC1000_PDATA_INPUT;

    while (c) {
        b <<= 1;
        CC1000_PCLK_LOW;
        if (CC1000_PDATA)
            b |= 0x01;
        CC1000_PCLK_HIGH;
        c--;
    }

    CC1000_PDATA_OUTPUT;

    return b;
}

/**
    \brief Write to a CC1000 register.

    \param addr Address of the register
    \param data Value to write to the register
 */
void cc1000_write_register(uint8_t addr, uint8_t data)
{
    // Send address byte with write bit set on
    CC1000_PALE_LOW;
    cc1000_write((addr << 1) | 0x01);
    
    // Send data bits
    CC1000_PALE_HIGH;
    cc1000_write(data);
}

/**
    \brief Read from CC1000 register
 
    \param addr Address of the register
    \return The value of the register
 */
uint8_t cc1000_read_register(uint8_t addr)
{
    uint8_t result;

    // Send address bits
    CC1000_PALE_LOW;
    cc1000_write(addr << 1);
    
    // Receive data bits
    CC1000_PALE_HIGH;
    result = cc1000_read();

    return result;
}


/**
    \breif Reset CC1000
 */
void cc1000_reset(void)
{
  uint8_t value;
  
  value = cc1000_read_register(CC1000_MAIN);
  cc1000_write_register(CC1000_MAIN, value & ~_BV(RESET_N));         // Reset CC1000
  cc1000_write_register(CC1000_MAIN, value | _BV(RESET_N));         // Bring CC1000 out of reset
}

uint8_t cc1000_value_wait(uint8_t reg, uint8_t val, uint16_t timeout)
{
	uint8_t result = 0;
    for (; !result && timeout; timeout--) {
        result = cc1000_read_register(reg) & _BV(val);
		wdt_reset();
    }
	return result;
}

/**
    \brief Perform VCO and PLL self calibration.

    Function performs VCO calibration. To determine if the calibratin was
    successful PLL lock state is checked.

    \return 1 when sucessful 0 otherwise
 */
uint8_t cc1000_calibrate(void)
{  
    uint16_t timeout;
    uint8_t result;

    // start calibration
    cc1000_write_register(CC1000_CAL, 0xA6);

    // wait for calibration complete

	cc1000_value_wait(CC1000_CAL, CAL_COMPLETE, CAL_TIMEOUT);
	cc1000_value_wait(CC1000_LOCK, PLL_LOCK_CONTINOUS, LOCK_TIMEOUT);

    // end callibration
    cc1000_write_register(CC1000_CAL, 0x26); 

    return result;
}

/**
    \brief Switch between receive and transmit mode.

    \param mode
 */
uint8_t cc1000_switch_mode(cc_mode_t mode)
{
    uint16_t timeout;
    uint8_t result = 0;
    uint8_t main = cc1000_read_register(CC1000_MAIN);
    uint8_t current, pll;

    if (cc1000_ctx.cc_mode == mode)
        return 1;

    if (cc1000_ctx.cc_mode == CC1000_PD) {
        // turn on crystal oscillator core
        main &= ~_BV(CORE_PD);
        cc1000_write_register(CC1000_MAIN, main);
        _delay_ms(2.0);

        // turn on bias generator
        main &= ~_BV(BIAS_PD);
        cc1000_write_register(CC1000_MAIN, main);
    }

    switch (mode) {
        case CC1000_TX:
            cc1000_write_register(CC1000_PA_POW, 0);
            main = MAIN_TX;
            current = TX_CURRENT;
            pll = TX_PLL;
        break;
        
        case CC1000_RX:
            main = MAIN_RX;
            current = RX_CURRENT;
            pll = RX_PLL;
        break;
    }
    
    // switch mode
    cc1000_write_register(CC1000_MAIN, main);

    // write current and pll values
    cc1000_write_register(CC1000_PLL, pll);
    cc1000_write_register(CC1000_CURRENT, current);
    _delay_us(250);

    // TRY TO LOCK PLL UNTIL SUCCESFULL - this might be an infinite loop. If this fails the watchdog
    // will reset the CPU

    while(!result) {
        // wait for PLL lock
		result = cc1000_value_wait(CC1000_LOCK, PLL_LOCK_CONTINOUS, LOCK_TIMEOUT);

        // if lock unsuccessful then try to calibrate
        if (!result) {
			result = cc1000_calibrate();
        }
		
		if (!result) {
        	ResetFreqSynth();
        }
    }

    // restore output power
    if (mode == CC1000_TX)
        cc1000_write_register(CC1000_PA_POW, cc1000_ctx.cc_pow);

    cc1000_ctx.cc_mode = mode;

    return result;
}

void cc1000_sleep(void)
{
    cc1000_write_register(CC1000_MAIN, MAIN_PD);
    cc1000_write_register(CC1000_PA_POW, 0);
    cc1000_ctx.cc_mode = CC1000_PD;
}

/****************************************************************************/
/*  This routine locks the averaging filter of the CC1000                   */
/****************************************************************************/

void AverageManualLockCC1000(void)
{
  cc1000_write_register(CC1000_MODEM1,0x19);
}

/****************************************************************************/
/*  This routine unlocks the averaging filter of the CC1000                 */
/****************************************************************************/

void AverageFreeRunCC1000(void)
{
  cc1000_write_register(CC1000_MODEM1,0x09);
}

/****************************************************************************/
/*  This routine sets up the averaging filter of the CC1000 for automatic   */
/*  lock. This can be used in polled receivers.                             */
/****************************************************************************/

void AverageAutoLockCC1000(void)
{
  cc1000_write_register(CC1000_MODEM1,0x01);
}

/****************************************************************************/
/*  This routine reads the current calibration values from the CC1000       */
/****************************************************************************/

void ReadCurrentCalibration(char *val1, char *val2)
{
  *val1=cc1000_read_register(CC1000_TEST0);
  *val2=cc1000_read_register(CC1000_TEST2);
}

/****************************************************************************/
/*  This routine overrides the current calibration of the CC1000            */
/****************************************************************************/

void OverrideCurrentCalibration(char val1, char val2)
{
  cc1000_write_register(CC1000_TEST5,(val1&0x0F)|0x10);
  cc1000_write_register(CC1000_TEST6,(val2&0x1F)|0x20);
}

/****************************************************************************/
/*  This routine stops override of the CC1000 calibration values            */
/****************************************************************************/

void StopOverridingCalibration(void)
{
  cc1000_write_register(CC1000_TEST5,0x00);
  cc1000_write_register(CC1000_TEST6,0x00);
}
  

/****************************************************************************/
/*  This CC1000 frequency synthesizer                                       */
/****************************************************************************/
void ResetFreqSynth(void)
{
  char modem1_value;
  modem1_value = cc1000_read_register(CC1000_MODEM1)&~0x01;
  cc1000_write_register(CC1000_MODEM1,modem1_value);
  cc1000_write_register(CC1000_MODEM1,modem1_value|0x01);
}
