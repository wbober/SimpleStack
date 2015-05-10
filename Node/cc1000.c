/****************************************************************************/
/* Application note AN009                                                   */
/* CC1000 interface library                                                 */
/*                                                                          */
/* File:      cc1000avr.c                                                   */
/* Revision:  2.1                                                           */
/*                                                                          */
/* Microcontroller:                                                         */
/*          Atmel AVRmega8L                                                 */
/* Written for the IAR AVR compiler                                         */
/*                                                                          */
/* Author:  Karl H. Torvmark, Field Applications Engineer, Chipcon          */
/*                                                                          */
/* Contact: Chipcon AS +47 22 95 85 44                                      */
/*          wireless@chipcon.com                                            */
/*                                                                          */
/* Changes:                                                                 */
/*      2.1 : First AVR version                                             */
/****************************************************************************/

/****************************************************************************/
/* This library contains functions for configuring the CC1000. These        */
/* routines use bit-banging to program the CC1000.                          */
/* Routines to read and write the calibration values in the CC1000 are      */
/* provided, they aree useful in frequency-agile and frequency hopping      */
/* applications. See application note AN009 for more information.           */
/* The routines in this file will have to be adapted depending on the MCU   */
/* and compiler used.                                                       */
/****************************************************************************/

/*                                                                           *
 * Revision history:                                                         *
 *                                                                           *
 * $Log: cc1000avr.c,v $
 * Revision 2.5  2003/05/08 10:51:52  tos
 * Corrected LOCK monitor in Calibrate.
 *
 * Revision 2.4  2003/05/08 10:05:26  tos
 * Corrections according to Errata Note 01: reset freq.synth if unable to lock PLL.
 *
 * Revision 2.3  2003/04/28 08:21:14  tos
 * Corrected inconsistent monitoring of CC1000: [calibration complete] + [lock].
 *
 *
 *                                                                           *
 ****************************************************************************/

#include "cc1000.h"
#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>


#define CAL_TIMEOUT   0x7FFE
#define LOCK_TIMEOUT  0x7FFE


/* Contents of PLL register for TX and RX, use SmartRF(R) Studio */
/* to find values for your application */

#define TX_PLL 0x48
#define RX_PLL 0x70

#define PA_VALUE 0xFF


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

void ConfigureCC1000(char Count, short Configuration[])
{
    //
}

/****************************************************************************/
/*  This routine writes to a single CC1000 register                         */
/****************************************************************************/

void WriteToCC1000Register(char addr, char data)
{
    CC1000_PALE_LOW;
    cc1000_write((addr << 1) | 0x01);
    
    // Send data bits
    CC1000_PALE_HIGH;
    cc1000_write(data);
}

/****************************************************************************/
/*  This routine writes to a single CC1000 register, with data and address  */
/*  given in the same variable                                              */
/****************************************************************************/

void WriteToCC1000RegisterWord(short addranddata)
{
 
  ConfigureCC1000(1,&addranddata);
}

/****************************************************************************/
/*  This routine reads from a single CC1000 register                        */
/****************************************************************************/

uint8_t ReadFromCC1000Register(char addr)
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
  
/****************************************************************************/
/*  This routine resets the CC1000, clearing all registers.                 */
/****************************************************************************/  

void ResetCC1000(void)
{
  char MainValue = 0;
  
  MainValue=ReadFromCC1000Register(CC1000_MAIN);
  WriteToCC1000Register(CC1000_MAIN,MainValue & 0xFE);         // Reset CC1000
  WriteToCC1000Register(CC1000_MAIN,MainValue | 0x01);         // Bring CC1000 out of reset
  //WriteToCC1000Register(CC1000_MAIN, 0x3A);
  //WriteToCC1000Register(CC1000_MAIN, 0x3B);
}


/****************************************************************************/
/*  This routine calibrates the CC1000                                      */
/*  Returns 0 if calibration fails, non-zero otherwise. Checks the LOCK     */
/*  to check for success.                                                   */
/****************************************************************************/

char CalibrateCC1000(void)
{  
  int TimeOutCounter;
  uint8_t result = 0;

  WriteToCC1000Register(CC1000_PA_POW,0x00); // Turn off PA to avoid spurs
                                             // during calibration in TX mode
  WriteToCC1000Register(CC1000_CAL,0xA6); // Start calibration

  // Wait for calibration complete
  for(TimeOutCounter=CAL_TIMEOUT, result = 0; !result && TimeOutCounter>0; TimeOutCounter--)
   	result = ReadFromCC1000Register(CC1000_CAL)&0x08;


  // Wait for lock
  for(TimeOutCounter=LOCK_TIMEOUT, result = 0; !result && TimeOutCounter>0; TimeOutCounter--)
  	result = ReadFromCC1000Register(CC1000_LOCK)&0x01;
  
  WriteToCC1000Register(CC1000_CAL,0x26); /* End calibration */
  WriteToCC1000Register(CC1000_PA_POW,PA_VALUE); /* Restore PA setting */

  return ((ReadFromCC1000Register(CC1000_LOCK)&0x01)==1);
}

/****************************************************************************/
/*  This routine puts the CC1000 into RX mode (from TX). When switching to  */
/*  RX from PD, use WakeupC1000ToRX first                                   */
/****************************************************************************/


char SetupCC1000RX(char RXCurrent)
{
  int i;
  char lock_status;

  WriteToCC1000Register(CC1000_MAIN,0x11);    // Switch into RX, switch to freq. reg A
  WriteToCC1000Register(CC1000_PLL,RX_PLL);   // Use RX refdiv setting
  WriteToCC1000Register(CC1000_CURRENT, RXCurrent); // Program VCO current for RX

  // Wait 250us before monitoring LOCK
  _delay_us(250);

  // Wait for lock
  for(i=LOCK_TIMEOUT; ((ReadFromCC1000Register(CC1000_LOCK)&0x01)==0)&&(i>0); i--);

  // If PLL in lock
  if ((ReadFromCC1000Register(CC1000_LOCK)&0x01)==0x01){
    // Indicate PLL in LOCK
    lock_status = LOCK_OK;
  // Else (PLL out of LOCK)
  }else{
    // If recalibration ok
    if(CalibrateCC1000()){
      // Indicate PLL in LOCK
      lock_status = LOCK_RECAL_OK;
    // Else (recalibration failed)
    }else{
      // Reset frequency syncthesizer (ref.: Errata Note 01)
      ResetFreqSynth();
      // Indicate PLL out of LOCK
      lock_status = LOCK_NOK;
    }
  }

  // Return LOCK status to application
  return (lock_status);
}

/****************************************************************************/
/*  This routine puts the CC1000 into TX mode (from RX). When switching to  */
/*  TX from PD, use WakeupCC1000ToTX first                                  */
/****************************************************************************/

char SetupCC1000TX(char TXCurrent)
{
  int i;
  char lock_status;

  WriteToCC1000Register(CC1000_PA_POW,0x00);   // Turn off PA to avoid frequency splatter

  WriteToCC1000Register(CC1000_MAIN,0xE1);    // Switch into TX, switch to freq. reg B
  WriteToCC1000Register(CC1000_PLL,TX_PLL);   // Use TX refdiv setting
  WriteToCC1000Register(CC1000_CURRENT,TXCurrent); // Program VCO current for TX

  // Wait 250us before monitoring LOCK
  _delay_us(250);

  // Wait for lock
  for(i=LOCK_TIMEOUT; ((ReadFromCC1000Register(CC1000_LOCK)&0x01)==0)&&(i>0); i--);

  // If PLL in lock
  if ((ReadFromCC1000Register(CC1000_LOCK)&0x01)==0x01){
    // Indicate PLL in LOCK
    lock_status = LOCK_OK;
  // Else (PLL out of LOCK)
  }else{
    // If recalibration ok
    if(CalibrateCC1000()){
      // Indicate PLL in LOCK
      lock_status = LOCK_RECAL_OK;
    // Else (recalibration failed)
    }else{
      // Reset frequency syncthesizer (ref.: Errata Note 01)
      ResetFreqSynth();
      // Indicate PLL out of LOCK
      lock_status = LOCK_NOK;
    }
  }

  // Increase output power
  WriteToCC1000Register(CC1000_PA_POW,PA_VALUE); // Restore PA setting
  
  // Return LOCK status to application
  return (lock_status);
}

/****************************************************************************/
/*  This routine puts the CC1000 into power down mode. Use WakeUpCC1000ToRX */
/*  followed by SetupCC1000RX or WakeupCC1000ToTX followed by SetupCC1000TX */
/*  to wake up from power down                                              */
/****************************************************************************/

void SetupCC1000PD(void)
{
  WriteToCC1000Register(CC1000_MAIN,0x3F);    // Put CC1000 into power-down
  WriteToCC1000Register(CC1000_PA_POW,0x00);  // Turn off PA to minimise current draw
}

/****************************************************************************/
/*  This routine wakes the CC1000 up from PD mode to RX mode, call          */
/*  SetupCC1000RX after this routine is finished.                           */
/****************************************************************************/

void WakeUpCC1000ToRX(char RXCurrent)
{
  WriteToCC1000Register(CC1000_MAIN,0x3B);  // Turn on xtal oscillator core
  WriteToCC1000Register(CC1000_CURRENT,RXCurrent); // Program VCO current for RX 
  WriteToCC1000Register(CC1000_PLL,RX_PLL); // Use RX refdiv setting
  
  _delay_us(5);
  WriteToCC1000Register(CC1000_MAIN,0x39);  // Turn on bias generator
  
  _delay_us(250);
  WriteToCC1000Register(CC1000_MAIN,0x31);  // Turn on frequency synthesiser
}

/****************************************************************************/
/*  This routine wakes the CC1000 up from PD mode to TX mode, call          */
/*  SetupCC1000TX after this routine is finished.                           */
/****************************************************************************/

void WakeUpCC1000ToTX(char TXCurrent)
{
  WriteToCC1000Register(CC1000_MAIN,0xFB);  // Turn on xtal oscillator core
  WriteToCC1000Register(CC1000_CURRENT,TXCurrent); // Program VCO current for TX
  WriteToCC1000Register(CC1000_PLL,TX_PLL); // Use TX refdiv setting

  //_delay_us(5);  
  WriteToCC1000Register(CC1000_MAIN,0xF9);  // Turn on bias generator


  //_delay_us(250);
  WriteToCC1000Register(CC1000_PA_POW,PA_VALUE); // Turn on PA
  WriteToCC1000Register(CC1000_MAIN,0xF1);  // Turn on frequency synthesiser
}

/****************************************************************************/
/*  This routine locks the averaging filter of the CC1000                   */
/****************************************************************************/

void AverageManualLockCC1000(void)
{
  WriteToCC1000Register(CC1000_MODEM1,0x19);
}

/****************************************************************************/
/*  This routine unlocks the averaging filter of the CC1000                 */
/****************************************************************************/

void AverageFreeRunCC1000(void)
{
  WriteToCC1000Register(CC1000_MODEM1,0x09);
}

/****************************************************************************/
/*  This routine sets up the averaging filter of the CC1000 for automatic   */
/*  lock. This can be used in polled receivers.                             */
/****************************************************************************/

void AverageAutoLockCC1000(void)
{
  WriteToCC1000Register(CC1000_MODEM1,0x01);
}

/****************************************************************************/
/*  This routine reads the current calibration values from the CC1000       */
/****************************************************************************/

void ReadCurrentCalibration(char *val1, char *val2)
{
  *val1=ReadFromCC1000Register(CC1000_TEST0);
  *val2=ReadFromCC1000Register(CC1000_TEST2);
}

/****************************************************************************/
/*  This routine overrides the current calibration of the CC1000            */
/****************************************************************************/

void OverrideCurrentCalibration(char val1, char val2)
{
  WriteToCC1000Register(CC1000_TEST5,(val1&0x0F)|0x10);
  WriteToCC1000Register(CC1000_TEST6,(val2&0x1F)|0x20);
}

/****************************************************************************/
/*  This routine stops override of the CC1000 calibration values            */
/****************************************************************************/

void StopOverridingCalibration(void)
{
  WriteToCC1000Register(CC1000_TEST5,0x00);
  WriteToCC1000Register(CC1000_TEST6,0x00);
}
  

/****************************************************************************/
/*  This CC1000 frequency synthesizer                                       */
/****************************************************************************/
void ResetFreqSynth(void)
{
  char modem1_value;
  modem1_value = ReadFromCC1000Register(CC1000_MODEM1)&~0x01;
  WriteToCC1000Register(CC1000_MODEM1,modem1_value);
  WriteToCC1000Register(CC1000_MODEM1,modem1_value|0x01);
}
