/****************************************************************************/
/* Application note AN009                                                   */
/* CC1000 MCU Interfacing                                                   */
/*                                                                          */
/* File:    cc1000.h                                                        */
/*                                                                          */
/* Contains definitions of all CC1000 registers                             */
/*                                                                          */
/* Author:  Karl H. Torvmark, Chipcon Field Applications Engineer           */
/*                                                                          */
/* Contact: Chipcon AS +47 22 95 85 44                                      */
/*          wireless@chipcon.com                                            */
/****************************************************************************/

/*                                                                           *
 * Revision history:                                                         *
 *                                                                           *
 * $Log: cc1000.h,v $
 * Revision 2.3  2003/05/08 10:05:25  tos
 * Corrections according to Errata Note 01: reset freq.synth if unable to lock PLL.
 *
 * Revision 2.2  2003/04/28 08:21:13  tos
 * Corrected inconsistent monitoring of CC1000: [calibration complete] + [lock].
 *
 *
 *                                                                           *
 ****************************************************************************/

/* Constants defined for CC1000 */
/* Register addresses */

#include <inttypes.h>

/* Macro for concatenating register address and its value */
#define CC1000_REGVAL(a, d)	   (((a & 0x7F) << 8) | (d & 0xFF))

/* Contents of CURRENT register for TX and RX, use SmartRF(R) Studio */
/* to find values for your application */   
 
#define TX_CURRENT 0x81
#define RX_CURRENT 0x44


#define CC1000_MAIN            0x00
#define CC1000_FREQ_2A         0x01
#define CC1000_FREQ_1A         0x02
#define CC1000_FREQ_0A         0x03
#define CC1000_FREQ_2B         0x04
#define CC1000_FREQ_1B         0x05
#define CC1000_FREQ_0B         0x06
#define CC1000_FSEP1           0x07
#define CC1000_FSEP0           0x08
#define CC1000_CURRENT         0x09
#define CC1000_FRONT_END       0x0A
#define CC1000_PA_POW          0x0B
#define CC1000_PLL             0x0C
#define CC1000_LOCK            0x0D
#define CC1000_CAL             0x0E
#define CC1000_MODEM2          0x0F
#define CC1000_MODEM1          0x10
#define CC1000_MODEM0          0x11
#define CC1000_MATCH           0x12
#define CC1000_FSCTRL          0x13
#define CC1000_FSHAPE7         0x14
#define CC1000_FSHAPE6         0x15
#define CC1000_FSHAPE5         0x16
#define CC1000_FSHAPE4         0x17
#define CC1000_FSHAPE3         0x18
#define CC1000_FSHAPE2         0x19
#define CC1000_FSHAPE1         0x1A
#define CC1000_FSDELAY         0x1B
#define CC1000_PRESCALER       0x1C
#define CC1000_TEST6           0x40
#define CC1000_TEST5           0x41
#define CC1000_TEST4           0x42
#define CC1000_TEST3           0x43
#define CC1000_TEST2           0x44
#define CC1000_TEST1           0x45
#define CC1000_TEST0           0x46

#define  LOCK_NOK         0x00
#define  LOCK_OK          0x01
#define  LOCK_RECAL_OK    0x02

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

void ConfigureCC1000(char Count, short Configuration[]);
void WriteToCC1000Register(char addr, char data);
void WriteToCC1000RegisterWord(short addranddata);
uint8_t ReadFromCC1000Register(char addr);
void ResetCC1000(void);
char CalibrateCC1000(void);
char SetupCC1000RX(char RXCurrent);
char SetupCC1000TX(char TXCurrent);
void SetupCC1000PD(void);
void WakeUpCC1000ToRX(char RXCurrent);
void WakeUpCC1000ToTX(char TXCurrent);
void AverageManualLockCC1000(void);
void AverageFreeRunCC1000(void);
void AverageAutoLockCC1000(void);
void ReadCurrentCalibration(char *val1, char *val2);
void OverrideCurrentCalibration(char val1, char val2);
void StopOverridingCalibration(void);
void ResetFreqSynth(void);
