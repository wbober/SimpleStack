#ifndef CC1000_DEFS
#define CC1000_DEFS

/* Contents of CURRENT register for TX and RX, use SmartRF(R) Studio */
/* to find values for your application */   
 
#define TX_CURRENT 0x81
#define RX_CURRENT 0x44
#define TX_PLL     0x48
#define RX_PLL     0x70
#define PA_VALUE   0xFF

/**
    Bit definitions for MAIN register
 */
#define RXTX                    7
#define F_REG                   6
#define RX_PD                   5
#define TX_PD                   4
#define FS_PD                   3
#define CORE_PD                 2
#define BIAS_PD                 1
#define RESET_N                 0

#define MAIN_PD                 (_BV(RX_PD) | _BV(TX_PD) | _BV(FS_PD) | _BV(CORE_PD) | _BV(BIAS_PD) | _BV(RESET_N))
#define MAIN_RX                 (_BV(TX_PD) | _BV(RESET_N))     
#define MAIN_TX                 (_BV(RXTX) | _BV(F_REG) | _BV(RX_PD) | _BV(RESET_N))

/**
    Bit definitions for CALIBRATION register.
 */
#define CAL_START               7
#define CAL_DUAL                6
#define CAL_WAIT                5
#define CAL_CURRENT             4
#define CAL_COMPLETE            3
#define CAL_ITERATE2            2
#define CAL_ITERATE1            1
#define CAL_ITERATE0            0

/**
    Bit definitions for PLL register
 */
#define LOCK_SELECT3            7
#define LOCK_SELECT2            6
#define LOCK_SELECT1            5
#define LOCK_SELECT0            4
#define PLL_LOCK_ACCURACY       3
#define PLL_LOCK_LENGTH         2
#define PLL_LOCK_INSTANT        1
#define PLL_LOCK_CONTINOUS      0

/**
    CC1000 registers definitions
 */
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

#endif
