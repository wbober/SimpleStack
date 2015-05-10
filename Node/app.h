#ifndef _APP_H_
#define _APP_H_

#include "event.h"
#include <inttypes.h>

#define APP_MASTER_NID          1

#define APP_CFG_FLAGS           1

#define APP_RECONNECT_TIME      10000
#define APP_COMMAND_TIMEOUT     5       // in minutes
#define APP_REPORT_TIME         10000

/*!
 */
typedef enum _app_flags {
    APP_REGISTERED,
    APP_TOUCHED,
    APP_REPORT,
    APP_RELAY
} app_flags_t;


/*
typedef enum _app_state {
    APP_NREG,
    APP_WFREGACK,
    APP_WFCMD,
    APP_WFCMDACK
} app_state_t;
*/

void app_init(pcfg_t);
void app_radio_txc(ptr_t);
void app_radio_rxc(ptr_t);
void app_usart_txc(ptr_t);
void app_usart_rxc(ptr_t);


#endif
