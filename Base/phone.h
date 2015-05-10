#ifndef PHONE_H
#define PHONE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include "usart.h"
#include "app.h"

uint8_t phone_sms_encode(char **buffer, uint8_t* cnt_len, char *msg, char *address);
uint8_t phone_sms_decode(uint8_t *buffer, uint8_t **sender);
uint8_t phone_parse_cmti(pusart_ctx_t pu);
uint8_t phone_parse_cmgr(pusart_ctx_t pu, pcmd_entry_t pce);
uint8_t phone_handle_rx(pusart_ctx_t pu, pcmd_entry_t pce);
uint8_t phone_send_sms(char *msg, char *address);
void phone_init();

#endif
