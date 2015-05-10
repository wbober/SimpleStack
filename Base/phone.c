#include "phone.h"
#include "usart.h"
#include "debug.h"
#include "event.h"
#include "timers.h"

typedef enum {
    PHONE_IDLE,
    PHONE_CMGR,
    PHONE_CMGS,
    PHONE_PDU,
    PHONE_CMGD,
    PHONE_RESET,
    PHONE_RESET_DONE,
    PHONE_PROMPT,
    PHONE_CHECK
} phone_state_t;

typedef struct {
    phone_state_t p_state;
    uint8_t p_index;
    uint8_t *p_pdu;
    uint8_t *p_address;
    uint8_t p_lenght;
    uint8_t p_timer;
    uint8_t p_watchdog_timer;
} phone_ctx_t;

static volatile phone_ctx_t phone_ctx;

/**
    \brief Encode PDU message.
 */
uint8_t phone_sms_encode(char **buffer, uint8_t *cnt_len, char *msg, char *address)
{
    uint8_t i, shift, n;
    char *bp, *mp = msg;
    size_t lenght = strlen(msg);
    uint8_t enc_len = lenght - (lenght / 8);
    size_t pdu_lenght = 14 + strlen(address) + enc_len*2 + 1;

    *cnt_len = (strlen(address) >> 1);
    *cnt_len += 6 + enc_len; 

    // free if already allocated
    if (*buffer) {
        free(*buffer);
    }

    *buffer = (char *)malloc(pdu_lenght);
    // if memory allocation failed
    if (!*buffer) {
        DBG_PHN(usart_printf(PSTR("PHN pdu alloc failed\r\n")));
        return 0;
    }

    DBG_PHN(usart_printf(PSTR("PHN pdu alloc p:%p size:%d\r\n"), *buffer, pdu_lenght));

    bp = *buffer;

    // set PDU header
    strcpy_P(bp, PSTR("001100"));
    bp += 6;

    // copy address
    strcpy(bp, address);
    bp += strlen(address);

    // set TP-PID, TP-DCS, TP-Validity-Period
    strcpy_P(bp, PSTR("0000AA"));
    bp += 6;

    // copy user data lenght
    app_n2sh(bp, lenght);
    bp += 2;

    // encode message
    shift = 7;
    for (i = 0; i < lenght; i++) {
        n = *mp >> (7 - shift);
        n |= (*(mp + 1) << shift);

        mp++;
        app_n2sh(bp, n);
        bp += 2;
      
        if (shift == 1) {
            shift = 7;
            mp++;
        } else
            shift--;
    }
    *bp = 0;  

    DBG_PHN(usart_printf(PSTR("PHN msg '%s' encoded\r\n"), msg));    
    return 1;  
}

/**
    \brief Decode PDU message.
 */
uint8_t phone_sms_decode(uint8_t *buffer, uint8_t **sender)
{
    uint8_t i, shift, n;
    uint8_t *bp = buffer, *dp = buffer;
    size_t lenght;

    // read SMSC info lenght
    sscanf((char *)bp, "%2x", &lenght);
    bp += 4 + (lenght << 1);

    // read sender address lenght
    sscanf((char *)bp, "%2x", &lenght);

    // if address is odd then it has additional character
    if (lenght % 2)
      lenght++;

    lenght += 4;
    //bp += 2;

    *sender = (uint8_t *)malloc(lenght + 1);
    if (*sender) {
        // copy sender number
        memcpy(*sender, bp, lenght);
        // add trailing zero
        (*sender)[lenght] = 0;
        // debug
        DBG_PHN(usart_printf(PSTR("PHN alloc p:%p sender info '%s' size:%d\r\n"), *sender, *sender, lenght+1));
    }
               
    bp += 18 + lenght;

    // read msg lenght
    sscanf((char *)bp, "%2x", &lenght);
    bp += 2;

    // decode from octets to septets
    dp = bp;
    bp = buffer;
    *bp = 0;
    shift = 7;

    // decode content
    for (i = 0; i < lenght; i++) {
      n = app_sh2n(dp);
      dp += 2;
      *bp |= (n << (7 - shift)) & 0x7f;
      
      bp += 1;
      *bp = n >> shift;

      if (shift == 1) {
        shift = 7;
        bp += 1;
        *bp = 0;
        i += 1;
      } else
        shift--;
    }

    *bp = 0;

    DBG_PHN(usart_printf(PSTR("PHN decoded msg '%s'\r\n"), buffer));

    return 1;
}

/**
    \brief Handle new message notice
 */
uint8_t phone_parse_cmti(pusart_ctx_t pu)
{
    uint8_t buffer[20], *bp = buffer;
    uint8_t index = 0;

    // lock receiver to prevent data receiving
    // usart_mode(pu, USART_LOCK);

    // check if it will fit in the buffer, if not discard it
    uint8_t count = pu->u_count;
    if (count > sizeof(buffer)) {
        usart_skip(pu, count);
        return 0;
    }

    // read everything from the buffer
    usart_read(pu, bp, count);

    while (*bp != ',')
        bp++;
    bp += 1;

    // check if index is given by one or two digit number
    if (*(bp + 1) != '\r') {
        index = app_s2n(bp);
    } else {
        index = *bp - 0x30;
    }

    // fetch the message
    if (index > 0 && index <= 20) {
        usart_printf2(pu->u_id, PSTR("AT+CMGR=%d\r"), index);
    } else {
        usart_mode(pu->u_id, USART_RX);
    }

    DBG_PHN(usart_printf(PSTR("PHN SMS notice i:%d\r\n"), index)); 

    phone_ctx.p_index = index;

    return 1;
}

/**
    \breif Handle PDU data
 */
uint8_t phone_parse_cmgr(pusart_ctx_t pu, pcmd_entry_t pce)
{
    uint8_t buffer[USART_BUFFER_SIZE], *bp = buffer;
 
    // check if it will fit in the buffer, if not discard it
    uint8_t count = pu->u_count;
    if (count > sizeof(buffer)) {
        usart_skip(pu, count);
        return 0;
    }

    // read everything from the buffer
    usart_read(pu, buffer, count);


    // move pointer to skip +CMGR: x,,y
    while (*bp != ':')
        bp++;
    bp += 5;

    // check if pdu present
    if (*bp == '0') {
        usart_printf2(pu->u_id, PSTR("AT+CMGD=%d\r"), phone_ctx.p_index);
        phone_ctx.p_state = PHONE_CMGD;
        return 0;
    }
    
    // find the start of pdu
    while (*bp++ != '\r')
        ;

    // decode pdu
    phone_sms_decode(bp + 1, (uint8_t **)&pce->cmd_src_info);
    
    // lookup command start    
    while (*bp != CMD_CMD_KEY && *bp != '\0')
        bp++;

    // if command start found
    if (*bp == CMD_CMD_KEY) {
        if (app_cmd_parse(bp + 1, &pce->cmd)) {
            DBG_PHN(usart_printf(PSTR("PHN cmd present\r\n")));
            pce->cmd_valid = true;
            pce->cmd_local = pce->cmd.c_nid == APP_MASTER_NID ? true : false;
            pce->cmd_src = SRC_REMOTE;
        } else {
            DBG_PHN(usart_printf(PSTR("PHN cmd not present\r\n")));
            DBG_APP(usart_printf(PSTR("PHN free p:%p sender info"), pce->cmd_src_info));
            free(pce->cmd_src_info);
            pce->cmd_src_info = NULL;
        }
    }
    
    // send command to delete sms
    usart_printf2(pu->u_id, PSTR("AT+CMGD=%d\r"), phone_ctx.p_index);
    phone_ctx.p_state = PHONE_CMGD;

    return 1;
}

uint8_t phone_reset_ctx(pusart_ctx_t pu)
{
    DBG_PHN(usart_printf(PSTR("PHN reset context\r\n")));
    usart_skip(pu, pu->u_count);
    phone_ctx.p_state = PHONE_IDLE;
    if (phone_ctx.p_pdu) {
        DBG_PHN(usart_printf(PSTR("PHN free p:%p pdu\r\n"), phone_ctx.p_pdu));
        free(phone_ctx.p_pdu);
        phone_ctx.p_pdu = NULL;
    }
    //if (
    return 1;
}

/*!
 */
uint8_t phone_handle_rx(pusart_ctx_t pu, pcmd_entry_t pce)
{
    int8_t pos = -1;

    switch (phone_ctx.p_state) {
        case PHONE_CHECK:
            if (usart_find(pu, PSTR("OK\r\n")) >= 0) {
                app_lcd_msg(LCD_MODE_TTL, PSTR("GSM OK"));
                DBG_PHN(usart_printf(PSTR("PHN phone present\r\n")));  
                timer_disable(phone_ctx.p_timer);                      
                // context reset will set the state to PHONE_IDLE
                phone_reset_ctx(pu);
            } else {
                phone_ctx.p_state = PHONE_RESET;
                usart_skip(pu, pu->u_count);
            }
            break;

        case PHONE_IDLE:
            if (usart_find(pu, PSTR("+CMTI")) >= 0) {
                phone_parse_cmti(pu);
                phone_ctx.p_state = PHONE_CMGR;
                event_activate(EVENT_PHN_RXC, NULL);
            } else {
                phone_reset_ctx(pu);
            }
            break;

        case PHONE_CMGR:
            if (usart_find(pu, PSTR("+CMGR:")) >= 0) {
                phone_parse_cmgr(pu, pce);
                phone_ctx.p_state = PHONE_CMGD;
                DBG_PHN(usart_printf(PSTR("PHN msg read in\r\n")));        
            } else {
                phone_reset_ctx(pu);
                DBG_PHN(usart_printf(PSTR("PHN failed to read msg\r\n")));        
            }
            break;

        case PHONE_CMGD:
            if (usart_find(pu, PSTR("OK\r\n")) >= 0) {
                //usart_mode(pu, USART_LOCK);
                usart_skip(pu, pu->u_count);                
                if (phone_ctx.p_pdu) {                    
                    usart_printf2(USART_PHONE, PSTR("AT+CMGS=%d\r"), phone_ctx.p_lenght);
                    phone_ctx.p_state = PHONE_CMGS;
                    DBG_PHN(usart_printf(PSTR("PHN msg deleted, sending replay\r\n")));
                } else {
                    DBG_PHN(usart_printf(PSTR("PHN msg deleted\r\n")));
                    phone_reset_ctx(pu);
                }
            } else {
                phone_reset_ctx(pu);
                DBG_PHN(usart_printf(PSTR("PHN failed to delete msg\r\n")));
            }
            break;

         case PHONE_CMGS:
            if (usart_find(pu, PSTR("> ")) >= 0) {
                usart_skip(pu, pu->u_count);
                usart_printf2(pu->u_id, PSTR("%s%c"), phone_ctx.p_pdu, 0x1A);
                phone_ctx.p_state = PHONE_PDU;
            } else {
                DBG_PHN(usart_printf(PSTR("PHN no prompt, failed to send msg\r\n")));
                phone_reset_ctx(pu);
            }
            break; 

        case PHONE_PDU:
            if (usart_find(pu, PSTR("+CMGS")) >= 0) {
                DBG_PHN(usart_printf(PSTR("PHN msg sent\r\n")));
                event_activate(EVENT_PHN_TXC, NULL);               
            } else if (usart_find(pu, PSTR("ERROR"))) {\
                DBG_PHN(usart_printf(PSTR("PHN msg not sent\r\n")));
            }
            phone_reset_ctx(pu);
            break;
    }
    return 0;
}

/*!
 */
uint8_t phone_send_sms(char *msg, char *address)         
{
    phone_sms_encode((char *)&phone_ctx.p_pdu, &phone_ctx.p_lenght, msg, address);
    
    if (phone_ctx.p_state == PHONE_IDLE) {
        usart_printf2(USART_PHONE, PSTR("AT+CMGS=%d\r"), phone_ctx.p_lenght);
        phone_ctx.p_state = PHONE_CMGS;
        DBG_PHN(usart_printf(PSTR("PHN sending msg\r\n")));        
    } else {
        DBG_PHN(usart_printf(PSTR("PHN ctx busy, will try to send later\r\n")));        
    }

    return 1;
}

/*!
 */
void phone_watchdog(uint8_t tid) 
{
    static uint16_t counter = 0;

    switch (phone_ctx.p_state) {
        case PHONE_RESET:
            RELAY_ON(RELAY_1); 
            phone_ctx.p_state = PHONE_RESET_DONE;           
            break;

        case PHONE_RESET_DONE:
            RELAY_OFF(RELAY_1);
            if (++counter == 20) {
                counter = 0;
                phone_ctx.p_state = PHONE_PROMPT;
            }
            break;

        case PHONE_IDLE:
            if (++counter == 3600) {
                counter = 0;
                phone_ctx.p_state = PHONE_PROMPT;
            }
            break;
        
        case PHONE_PROMPT:
            usart_printf2(USART_PHONE, PSTR("ATE0\rAT+CNMI=1,1\r"));    
            phone_ctx.p_state = PHONE_CHECK;        
            break;

        case PHONE_CHECK:
            phone_ctx.p_state = PHONE_RESET;
            break;
    }
}

/*!
 */
void phone_init(pcfg_t cfg)
{
    phone_ctx.p_pdu = NULL;
    phone_ctx.p_state = PHONE_PROMPT;
    phone_ctx.p_watchdog_timer = timer_add(phone_watchdog);
    timer_set(phone_ctx.p_watchdog_timer, 1000, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);
}
