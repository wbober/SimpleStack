#ifndef _USART_H_
#define _USART_H_

#include <avr/pgmspace.h>
#include <stdarg.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>


#define USART_BUFFER_SIZE   50
#define USART_WAIT_COUNT    100


#define USART_OK            1
#define USART_ERROR         0

#define USART_PHONE         0
#define USART_TERMINAL      1

typedef enum {
    USART_RX,      // receive mode
    USART_TX,      // transmit mode
    USART_LOCK,     // lock mode,
    USART_IDLE
} usart_mode_t;

typedef struct {
    char    u_buffer[USART_BUFFER_SIZE];     // internal buffer
    uint8_t u_head, u_tail, u_count;         // current position in buffer pointer 
    uint8_t u_id;
    FILE    *u_stream;
    uint8_t u_timer;

    usart_mode_t u_mode;

    uint8_t *UDR;                           // pointer to UDR register
    uint8_t *UCSRA;                         // pointer to UCSRB register
    uint8_t *UCSRB;
    uint8_t *UCSRC;
    uint8_t *UBRRL;
    uint8_t *UBRRH;
} usart_ctx_t, *pusart_ctx_t;

#define USART_BAUD_SELECT(baudRate) ((F_CPU)/((baudRate)*16L)-1)

void usart_init(void);
void usart_tx(uint8_t);
void usart_rx(uint8_t);

uint8_t usart_mode(pusart_ctx_t, usart_mode_t);
uint8_t usart_write(pusart_ctx_t u, volatile uint8_t *bp, uint8_t n);
uint8_t usart_read(pusart_ctx_t u, volatile uint8_t *bp, uint8_t n);

pusart_ctx_t usart_get_ctx(uint8_t id);

int8_t usart_find(pusart_ctx_t u, PGM_P key);
void usart_skip(pusart_ctx_t, uint8_t);

void usart_printf(PGM_P fmt, ...);
void usart_printf2(uint8_t id, PGM_P fmt, ...);

#endif
