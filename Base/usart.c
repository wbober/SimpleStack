#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/wdt.h>
#include <string.h>
#include "usart.h"
#include "defines.h"
#include "event.h"
#include "timers.h"

static volatile usart_ctx_t usart_ctx[] =
{
    {
        .u_id = 0,
        .u_stream = NULL,
        .u_mode = USART_IDLE,
        .UDR   = (uint8_t *)&UDR0,
        .UCSRA = (uint8_t *)&UCSR0A,
        .UCSRB = (uint8_t *)&UCSR0B,
        .UCSRC = (uint8_t *)&UCSR0C,
        .UBRRL = (uint8_t *)&UBRR0L,
        .UBRRH = (uint8_t *)&UBRR0H
    }, 
    {
        .u_id = 1,
        .u_stream = NULL,
        .u_mode = USART_IDLE,
        .UDR   = (uint8_t *)&UDR1,
        .UCSRA = (uint8_t *)&UCSR1A,
        .UCSRB = (uint8_t *)&UCSR1B,
        .UCSRC = (uint8_t *)&UCSR1C,
        .UBRRL = (uint8_t *)&UBRR1L,
        .UBRRH = (uint8_t *)&UBRR1H
    }
};

ISR(USART0_RX_vect)
{
    usart_rx(USART_PHONE);
}

ISR(USART1_RX_vect)
{
    usart_rx(USART_TERMINAL);
}

ISR(USART0_UDRE_vect)
{
    usart_tx(USART_PHONE);
}

ISR(USART1_UDRE_vect)
{
    usart_tx(USART_TERMINAL);
}

/**
    \brief Set USART mode.
 */
uint8_t usart_mode(pusart_ctx_t u, uint8_t mode) 
{
    if (u->u_mode == mode)
        return 0;
            
    switch (mode) {
        case USART_IDLE:
            // disable transmission
            *u->UCSRB &= ~_BV(UDRIE);
            // enable receiver and receive interrupt
            *u->UCSRB |= _BV(RXCIE);
            break;

        case USART_RX:  
            if (u->u_mode != USART_IDLE)
                return 0;
            // disable transmission
            *u->UCSRB &= ~_BV(UDRIE);
            // enable receiver and receive interrupt
            *u->UCSRB |= _BV(RXCIE);
            break;

        case USART_TX:
            if (u->u_mode != USART_IDLE)
                return 0;
            // disable receiver and receive interrupt
            *u->UCSRB &= ~_BV(RXCIE);
            // enable transmission
            *u->UCSRB |= _BV(UDRIE);
            break;

        case USART_LOCK:
            // disable receiver and receive interrupt
            *u->UCSRB &= ~_BV(RXCIE);
            // disable transmission
            *u->UCSRB &= ~_BV(UDRIE);
            break;
    }

    u->u_mode = mode;
    return 1;
}

/**
    \brief Read number of bytes from given usart's circular buffer

    The invoker is responsible for providing sufficient space in the 
    buffer. If requested number of bytes is larger then number of bytes
    present in the buffer function fails.  
 
    \param u usart context
    \param bp destination buffer 
    \param n number of bytes to read
 */
uint8_t usart_read(pusart_ctx_t u, volatile uint8_t *bp, uint8_t n) 
{
    // check if sufficient number of characters in the buffer
    if (n <= 0 || !u->u_count || u->u_count < n)
        return 0;

    // subtract number of characters to be read
    u->u_count -= n;

    while (n--) {
        *bp++ = u->u_buffer[u->u_head++];
        //u->u_head = u->u_head < sizeof(u->u_buffer) - 1 ? u->u_head + 1 : 0;
    }

    // reset pointers if buffer is empty
    if (!u->u_count) {
         u->u_head = u->u_tail = 0;
         usart_mode(u, USART_IDLE);
    }

    return 1;
}

/**
    \brief Write number of bytes in given usart's context circular buffer.

    Function will fail if free space in the buffer is smaller then number
    of bytes requested to be written.

    \param u usart context
    \param bp pointer to source buffer
    \param n number of bytes to write from source buffer
 */
uint8_t usart_write(pusart_ctx_t u, volatile uint8_t *bp, uint8_t n) 
{
    // check space in the buffer
    if (n <= 0 || u->u_count + n > USART_BUFFER_SIZE - 1)
        return 0;

    // add number of characters to be written
    u->u_count += n;

    // fill in the buffer
    while (n--) {       
        u->u_buffer[u->u_tail++] = *bp++;
        //u->u_tail = u->u_tail < sizeof(u->u_buffer) - 1 ? u->u_tail + 1 : 0;
    }

    // put a '\0' char at the end of the buffer, so search function can be performed
    u->u_buffer[u->u_tail] = 0;

    return 1;
}

/**
    \brief Look up given key in usart's circural buffer.

    Function fails if number of bytes present in buffer is smaller
    then key lenght.

    \param u usart context
    \param key pointer to key stored in program memory
 */
int8_t usart_find(pusart_ctx_t u, PGM_P key) 
{
    char *p = NULL;
    uint8_t i, size;

    size = (uint8_t) strlen_P(key);
    if (u->u_count < size)
        return -1;
    
    p = strstr_P((char *)u->u_buffer + u->u_head, key);
    if (p) {
        i = p - (char *)u->u_buffer;
        if (i + size <= u->u_tail) 
            return i;
    }

    // the function may fail if the key is split in the buffer into two parts
    /*
    if (u->u_head < u->u_tail) {
        p = strstr_P((char *)u->u_buffer + u->u_head, key);
        i = p - (char *)u->u_buffer;
        if (p && (i >= u->u_head) && (i + size <= u->u_tail)) {
            return 1;
        }

    } else {
        p = strstr_P((char *)u->u_buffer, key);
        i = p - (char *)u->u_buffer;
        if (p && ((i >= u->u_head) || (i + size <= u->u_tail)))
            return 1;
    } 
    */  

    return -1; 
}

/**
    Insert character to buffer
 */
int usart_put(char c, FILE *f)
{
    pusart_ctx_t u = (f == usart_ctx[0].u_stream) ? (pusart_ctx_t)usart_ctx : (pusart_ctx_t)usart_ctx + 1;
   
    uint8_t wait = USART_WAIT_COUNT;
    wdt_reset();
    do {
        *u->UCSRB &= ~_BV(RXCIE);
        switch (u->u_mode) {
            case USART_IDLE:
                usart_write(u, (uint8_t *)&c, 1);
                return 0;

            case USART_TX:
                *u->UCSRB &= ~_BV(UDRIE);
                if (u->u_count < USART_BUFFER_SIZE) {
                    usart_write(u, (uint8_t *)&c, 1);
                    *u->UCSRB |= _BV(UDRIE);
                    return 0;
                }
                *u->UCSRB |= _BV(UDRIE);
                break;

            case USART_LOCK:
                return 1;

            default:
                *u->UCSRB |= _BV(RXCIE);                                
                break;
        }
        _delay_ms(10);
    } while (wait--);
    return 0;
}

/**
    \brief USART TX interrupt handler
    
    \param usart_id USART port number
 */
void usart_tx(uint8_t usart_id) 
{
    pusart_ctx_t u = (pusart_ctx_t)&usart_ctx[usart_id];

    if (!u->u_count) {
        event_activate(EVENT_USART_TXC, NULL);
    } else {
        uint8_t data = 0;
        usart_read(u, &data, 1);
        if (usart_id == USART_TERMINAL)
            UDR1 = data;
        else
            UDR0 = data;
    }
}

/**
    \brief USART RX interrupt handler

    \param usart_id USART port number
 */
void usart_rx(uint8_t u_id)
{
    pusart_ctx_t u = (pusart_ctx_t)&usart_ctx[u_id];
    uint8_t c = (u_id == USART_TERMINAL) ? UDR1 : UDR0;

    if (u->u_mode == USART_IDLE)
        usart_mode(u, USART_RX);
    else if (u->u_mode == USART_LOCK)
        event_activate(EVENT_USART_RXC, (ptr_t)u);

    if (u->u_mode == USART_RX && usart_write(u, &c, 1)) {
        timer_enable(u->u_timer);
    }
}

/**
    \brief Skip given number of bytes in usart buffer.

    \param u usart context in where bytes should be skipped
    \param count number of bytes to skip
 */
void usart_skip(pusart_ctx_t u, uint8_t count)
{
    u->u_head += count;
    if (u->u_head >= USART_BUFFER_SIZE)
       u->u_head -= USART_BUFFER_SIZE;
    u->u_count -= count;            

    // reset indexes if buffer empty
    if (!u->u_count) {
        u->u_head = u->u_tail = 0;
        usart_mode(u, USART_IDLE);
    }
        
}

/**
 */
void usart_timeout(uint8_t id)
{
    pusart_ctx_t u = (pusart_ctx_t)timer_get_pdata(id);
    usart_mode(u, USART_LOCK);
    event_activate(EVENT_USART_RXC, (ptr_t)u);
}

/**
    Initialize USART context
 */
void usart_init_ctx(pusart_ctx_t u, uint16_t baud, uint16_t timeout)
{
    u->u_head = u->u_tail = 0;
    u->u_stream = fdevopen(usart_put, NULL);
    u->u_timer = timer_add(usart_timeout);
    timer_set(u->u_timer, timeout, 0, (ptr_t)u);
    
    *u->UBRRL = baud;
    *u->UBRRH = baud >> 8;
    *u->UCSRB = _BV(RXCIE) | _BV(RXEN) | _BV(TXEN);
    *u->UCSRC = _BV(UCSZ1) | _BV(UCSZ0);
}

/**
    Initalize USARTs
 */
void usart_init(void)
{
    usart_init_ctx(&usart_ctx[USART_TERMINAL], USART_BAUD_SELECT(115200), 200);
    usart_init_ctx(&usart_ctx[USART_PHONE], USART_BAUD_SELECT(115200), 500);

    stdout = usart_ctx[USART_TERMINAL].u_stream;
    stderr = stdout;
}


/**
    Print debug info to terminal.
 */
void usart_printf(PGM_P fmt, ...)
{
    va_list ap;

    #ifdef DBG_TSTAMP
    fprintf_P(usart_ctx[USART_TERMINAL].u_stream, PSTR("%06ld.%03d "), clock.s, clock.ms);
    #endif

    va_start(ap, fmt);
    vfprintf_P(usart_ctx[USART_TERMINAL].u_stream, fmt, ap);
    va_end(ap);
    usart_mode(&usart_ctx[USART_TERMINAL], USART_TX);
}


void usart_printf2(uint8_t id, PGM_P fmt, ...) 
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf_P(usart_ctx[id].u_stream, fmt, ap);
    va_end(ap);
    usart_mode(&usart_ctx[id], USART_TX);
}

pusart_ctx_t usart_get_ctx(uint8_t id)
{
    return usart_ctx + id;
}
