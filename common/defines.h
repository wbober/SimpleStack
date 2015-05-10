#ifndef _PORTS_H
#define _PORTS_H

#include <avr/io.h>
#include <stdbool.h>

/** Pointer type definition */
typedef void * ptr_t;

/** Configuration set definition */
typedef struct {
    uint8_t cfg_nid_addr;
    uint8_t cfg_rst_cnt;
    int8_t  cfg_temp_cal;
    uint8_t cfg_relays;
} cfg_t, *pcfg_t;

#define PGM_S(k,v) const char k[] PROGMEM = v
#define CFG_NID 0

#ifdef BASE
    #ifdef DEBUG_MAC
       #define DBG_MAC(f)(f)
    #else
       #define DBG_MAC(f)({})
    #endif
#else
    #define DBG_MAC(f)({})
#endif


#ifdef BASE
    #ifdef DEBUG_EVENT
       #define DBG_EVT(f)(f)
    #else
       #define DBG_EVT(f)({})
    #endif
#else
    #define DBG_EVT(f)({})
#endif

#ifdef BASE
    #ifdef DEBUG_APP
       #define DBG_APP(f)(f)
    #else
       #define DBG_APP(f)({})
    #endif
#else
    #define DBG_APP(f)({})
#endif

#ifdef BASE
    #ifdef DEBUG_LINK
       #define DBG_LINK(f)({f})
    #else
       #define DBG_LINK(f)({})
    #endif
#else
    #define DBG_LINK(f)({})
#endif

#ifdef BASE
    #ifdef DEBUG_NET
       #define DBG_NET(f)(f)
    #else
       #define DBG_NET(f)({})
    #endif
#else
    #define DBG_NET(f)({})
#endif

#ifdef BASE
    #ifdef DEBUG_PHONE
       #define DBG_PHN(f)(f)
    #else
       #define DBG_PHN(f)({})
    #endif
#else
    #define DBG_PHN(f)({})
#endif


#ifdef BASE
    #include "../Base/defines.h"
#else
    #include "../Node/defines.h"
#endif

#endif
