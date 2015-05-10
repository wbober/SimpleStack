#ifndef _LINK_H_
#define _LINK_H_

#include "defines.h"
#include "network.h"
#include <inttypes.h>
#include <stdlib.h>


/**
    \defgroup link <link.h> Link layer routines
 */
#define LINK_RX_QUEUE_SIZE      10
#define LINK_TX_QUEUE_SIZE      10
#define LINK_FRAME_LOAD_SIZE    sizeof(net_packet_t)

/*!
 */
typedef enum _link_error_code_t {
    LINK_ERROR_MAC,
} link_error_code_t;


/*!
 */
typedef enum _link_frame_type_t {
    LINK_DTA,
    LINK_ACK
} link_frame_type_t;

/*!
 */
typedef struct {
        uint16_t f_seq;
        link_frame_type_t f_flags;
        uint8_t f_dest_addr;
        uint8_t f_src_addr;
        uint8_t f_size;
} link_frame_header_t, *plink_frame_header_t;

/** \ingroup link
	\brief Link layer frame.

    A physical frame transmited between adjacent nodes.
 */
typedef struct {
    link_frame_header_t f_header;           /**< Frame header */
	uint8_t f_load[LINK_FRAME_LOAD_SIZE];   /**< Frame load */
	uint16_t  f_crc;                        /**< Frame CRC16. This is an xmodem CRC16 value calculated from header and load. */
} link_frame_t, *plink_frame_t;


/*!
typedef struct {
    link_error_code_t e_code;
    uint8_t e_seq;
    uint8_t e_dest_addr;
    uint8_t e_src_addr;
} link_error_t, *plink_error_t;
 */

#define link_frame_size(pf) (sizeof(link_frame_header_t) + pf->f_header.f_size + 2)

/** \ingroup link
 */
void link_init(pcfg_t);

/** \ingroup link
 */
uint16_t link_send(uint8_t addr, uint8_t *data, uint8_t size);

/** \ingroup link
 */
uint8_t link_receive(uint8_t *addr, uint8_t *data, uint8_t size);

/** \ingroup link
 */
uint8_t link_txc(ptr_t);

/** \ingroup link
 */
uint8_t link_rxc(plink_frame_t);

/** \ingroup link
 */
uint8_t link_fail(ptr_t);


plink_frame_t link_frame_alloc(link_frame_type_t type);
plink_frame_t link_frame_copy(plink_frame_t dest, plink_frame_t src);
plink_frame_t link_frame_free(plink_frame_t *f);

#endif
