#ifndef _APP_H_
#define _APP_H_

#include <inttypes.h>
#include <stdbool.h>
#include "event.h"
#include "list.h"
#include "cmd.h"
#include "pool.h"


#define APP_CFG_FLAGS           1
#define APP_NODE_TIMEOUT        20	 // 10 minutes
#define APP_TEMP_SAMPLE_TIME    2000
#define APP_REPORT_TIME         60000
#define APP_CMD_TIMEOUT         2000
#define APP_QUERY_PERIOD        30000
#define APP_QUERY_DURATION      5000

#define APP_RESET_PERIOD        1440 // 24hrs

#define APP_MAX_NODE            4
#define APP_MAX_CMD             5

#define APP_MASTER_NID          1

/*!
    Application flags.
 */
typedef enum {
    APP_RESET_QUERY = 1,
    APP_REPORT_STATUS = 5
} app_flags_t;

/*!
    Source of command.
 */
enum _commands_t {
    CMD_LOCAL,
    CMD_REMOTE
};

/*!
    Application state.
 */
typedef enum _app_state_t {
    APP_IDLE,
    APP_WAIT,
    APP_DONE
} app_state_t;

/*!
    Node definition.
 */
typedef struct {
    uint8_t n_present;
    uint8_t n_relays;
    float   n_temp;
    int16_t n_rssi;
    struct {
        uint16_t d_sent;
        uint16_t d_recv;
    } n_debug;
} node_t, *pnode_t;

/*!
    \brief Command queue entry.
 */
typedef struct {
    uint8_t cmd_result;
    uint8_t cmd_type;
    cmd_t   cmd;
} cmd_entry_t, *pcmd_entry_t;

/*
    Application context definition.
 */
typedef struct {
    app_state_t a_state;
    uint8_t  a_flags;
    uint8_t  a_node_index;

    uint8_t a_nodes_count;
    node_t  a_nodes[APP_MAX_NODE];
    plist_t a_cmd_queue;

    uint8_t a_timer;
    pcfg_t a_pcfg;

} app_ctx_t;


void app_init(pcfg_t);
void app_radio_txc(ptr_t);
void app_radio_rxc(ptr_t);
void app_usart_txc(ptr_t);
void app_usart_rxc(ptr_t);

uint8_t app_s2n(uint8_t *s);
uint8_t app_sh2n(uint8_t *s);
void app_n2sh(char *s, uint8_t n);

uint8_t app_cmd_parse(uint8_t *bp, pcmd_t pcmd);
#endif
