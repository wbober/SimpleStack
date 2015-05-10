#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stddef.h>
#include "timers.h"
#include "transport.h"
#include "app.h"
#include "usart.h"
#include "lcd.h"
#include "rssi.h"
#include "ow.h"
#include "link.h"
#include "phone.h"
#include "mac.h"
#include "cc1000.h"
#include "pool.h"

#define APP_NODE(n) (app_ctx.a_nodes[n - 1])

PGM_S(cmd_set_state_s, "CMD_SET_STATE");
PGM_S(cmd_get_state_s, "CMD_GET_STATE");
PGM_S(cmd_state_s,     "CMD_STATE");
PGM_P cmd_string[] = {cmd_set_state_s, cmd_get_state_s, cmd_state_s};

void store_cfg(pcfg_t cfg);

/*!
    Application context variable.
 */
static app_ctx_t app_ctx = {
        .a_flags = 0,
        .a_nodes_count = APP_MAX_NODE,
        .a_state = APP_IDLE,
        .a_node_index = 1
        //.a_cmd_queue = LIST_INIT(5),
        //.a_cmd_pool = POOL_INIT(cmd_entry_t, 10),
        //.a_node_queue = LIST_INIT(APP_MAX_NODE),
        //.a_node_pool = POOL_INIT(node_t, APP_MAX_NODE)
    };

/*!
    \brief Send command to remote node

    \param pcmd pointer to command entry
    \return if successfull message sequence number, else 0
 */

uint16_t app_cmd_send(pcmd_entry_t pce)
{
    pcmd_t pcmd = &pce->cmd;
    pce->cmd_result = (uint8_t)tcp_send(pcmd->c_nid, (uint8_t *)pcmd + offsetof(cmd_t, c_id), pcmd->c_argc + 2);
    return pce->cmd_result;
}

/*!
    Insert a command into the command queue.

    \param id command id
    \param nid destination nid
    \param src command source
    \param arg command argument

    \return 1 if successfull, 0 otherwise
 */
uint8_t app_push_cmd(uint8_t id, uint8_t nid, uint8_t type, uint8_t argc, ...)
{
    pcmd_entry_t pce = (pcmd_entry_t)malloc(sizeof(cmd_entry_t));
    va_list ap;
    uint8_t i = 0;

    if (!pce)
        return 0;

    pce->cmd_type = type;
    pce->cmd.c_nid = nid;
    pce->cmd.c_id = id;
    pce->cmd.c_argc = argc;
    
    va_start(ap, argc);
    while (i < argc) {
        pce->cmd.c_argv[i] = va_arg(ap, int);
        i += 1;
    }
    va_end(ap);

    list_push(app_ctx.a_cmd_queue, pce);

    return 1;
}

static inline void app_reset_cycle() {
	app_ctx.a_node_index = 1;
	if (app_ctx.a_state == APP_IDLE) {
		timer_set(app_ctx.a_timer, APP_QUERY_DURATION, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);
	}
}

/*!
 */
uint8_t app_set_all(uint8_t state, bool store_in_eeprom)
{ 
    uint8_t i;

    for (i = 1; i < APP_MAX_NODE; i++) {
        // store requested relay state 
        app_ctx.a_nodes[i].n_relays = (app_ctx.a_nodes[i].n_relays & 0x0f) + (state << 4);
    }

    if (store_in_eeprom) {
        app_ctx.a_pcfg->cfg_relays = state;
        store_cfg(app_ctx.a_pcfg);
		app_reset_cycle();
    }
	
    return 1;
}

void app_report_state() {

    uint8_t i, on_cnt = 0;
    char t[5];

    dtostrf(APP_NODE(APP_MASTER_NID).n_temp, 5, 1, t);

    for (i = 1; i < APP_MAX_NODE; i++) {
        if (app_ctx.a_nodes[i].n_present && (app_ctx.a_nodes[i].n_relays & 0x0f)) {
            on_cnt += 1;
        }
    }

    usart_printf2(USART_PHONE, PSTR("OK TEMP %s ON %d\r\n"), t, on_cnt);
}


/*!
    \brief Execute a command.

    \param cmd pointer to command
 */
uint8_t app_cmd_execute(pcmd_entry_t pce) 
{
    uint8_t result = 0;
    pcmd_t cmd = &(pce->cmd);

	switch (cmd->c_id) {        
        // get node's state
        case CMD_GET_STATE: {
            break;
        }

        // node is reporting state
        case CMD_STATE: {
            if (cmd->c_nid - 1 < APP_MAX_NODE) {
                pnode_t node = &APP_NODE(cmd->c_nid);
                if (node->n_present == 0) {
                    app_ctx.a_nodes_count += 1;
                }
                node->n_present = APP_NODE_TIMEOUT;
                node->n_temp = (cmd->c_argv[1] ? cmd->c_argv[2] : -cmd->c_argv[2]) + (cmd->c_argv[3] ? 0.5 : 0);
                node->n_rssi = mac_debug_info()->d_rssi;
                node->n_relays = (node->n_relays & 0xf0) + cmd->c_argv[0];

                #ifdef DEBUG_APP
                    node->n_debug.d_recv++;
                #endif

                result = 1;
            }
            break;
        }

        // set app flags
        case CMD_SET_CFG: {            
            switch (cmd->c_argv[0] >> 4) {
                /*
                case CFG_APP:
                    app_set_cfgc(cmd->c_argv[0] & 0x0F, 0, cmd->c_argv[1]);
                    app_ctx.a_flags = lsb;
                break;
                */
                /*    
                case CFG_MAC:
                    mac_set_cfg(cmd->c_argv[0] & 0x0F, 0, cmd->c_argv[1]);
                break;
                */
                case CFG_CC1000:
                    cc1000_set_cfg(cmd->c_argv[0] & 0x0F, 0, cmd->c_argv[1]);
                break;
            }
                
            result = 1;
            break;
        }

        case CMD_REPORT:
            app_report_state();
            break;

	}		

    pce->cmd_result = result;
    return result;
}

/*!
 */
uint8_t app_handle_cmd(pusart_ctx_t pu)
{
    if (usart_find(pu, PSTR("\r\n")) < 0) {
        usart_skip(pu, pu->u_count);
        return 0;
    }

    if (usart_find(pu, PSTR("ON")) >=0 ) {
        usart_skip(pu, pu->u_count);          
        if (app_set_all(1, true) == 0) {
            usart_printf2(USART_PHONE, PSTR("ERROR NO NODES\r\n"));
        } else {
            app_ctx.a_flags |= _BV(APP_REPORT_STATUS);
            usart_printf2(USART_PHONE, PSTR("+OK\r\n"));
        }
    } else if (usart_find(pu, PSTR("OFF")) >= 0) {
        usart_skip(pu, pu->u_count);
        if (app_set_all(0, true) == 0) {
            usart_printf2(USART_PHONE, PSTR("ERROR NO NODES\r\n"));
        } else {
            app_ctx.a_flags |= _BV(APP_REPORT_STATUS);
            usart_printf2(USART_PHONE, PSTR("+OK\r\n"));
        }
    } else if (usart_find(pu, PSTR("STAN")) >= 0) {
        usart_skip(pu, pu->u_count);
        app_report_state();
    } else {
        usart_printf2(USART_PHONE, PSTR("+ERROR\r\n"));
    }

    return 1;
}


/*!
    Handle USART data arrival.
 */
void app_usart_rxc(ptr_t p)
{
    pusart_ctx_t u = NULL;

    // gsm 
    u = usart_get_ctx(USART_PHONE);
    if (u->u_mode == USART_LOCK) {
        app_handle_cmd(u);
    }

    // terminal
    u = usart_get_ctx(USART_TERMINAL);
    if (u->u_mode == USART_LOCK) {
        app_handle_cmd(u);
    }

    event_activate(EVENT_APP_MAIN, NULL);
}

void app_usart_txc(ptr_t p) {
}


/*!
    Receive message from radio link.
 */
void app_radio_rxc(ptr_t p)
{
    pcmd_entry_t pce = (pcmd_entry_t)malloc(sizeof(cmd_entry_t));

    // store received command in queue
    if (pce) {
        pcmd_t pcmd = &pce->cmd;
        // size = max argc + id + argc
        tcp_recv(&pcmd->c_nid, (uint8_t *)pcmd + offsetof(cmd_t, c_id), CMD_MAX_ARGC + 2);
        pce->cmd_type = CMD_LOCAL;
        pce->cmd_result = 0;
        list_push(app_ctx.a_cmd_queue, pce);
    }
    event_activate(EVENT_APP_MAIN, NULL);
}

/*!
    Handle message delivery success.
 */
void app_radio_txc(ptr_t p)
{
    pcmd_entry_t pce = (pcmd_entry_t)list_head(app_ctx.a_cmd_queue);

    if (app_ctx.a_state == APP_WAIT) {
        #ifdef DEBUG_APP
        if (pce->cmd.c_id == CMD_GET_STATE)
            APP_NODE(pce->cmd.c_nid).n_debug.d_sent++;
        #endif
        pce->cmd_result = 1;
        app_ctx.a_state = APP_DONE;
    }
    event_activate(EVENT_APP_MAIN, NULL);
}

/*!
    Handle message delivery failure.
 */
void app_radio_fail(ptr_t p)
{
    pcmd_entry_t pce = (pcmd_entry_t)list_head(app_ctx.a_cmd_queue);
    uint8_t nid = ((plink_frame_t )p)->f_header.f_dest_addr;
    
    if (app_ctx.a_state == APP_WAIT) {
        pce->cmd_result = 0;
        app_ctx.a_state = APP_DONE;
    }

    event_activate(EVENT_APP_MAIN, NULL);
}

void app_query_node(uint8_t timer) 
{
    static uint8_t counter = 0;
	uint16_t time;
	pnode_t node;
    uint8_t i = app_ctx.a_node_index; 
  
    if (i == 0) { 
 	   app_ctx.a_flags &= ~_BV(APP_REPORT_STATUS);
       app_report_state();
	   i = 1;
	   time = APP_QUERY_PERIOD;
	   goto out;
    }

    if (app_ctx.a_nodes_count == 1 || app_ctx.a_state == APP_WAIT) 
       	while(true);
	
	counter += 1;
    if (counter == APP_RESET_PERIOD) {
		while(true);
    }
    
	node = &app_ctx.a_nodes[i];
	// change rely state if required
	if ((node->n_relays >> 4) != (node->n_relays & 0x0f)) {
		app_push_cmd(CMD_SET_STATE, i + 1, CMD_REMOTE, 1, (node->n_relays >> 4));
	}
    // send cmd to query the node                
    app_push_cmd(CMD_GET_STATE, i + 1, CMD_REMOTE, 0);
	
	if (node->n_present > 0) {
         node->n_present -= 1;
         if (node->n_present == 0 && app_ctx.a_nodes_count > 1) {
            app_ctx.a_nodes_count -= 1;
         }
    }

    if (i + 1 < APP_MAX_NODE) {
        i += 1;
        time = APP_QUERY_DURATION;
    } else {
		if (app_ctx.a_flags & _BV(APP_REPORT_STATUS)) {
			i = 0;
			time = APP_QUERY_DURATION;
		} else {
        	i = 1;
        	time = APP_QUERY_PERIOD;
		}
    }

out:
	timer_set(app_ctx.a_timer, time, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);
    app_ctx.a_node_index = i;
}

/*!
    Sample temerature from the sensor
 */
void app_sample_temperature(uint8_t timer) {
    APP_NODE(APP_MASTER_NID).n_temp = ds18x20_get_temperature();
    ds18x20_start_conversion();
}

/**
    \brief Function process commands from the queue.

    \param p not used
 */
void app_main(ptr_t p) 
{
    pcmd_entry_t pce = (pcmd_entry_t)list_head(app_ctx.a_cmd_queue);

    if (pce == NULL)
        goto out;

    switch (app_ctx.a_state) {
        case APP_IDLE:
                // if command designed for this node then execute else send by radio
                if (pce->cmd_type == CMD_LOCAL) {
                    app_cmd_execute(pce);
                    app_ctx.a_state = APP_DONE;
                    DBG_APP(usart_printf(PSTR("CMD_EXECUTE n:%d c:%S\r\n"), pce->cmd.c_nid, cmd_string[pce->cmd.c_id], pce->cmd.c_argv[0]));
                } else {
                    app_ctx.a_state = (app_cmd_send(pce) == 0) ? APP_DONE : APP_WAIT;                  
                    DBG_APP(usart_printf(PSTR("CMD_SEND n:%d c:%S\r\n"), pce->cmd.c_nid, cmd_string[pce->cmd.c_id], pce->cmd.c_argv[0]));
                }
            break;
        
        case APP_WAIT:
            break;

        case APP_DONE: {
            DBG_APP(usart_printf(PSTR("CMD_DONE\r\n")));
            pce = list_pop(app_ctx.a_cmd_queue);
            free(pce);
            //pool_put(app_ctx.a_cmd_pool, (void *)pce);
            app_ctx.a_state = APP_IDLE;
            break;
        }
    }

out:
    event_activate(EVENT_APP_MAIN, NULL);
}

/*!
 */
void app_init(pcfg_t cfg)
{    
    uint8_t i = 0;
    app_ctx.a_cmd_queue = list_create(10);
    app_ctx.a_pcfg = cfg;
	app_ctx.a_timer = timer_add(app_query_node);
	app_ctx.a_node_index = 1;

    for (i = 0; i < APP_MAX_NODE; i++) {
    	app_ctx.a_nodes[i].n_present = APP_NODE_TIMEOUT;
    }
    // restore relay states
    app_set_all(app_ctx.a_pcfg->cfg_relays, false);

    // register event handlers
    event_register(EVENT_TCP_RXC, app_radio_rxc, false);
    event_register(EVENT_TCP_TXC, app_radio_txc, false);
    event_register(EVENT_TCP_FAIL, app_radio_fail, false);
    event_register(EVENT_USART_RXC, app_usart_rxc, true);
    event_register(EVENT_USART_TXC, app_usart_txc, true);
    event_register(EVENT_APP_MAIN, app_main, false);
    
    timer_set(timer_add(app_sample_temperature), APP_TEMP_SAMPLE_TIME, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);
    timer_set(app_ctx.a_timer, APP_QUERY_DURATION, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);

    ds18x20_start_conversion();
    event_activate(EVENT_APP_MAIN, NULL);
}



