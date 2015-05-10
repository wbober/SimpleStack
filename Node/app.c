#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include "timers.h"
#include "transport.h"
#include "app.h"
#include "rssi.h"
#include "ow.h"
#include "cmd.h"


/*!
    Application context
 */
static volatile struct {
    uint8_t a_flags;
    uint8_t a_led_timer;
    uint8_t a_timeout;
    float a_temp;
    pcfg_t a_cfg;
} app_ctx;

/*!
    \brief Send command to remote node
 */
void app_cmd_send(pcmd_t pcmd)
{
    tcp_send(pcmd->c_nid, (uint8_t *)pcmd + offsetof(cmd_t, c_id), pcmd->c_argc + 2);
}

/*!
    Report state to base station
 */
void app_report(uint8_t timer)
{
    if (app_ctx.a_flags & _BV(APP_REPORT)) {
        cmd_t cmd;
        // init state report cmd
        cmd.c_id = CMD_STATE;
        cmd.c_nid = APP_MASTER_NID;
        cmd.c_argc = 4;
        // relay state
        cmd.c_argv[0] = app_ctx.a_flags & _BV(APP_RELAY) ? 1 : 0;
        // current temp sign
        cmd.c_argv[1] = app_ctx.a_temp > 0 ? 1 : 0;
        // current temp value
        cmd.c_argv[2] = abs(app_ctx.a_temp);
        // current temp half
        cmd.c_argv[3] = ((uint8_t)app_ctx.a_temp << 1) % 5 ? 1 : 0;   
        // send command
        app_cmd_send(&cmd); 
    }
}


/*!
 */
void app_set_cfg(uint8_t cfg, uint8_t msb, uint8_t lsb)
{
    switch (cfg) {
        case APP_CFG_FLAGS:
            app_ctx.a_flags = lsb;
        break;
    }
}

/*!
 */
uint8_t app_cmd_execute(pcmd_t cmd) 
{
    uint8_t result = 0;

	switch (cmd->c_id) {
        case CMD_SET_STATE: {
            if (cmd->c_argv[0] == 1) {
                RELAY_ON;
                app_ctx.a_flags |= _BV(APP_RELAY);
            } else {
                RELAY_OFF;
                app_ctx.a_flags &= ~_BV(APP_RELAY);
            }
            break;
        }    

        case CMD_GET_STATE: {
            timer_disable(app_ctx.a_led_timer);
            if (cmd->c_argc > 0) {
                app_ctx.a_timeout = cmd->c_argv[0];
            }
            app_report(0);
            result = 1; 
            break;
        }

        case CMD_SET_CFG: {
            switch (cmd->c_argv[0] >> 4) {
                case CFG_APP:
                    app_set_cfg(cmd->c_argv[0] & 0x0F, 0, cmd->c_argv[1]);
                break;
                case CFG_CC1000:
                    cc1000_set_cfg(cmd->c_argv[0] & 0x0F, 0, cmd->c_argv[1]);
                break;
            }              
            result = 1;
            break;
        }

        default:
            break;
	}	

	return result;
}

/*!
    Receive command.
 */
void app_radio_rxc(ptr_t p)
{
    cmd_t cmd;
    uint8_t addr;

    // mark that nodes has been queried by the base station
    app_ctx.a_flags |= _BV(APP_TOUCHED);

    // size = max argc + id + argc
    tcp_recv(&addr, (uint8_t *)&cmd + offsetof(cmd_t, c_id), CMD_MAX_ARGC + 2);
    app_cmd_execute(&cmd);

    //timer_set(app_ctx.a_command_timer, APP_COMMAND_TIMEOUT, 0, NULL);
}

/*!
    Packet successfully send.
 */
void app_radio_txc(ptr_t p)
{
}

/*!
    Fail to send packet.
 */
void app_radio_fail(ptr_t p)
{
    // fail to send packet to master node, mark node as unregistered
    // app_ctx.a_flags &= ~_BV(APP_TOUCHED);
    
    // start blinking to notice error
    //LED_ON(LED_RED);
}

/*!
    Blink timer handler. 

    Method is invoked to toggle leds wich are used to notice error condition.
 */
void app_led(uint8_t timer)
{
    LED_TOGGLE(LED_RED);
}



/*!
    Register in master node.
 */
void app_keep_alive(uint8_t timer)
{
    static uint8_t c = 0;

    // stop blinking
    timer_disable(app_ctx.a_led_timer);
    LED_OFF(LED_RED);

    // if node is not registered in base station
    if (!(app_ctx.a_flags & _BV(APP_TOUCHED))) {
        if (c++ < app_ctx.a_timeout)
            timer_set(app_ctx.a_led_timer, 200, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);
        else
            while(true);
    } else {
        c = 0;
    }

    app_ctx.a_flags &= ~_BV(APP_TOUCHED);
}


/*!
    Sample temperature from the sensor.
 */
 void app_sample_temperature(uint8_t timer)
 {
    app_ctx.a_temp = ds18x20_get_temperature() + app_ctx.a_cfg->cfg_temp_cal;
    if (app_ctx.a_temp > 50) {
        RELAY_OFF;
    }
    ds18x20_start_conversion();
 }

/*!
    Initialize application layer
 */
void app_init(pcfg_t cfg)
{
    ds18x20_start_conversion();

    // set app parameters
    app_ctx.a_cfg = cfg;
    app_ctx.a_flags = _BV(APP_REPORT);
    
    // register event handlers
    event_register(EVENT_TCP_RXC, app_radio_rxc, false);
    event_register(EVENT_TCP_TXC, app_radio_txc, false);
    event_register(EVENT_TCP_FAIL, app_radio_fail, false);

    // set timer handlers
    timer_set(timer_add(app_sample_temperature), 2000, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);
    timer_set(timer_add(app_keep_alive), 60000, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);
    //timer_set(timer_add(app_report), APP_REPORT_TIME, _BV(TIMER_AUTO) | _BV(TIMER_ENABLED), NULL);
    
    app_ctx.a_led_timer = timer_add(app_led);
    app_ctx.a_temp = ds18x20_get_temperature();
    app_ctx.a_timeout = APP_COMMAND_TIMEOUT;
}









