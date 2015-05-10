#ifndef CMD_H
#define CMD_H

/*!
 */
#define CMD_MAX_ARGC    5

// application configuration
#define CFG_APP         0
// cc1000 configuration 
#define CFG_CC1000      1

/*!
 */
typedef enum _cmd_id_t {
    CMD_SET_STATE = 0,
    CMD_GET_STATE = 1,
    CMD_STATE = 2,
    CMD_SET_CFG = 3,
    CMD_GET_CFG = 4,
    CMD_REPORT = 5
} cmd_id_t;

/*!
    Command definition.
 */
typedef struct {
    uint8_t c_nid;
    uint8_t c_id;
    uint8_t c_argc;
    uint8_t c_argv[CMD_MAX_ARGC];
} cmd_t, *pcmd_t;


#endif
