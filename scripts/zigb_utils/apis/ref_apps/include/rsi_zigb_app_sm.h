/**
 *  @file     rsi_zigb_app_sm.h
 *  @version  1.0
 *  @date     2014-Aug-23
 *
 *  Copyright(C) Redpine Signals 2014
 *  All rights reserved by Redpine Signals.
 *
 *  @section License
 *  This program should be used on your own responsibility.
 *  Redpine Signals assumes no responsibility for any losses
 *  incurred by customers or third parties arising from the use of this file.
 *
 *  @brief Various data structures and variables of state machine
 *
 *  @section Description
 *  This file contain structures and function for handling state machine
 *
 *  @section Improvements
 *  New command frames are added.
 *
 */


/**
 * Includes
 * */
 
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum fsm_state_s {
  FSM_CARD_NOT_READY = 0,
  FSM_CARD_READY,
  FSM_INIT_STACK,
  FSM_RESET_STACK,
  FSM_GET_DEV_TYPE,
  FSM_INIT_ENDDEV_ROUTER,
  FSM_INIT_COORDINATOR,
  FSM_INIT_SCAN,
  FSM_SCAN_DONE,
  FSM_JOIN_NETWORK,
  FSM_FORM_NETWORK,
  FSM_ZB_CONNECTED,
  FSM_API_TEST,
  FSM_ZB_FORMED,
  FSM_ZB_HANDLE_ROUTER,
  FSM_ZB_REJOIN_SEND,
  FSM_ZB_REJOINED,
  FSM_ZB_PS_CMD
} fsm_state_t;


typedef enum data_events {
  g_DATA_NO_EVENT_c,
  g_TRIGGER_EVENT_c,
  g_MATCH_DESC_REQ_c,
  g_SEND_ON_CMD_c,
  g_READ_ONOFF_ATTRIBUTE_c,
  g_SEND_OFF_CMD_c,
  g_WAIT_FOR_MATCH_RESP_c,
  g_WAIT_TO_COMPLETE_BCAST_c,
  g_WAIT_FOR_ON_CMD_CONF_c,
  g_WAIT_FOR_OFF_CMD_CONF_c,
  g_WAIT_FOR_READ_ATTRIB_RESP_c,
} data_events_t;



#endif
