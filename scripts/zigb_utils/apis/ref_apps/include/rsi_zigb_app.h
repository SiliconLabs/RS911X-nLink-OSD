/**
 * @file     rsi_zigb_app.h
 * @version      1.0
 * @date         2014-Aug-21
 *
 * Copyright(C) Redpine Signals 2014
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief HEADER, APP, APPLICATION Header file which contains application specific structures 
 *
 * @section Description
 * This file contains the Application related information.
 *
 *
 */

#ifndef _RSI_ZIGB_APP_CB_H
#define _RSI_ZIGB_APP_CB_H

#include "rsi_zigb_types.h"
#include "rsi_zigb_global.h"
#include "rsi_zigb_interfaces.h"

#define MAX_PKT_LENGTH          256
#define SHORT_PANID_SIZE        2
#define EXTENDED_PANID_SIZE     8
#define EXTENDED_ADDR_SIZE      8

/* ZigBee Application control block */
typedef struct 
{
  /* Channel*/
  uint8_t           channel;
  /* Power */
  uint8_t           power;
  /* received paket count */
  uint32_t          rcvd_data_pkt_count;
  /* Error code */
  uint16_t          error_code;
  /* ZB card ready Ind pkt */
  uint16_t          zb_card_ready_ind;
  /* Device Type */
  ZigBeeDeviceType  device_type;
  /* Num of Scan results / Nwks found */
  uint16_t          num_nwk_found;
  /* FSM state */
  uint16_t          fsm_state;
  /* Mac address */
  uint8_t           mac_addr[8];
  /* PANID */
  uint16_t          panid;
  /* Short Address */
  uint16_t          short_addr;
  /* Flag to send data */
  uint16_t          send_data;
  /* Response frame structure */
  //rsi_uCmdRsp       *uCmdRspFrame;
  /* packet pending flag */
  volatile uint32_t pkt_pending;
  /* Send buffer data */
  uint8_t           send_buffer[MAX_PKT_LENGTH];
  /* Buffer to hold the received packet */
  uint8_t           read_packet_buffer[MAX_PKT_LENGTH];

} rsi_zigb_app_cb_t;

typedef struct
{
  uint32_t channel;
  uint8_t  scan_status;  
} scan_done_cb_t;

typedef struct
{
  /* data confirmation wait */
  uint8_t dataConfWait;
  /* Cmd response confirmation status */
  uint8_t matchDescRspStatus;
  uint8_t powerDescRspStatus;
  uint8_t nodeDescRspStatus;
  uint8_t bindRspStatus;
  uint8_t unbindRspStatus;
  uint8_t actepRspStatus;
  uint8_t ieeeaddrRspStatus;
  uint8_t simpledescRspStatus;
  uint8_t networkaddrRspStatus;
  uint8_t scanReqSent;
} rsi_zigb_app_status_var_t;

typedef struct
{
  /* Cmd response confirmation data */
  uint8_t matchDescResp[30];
  uint8_t powerDescResp[30];
  uint8_t nodeDescResp[30];
  uint8_t bindResp[30];
  uint8_t unbindResp[30];
  uint8_t actepResp[30];
  uint8_t ieeeaddrResp[30];
  uint8_t simpledespResp[30];
  uint8_t networkaddrResp[30];
  uint8_t asdu_pkt[RSI_ZIGB_MAX_PAYLOAD_SIZE];
} rsi_zigb_app_resp_info_t;

typedef struct
{
  /* Stack Handler status */
  ZigBeeNWKStatusInfo        stack_status;      
  /* Scan Complete Handler status */    
  scan_done_cb_t             scan_done_cb;
  /* ZigBee Network Coordinator Info */
  ZigBeeNetworkDetails       nwkinfo;
  /* Data Confirmation Handling */
  APSDE_Data_Confirmation_t  data_conf;
  /* ZigBee Variable Status */
  rsi_zigb_app_status_var_t  status_var;
  /* ZigBee Response Info */
  rsi_zigb_app_resp_info_t   zb_resp_info;
  /* ZigBee Simple discriptor */
  Simple_Descriptor_t        *DeviceSimpleDesc;
  /* response wait variable */
  uint8_t                    wait_for_rsp;
  /* Basic, Identify and ON_OFF switch configuration 
   * clusters on Server side. */
  cluster_id_t               *a_In_Cluster_List; 
  /* ON_OFF cluster on Client side */
  cluster_id_t               *a_Out_Cluster_List;
  /* Channel Mask */
  uint32_t                    channel_mask;
  /* Scan Duration */
  uint8_t                     scan_duration;
  /* Network down*/
  uint8_t                     network_down;
} rsi_zigb_app_info_t;

ZigBeeNWKStatusInfo nwkStatusInfo;
extern rsi_zigb_app_cb_t rsi_zigb_app_cb;

/* Application Specific Simple descriptor */
#define ONOFF_SWITCH_END_POINT     0x01
#define ONOFF_LIGHT_END_POINT      0x02
#define HA_PROFILE_ID              0x0104
#define DEV_ID_ON_OFF_SWITCH       0x00
#define ON_OFF_LIGHT_ID            0x0100


/* Function prototypes */ 
void rsi_zb_app_init(uint32_t);
RSI_ZB_STATUS rsi_zigb_handle_data();
RSI_ZB_STATUS rsi_zigb_handle_join_state(uint8_t intf, uint8_t cmd);
RSI_ZB_STATUS rsi_zigb_handle_scan_state(uint8_t intf, uint8_t cmd);
RSI_ZB_STATUS rsi_zigb_handle_form_state(uint8_t intf, uint8_t cmd);
int16_t rsi_zigb_frame_read(uint8_t *packet_buffer);
uint8_t rsi_zigb_channel_ext(uint32_t channel_Mask);
#ifdef ZB_ONLY
int16_t rsi_zigb_init_oper_mode(void); 
#endif
#endif
