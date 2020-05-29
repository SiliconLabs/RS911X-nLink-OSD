/**
 *  @file     rsi_zigb_api_test.h
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
 *  @brief 
 *  structures and variables for on/off cluster
 *
 *  @section Description
 *  This file contain structures and variable used for on/off cluster
 *
 *  @section Improvements
 *
 */


/**
 * Includes
 * */
 
#ifndef RSI_ZB_APITEST_H
#define RSI_ZB_APITEST_H

//in Cluster id defines. Remove these once profile porting completes.
#define CUSTOM_EP           1

#define CUSTOM_CLUSTER      0x0001
#define CUSTOM_PROFILE      0x0F05
#define CUSTOM_DEVICEID     0x0001
#define CUSTOM_VERSION      0
#define CHANNEL_12          12
//out Cluster id defines. Remove these once profile porting completes.
#define g_ON_OFF_CLUSTER_c                             0x0006
#define OCCUPANCY_SENSING_CLUSTER                      0x0406

uint16_t TestCasesExecuted , TestCasesFailed , TestCasesSucceded ;
#define TCSUCCESS TestCasesExecuted++;TestCasesSucceded++; 
#define TCFAIL TestCasesExecuted++;TestCasesFailed++; 

typedef struct
{
  Address SelfAddress;
  Address ParentAddress;
  uint8_t Source_EP;
  uint8_t Dest_EP;
  uint8_t DestAddrMode;
  BOOL APSAckRequired;
  uint16_t ClusterId;
} rsi_zigb_apitest_t;

typedef struct {
  uint8_t                 key_index;
  uint8_t                 channel;
  uint8_t                 data_indication_count;
  uint8_t                 data_cnrfm_count;
  uint8_t                 end_device_poll_count;
  uint8_t                 cmd;
  uint8_t                 child_index;
  uint8_t                 child_dev_type;
  uint8_t                 child_dev_count;
  uint8_t                 rout_child_dev_count;
  uint8_t                 aps_ack_required;
  uint8_t                 end_pont_id;
  uint8_t                 request_type;
  uint8_t                 start_index;

  uint16_t                endpoint_cluster;
  uint16_t                ManufacturerCode;
  uint16_t                short_addr_t;
  uint16_t                parent_short_addr;
  uint16_t                pan_Id;
  uint16_t                child_short_addr_t;
  uint16_t                broad_cast_addr;

  ZDP_Bind_Request_t      SetBindingEntry;
  Address                 DestAddress;
  ZigBeeKeyStructure_t    KeyStruct;
  Node_Power_Descriptor_t nodePowerDesc_t;
  ZigBeeAPSDEDataRequest_t APSDEDataReq;
  RSI_ZB_STATUS           rsi_status;
  uint8_t                 *link_key_ptr;
  uint8_t                 parent_ieee[8];
  uint8_t                 child_ieee[8];
  uint8_t                 get_ieee[8];
  uint8_t                 Ext_PanId[8] ;
} rsi_zigb_apitest_var_t;

RSI_ZB_STATUS rsi_zigb_api_test();
RSI_ZB_STATUS rsi_zigb_router_api_test();
RSI_ZB_STATUS rsi_zigb_cord_api_test();
void rsi_zigb_api_var_init();
#endif
