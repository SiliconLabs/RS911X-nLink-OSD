/**
 *  @file     rsi_zigb_onoff.h
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
 
#ifndef RZSP_ONOFF_H
#define RZSP_ONOFF_H
//in Cluster id defines. Remove these once profile porting completes.
#define g_BASIC_CLUSTER_c                             0x0000
#define g_IDENTIFY_CLUSTER_c                          0x0003
#define g_GROUPS_CLUSTER_c                            0x0004
#define g_ON_OFF_SWITCH_CONFIGURATION_CLUSTER_c       0x0007
#define g_SCENES_CLUSTER_c                            0x0005

//out Cluster id defines. Remove these once profile porting completes.
#define g_ON_OFF_CLUSTER_c        0x0006

enum
{
  g_OFF_c,
  g_ON_c,
  g_TOGGLE_c
};

typedef struct lightDeviceInfo_tag{

  uint16_t shortaddress;
  uint8_t endpoint;

}lightDeviceInfo_t;

typedef enum FrameControl_Direction
{
  g_Client_To_Server_c = 0x00,
  g_Server_To_Client_c = 0x08

}Frame_Control_direction;


#if 0
typedef struct App_ZCL_Request_Tag{
  uint8_t command_type;
  uint8_t manufacture_specific;
  uint8_t a_manufacturer_code[2];
  uint8_t disable_default_response;
  ZCL_Command_t ZCL_command_received;
  }App_ZCL_Request_t;
#endif

#endif
