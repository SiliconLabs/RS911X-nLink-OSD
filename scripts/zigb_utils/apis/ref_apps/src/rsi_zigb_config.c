/**
 *  @file     rsi_zigb_config.c
 *  @version  1.0
 *  @date     2014-Oct-24
 *
 *  Copyright(C) Redpine Signals 2014
 *  All rights reserved by Redpine Signals.
 *
 *  @section License
 *  This program should be used on your own responsibility.
 *  Redpine Signals assumes no responsibility for any losses
 *  incurred by customers or third parties arising from the use of this file.
 *
 *  @brief API: ZigBee Application configuration
 *
 *  @section Description
 *  Basic Structure initailization has done it from here
 *
 *  @section Improvements
 *
 */


/**
 * Includes
 * */

#include "rsi_zigb_types.h"
#include "rsi_zigb_app.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_config.h"
#include "rsi_zigb_onoff.h"
 
/* Basic, Identify and ON_OFF switch configuration clusters on Server side. */
const cluster_id_t a_In_Cluster_List_For_Custom_Device[] = {
  g_BASIC_CLUSTER_c,
  g_IDENTIFY_CLUSTER_c,
  g_ON_OFF_SWITCH_CONFIGURATION_CLUSTER_c
};

/* ON_OFF cluster on Client side */
const cluster_id_t a_Out_Cluster_List_For_Custom_Device[] = {
  g_ON_OFF_CLUSTER_c
};

/* Simple Descriptor */
Simple_Descriptor_t SimpleDesc =  
{
  HA_PROFILE_ID,
  DEV_ID_ON_OFF_SWITCH,
  0x00,
  sizeof(a_In_Cluster_List_For_Custom_Device)/  sizeof(cluster_id_t),
  (cluster_id_t*)a_In_Cluster_List_For_Custom_Device,
  sizeof(a_Out_Cluster_List_For_Custom_Device)/ sizeof(cluster_id_t),
  (cluster_id_t*)a_Out_Cluster_List_For_Custom_Device
};

/* Default values of the Startup Attribute Set */
Startup_Attribute_Set_t Startup_Attribute_Set_Default =
{
    g_EXTENDED_PAN_ID_c,
    g_CHANNEL_MASK_c,
    g_STARTUP_CONTROL_c,
    g_USE_INSECURE_JOIN_c,
    g_SCAN_ATTEMPTS_c,
    g_PARENT_RETRY_THRESHOLD_c,
    g_TRUST_CENTER_ADDRESS_c,
    g_NETWORK_KEY_c,
    g_TIME_BETWEEN_SCANS_c,
    g_REJOIN_INTERVAL_c,
    g_MAX_REJOIN_INTERVAL_c,
    g_POLL_RATE_c,
    g_PANID_c,
    g_NETWORK_MANAGER_ADDRESS_c,
    g_TC_MASTER_KEY_c,
    g_PRECONFG_LINK_KEY_c,
    g_END_DEVICE_TIMEOUT_c
};

/* Default value of the ZDO configuration table */
ZDO_Configuration_Table_t g_Table_Default =
{
    g_PERMIT_JOIN_DURATION_c,
    g_NWK_SECURE_ALL_FRAMES_c,
    g_FORMATION_ATTEMPTS_c,
    g_SCAN_DURATION_c,
    g_JOIN_ATTEMPTS_c,
    g_PRECONFIGURED_KEY_c,
    g_TRUST_CENTER_SHORT_ADDRESS_c,
    g_AUTOMATIC_POLL_ALLOWED_c,
    g_AUTHENTICATION_POLL_RATE_c,
    g_SWITCH_KEY_TIMEOUT_c,
    g_NWK_SECURITY_LEVEL_c,
    g_APS_ACK_POLL_TIME_OUT_c,
    g_MANUFACTURER_CODE_c,
};
