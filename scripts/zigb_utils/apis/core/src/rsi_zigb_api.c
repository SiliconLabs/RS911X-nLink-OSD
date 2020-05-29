/**
 *   @file     rsi_zigb_api.c
 *   @version  1.0
 *   @date     2014-Sep-13
 *
 *   Copyright(C) Redpine Signals 2014
 *   All rights reserved by Redpine Signals.
 *
 *   @section License
 *   This program should be used on your own responsibility.
 *   Redpine Signals assumes no responsibility for any losses
 *   incurred by customers or third parties arising from the use of this file.
 *
 *   @brief API: Definitions of various data structures and variables
 *
 *   @section Description
 *   This file contain definition of different mangament, control & data commands variables.
 *   These definition are used to construct frames. 
 *
 *   @section Improvements
 *
 */

/**
 * Includes
 * */
#include "rsi_zigb_types.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_global.h"
#include "rsi_zigb_interfaces.h"

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_init_stack(void)
 * @brief       Sends the ZigBee Stack Initialization command to the ZigBee 
 * module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to initialize the ZigBee stack. This API should be first 
 * command called after receiving card ready.
 *
 */
int16_t rsi_zigb_init_stack(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameInitStack, NULL, 0);
  
  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_reset_stack(void)
 * @brief       Sends the ZigBee Stack reset command to the ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to soft reset the ZigBee stack. 
 *
 */
int16_t rsi_zigb_reset_stack(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameResetStack, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_sleep_request(uint32_t timeout)
 * @brief       Sends the device sleep request command to the ZigBee module
 * @param[in]   timeout - Sleep timeout
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to send the sleep request to module. Don't send any other
 * Commands without issuing wakeup request.
 *
 */
int16_t rsi_zigb_sleep_request(uint32_t timeout)
{
  uint8_t pkt_size = 0;
  int16_t ret_val;
  sleepReqFrameSnd sleep_req;

  pkt_size = sizeof(sleepReqFrameSnd);
  
  sleep_req.timeout = timeout;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSleepReq, (uint8_t *)&sleep_req, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_wake_up_request(void)
 * @brief       Sends the device wake-up command to device
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to send the device wake-up command to device. Don't send any
 * Commands without issuing this request
 *
 */
int16_t rsi_zigb_wake_up_request(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameWakeupReq, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_form_network(uint8_t RadioChannel, 
 * 				uint8_t tx_power, uint8_t *pExtendedPanId)
 * @brief       Sends the ZigBee Form Network command to ZigBee Module
 * @param[in]   RadioChannel - Channel in which network is to be formed
 *              tx_power - 
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to soft reset the ZigBee stack. 
 *
 */
int16_t rsi_zigb_form_network (uint8_t RadioChannel, uint8_t power,
    uint8_t * pExtendedPanId )
{
  uint8_t pkt_size;
  int16_t ret_val;
  formNetworkFrameSnd form_nwk;

  pkt_size = sizeof(formNetworkFrameSnd);

  form_nwk.RadioChannel = RadioChannel;
  form_nwk.power = power;

  rsi_zigb_copy_xt_panid(pExtendedPanId, (uint8_t *)&form_nwk.ExtPanId);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameFormNWK, (uint8_t *)&form_nwk, pkt_size);

  return ret_val;
}

int16_t rsi_zigb_join_network(uint8_t DeviceType, uint8_t RadioChannel,uint8_t power ,uint8_t * pExtendedPanId)
{
  uint8_t pkt_size;
  int16_t ret_val;
  joinNetworkFrameSnd join_nwk;

  pkt_size = sizeof(joinNetworkFrameSnd);

  join_nwk.DeviceType = DeviceType;
  join_nwk.RadioChannel = RadioChannel;
  join_nwk.power = power;	

  rsi_zigb_copy_xt_panid(pExtendedPanId, (uint8_t *)&join_nwk.ExtPanId);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameJoinNWK, (uint8_t *)&join_nwk, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_permit_join(uint8_t PermitDuration)
 * @brief       Sends the permit join command to ZigBee module
 * @param[in]   PermitDuration - Duration to accept associations
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to trigger permit join for the specific amount of time.
 *
 */
int16_t rsi_zigb_permit_join(uint8_t PermitDuration)
{
  uint8_t pkt_size;
  int16_t ret_val;
  permitJoinFrameSnd permit_join;

  pkt_size = sizeof(permitJoinFrameSnd);

  permit_join.PermitDuration = PermitDuration;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_framePermitJoin, (uint8_t *)&permit_join, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_leave_network(void)
 * @brief       Sends the leave network command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to leave the network and connect to another device
 *
 */
int16_t rsi_zigb_leave_network(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameLeaveNWK, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_find_network_and_perform_rejoin(bool secured,
 * 					uint32_t ChannelMask)
 * @brief       Sends the Find network and perform rejoin command to ZigBee module 
 * @param[in]   Secured - Is rejoin secured or not
 * 				ChannelMask - Channels to scan
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to perform rejoin by finding network by stack itself
 *
 */
int16_t rsi_zigb_find_network_and_perform_rejoin(BOOL Secured, uint32_t ChannelMask)
{
  uint8_t pkt_size;
  int16_t ret_val;
  findNWKRejoinFrameSnd buffer;

  pkt_size = sizeof(findNWKRejoinFrameSnd);

  buffer.Secured = Secured;

  buffer.ChannelMask= ChannelMask;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameFindNWKnRejoin, (uint8_t *)&buffer, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_rejoin_network(bool secured)
 * @brief       Sends the perform rejoin command to ZigBee module 
 * @param[in]   Secured - Is rejoin secured or not
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to perform rejoin in the channel connected
 *
 */
int16_t rsi_zigb_rejoin_network(BOOL Secured)
{
  uint8_t pkt_size;
  int16_t ret_val;
  rejoinNWKFrameSnd rejoin;

  pkt_size = sizeof(rejoinNWKFrameSnd);

  rejoin.Secured = (uint8_t)Secured;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameRejoinNWK, (uint8_t *)&rejoin, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_network_restore(void)
 * @brief       Sends the perform network restore command to ZigBee module 
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to restore the previos network without expecting configuration 
 * parameters again
 *
 */
int16_t rsi_zigb_network_restore(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameNWKRestore, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          rsi_zigb_initiate_scan(uint8_t scanType, 
 *                                     uint32_t channelMask, 
 *                                     uint8_t duration)
 * @brief       Sends the scan command to ZigBee module 
 * @param[in]   scanning Type
 * @param[in]   channel Mask
 * @param[in]   duration to scan in  each channel
 * @param[out]  status
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used initiate scan process
 *
 *===========================================================================
 */
int16_t rsi_zigb_initiate_scan(uint8_t ScanType, uint32_t ChannelMask, uint8_t duration)
{
  uint8_t pkt_size = 0;
  int16_t ret_val;
  initScanFrameSnd init_scan;

  if(duration > MAX_SCAN_DURATION) {
    return 1; 
  }

  pkt_size = sizeof(initScanFrameSnd);

  init_scan.ScanType = ScanType;

  init_scan.ChannelMask = ChannelMask;
  
  init_scan.duration = duration;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameInitScan, (uint8_t *)&init_scan, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_stop_scan(void)
 * @brief       Sends the stop scan command to ZigBee module 
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to stop the ongoing scan procedure
 *
 */
int16_t rsi_zigb_stop_scan(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameStopScan, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_network_state(void)
 * @brief       Sends the network state command to ZigBee
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to read the cuurent network state
 *
 */
int16_t rsi_zigb_network_state(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameNWKState, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_stack_is_up(void)
 * @brief       Sends the stack is up command to ZigBee
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to read the stack status to know the stack state
 *
 */
int16_t rsi_zigb_stack_is_up(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameStackIsUp, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_self_ieee_address(void)
 * @brief       Sends the get self 64-bit extended address from ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to read back the self IEEE address of the ZigBee device
 *
 */
int16_t rsi_zigb_get_self_ieee_address(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetSelfIEEE, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_is_it_self_ieee_address(uint8_t *pIEEEAddress)
 * @brief       Sends the self IEEE Address check command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to check whether the given address is self IEEE address or not
 *
 */
BOOL rsi_zigb_is_it_self_ieee_address(uint8_t *pIEEEAddress)
{
  uint8_t pkt_size = 8;
  int16_t ret_val;
  isitSelfIEEEFrameSnd self_ieee;

  pkt_size = sizeof(isitSelfIEEEFrameSnd);

  rsi_zigb_copy_xt_panid(pIEEEAddress, (uint8_t *)&self_ieee.ieee_Addr);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameIsItSelfIEEE, (uint8_t *)&self_ieee, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_self_short_address(void)
 * @brief       Sends the self short Address command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the self short address of the device
 *
 */
int16_t rsi_zigb_get_self_short_address(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetSelfShortAddr, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_manufacturer_code_for_node_desc(uint16_t
 * 						ManufacturerCode)
 * @brief       Sends the set manufcaturing code command to ZigBee module
 * @param[in]   ManufacturerCode - 16-bit manufcaturer code
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to set the manufacturer  code for node descriptor
 *
 */
int16_t rsi_zigb_set_manufacturer_code_for_node_desc(uint16_t ManufacturerCode)
{
  uint8_t pkt_size = 0;
  int16_t ret_val;
  setManufFrameSnd set_manuf;

  pkt_size = sizeof(setManufFrameSnd);

  set_manuf.ManufacturerCode = ManufacturerCode;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetManufNodeDesc, (uint8_t *)&set_manuf, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_power_descriptor(Node_Power_Descriptor_t
 * 								*nodePowerDesc)
 * @brief       Sends the set power descriptor command to ZigBee module
 * @param[in]   nodePowerDesc - Node Power descriptor 
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to set the node power descriptor
 *
 */
int16_t rsi_zigb_set_power_descriptor(Node_Power_Descriptor_t *nodePowerDesc)
{
  uint8_t pkt_size = 0;
  int16_t ret_val;
  setPowerDescFrameSnd set_pwr_desc;

  pkt_size = sizeof(setPowerDescFrameSnd);

  set_pwr_desc.PowerSources = nodePowerDesc->current_powermode_avail_power_sources;
  set_pwr_desc.CurPowerLevel = nodePowerDesc->current_powersource_currentpowersourcelevel;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetPwrDesc, (uint8_t *)&set_pwr_desc, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_maxm_incoming_txfr_size(uint16_t 
 * 							MaxIncomingTxfrSize)
 * @brief       Sends the max incoming frame tx frame size command to ZigBee 
 * @param[in]   MaxIncomingTxfrSize - max possible Incoming tx frame size
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to set the maximum incoming tx frame size to ZigBee stack
 *
 */
int16_t rsi_zigb_set_maxm_incoming_txfr_size(uint16_t MaxIncomingTxfrSize)
{
  uint8_t pkt_size = 0;
  int16_t ret_val;
  setMaxIncomingTxfrFrameSnd max_in_txfr;

  pkt_size = sizeof(setMaxIncomingTxfrFrameSnd);

  max_in_txfr.MaxIncomingTxfrSize = MaxIncomingTxfrSize;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetMaxIncmgSize, (uint8_t *)&max_in_txfr, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_maxm_out_going_txfr_size(uint16_t 
 * 							MaxOutgoingTxfrSize)
 * @brief       Sends the max outgoing frame tx frame size command to ZigBee 
 * @param[in]   MaxOutgoingTxfrSize - max possible outgoing tx frame size
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to set the maximum outgoing tx frame size to ZigBee stack
 *
 */
int16_t rsi_zigb_set_maxm_out_going_txfr_size(uint16_t MaxOutgoingTxfrSize)
{
  uint8_t pkt_size = 2;
  int16_t ret_val;
  setMaxOutTxfrFrameSnd max_out_txfr;

  pkt_size = sizeof(setMaxOutTxfrFrameSnd);

  max_out_txfr.MaxOutgoingTxfrSize = MaxOutgoingTxfrSize;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetMaxOutSize, (uint8_t *)&max_out_txfr, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_operating_channel(uint8_t channel)
 * @brief       Sends the Set channel command to ZigBee 
 * @param[in]   channel - channel in which operations need to be carried out
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to chnage the operating channel in ZigBee stack
 *
 */
int16_t rsi_zigb_set_operating_channel(uint8_t Channel)
{
  uint8_t pkt_size = 1;
  int16_t ret_val;
  setOperChanFrameSnd set_chan;

  pkt_size = sizeof(setOperChanFrameSnd);

  set_chan.Channel = Channel;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetChan, (uint8_t *)&set_chan, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_device_type(void)
 * @brief       Sends the get device type command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the current decice type.
 *
 */
int16_t rsi_zigb_get_device_type(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetDevType, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_operating_channel()
 * @brief       Sends the get cuurent operating channel command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the current operating channel commanf from ZigBee stack
 *
 */
int16_t rsi_zigb_get_operating_channel(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetOperChan, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_short_panid()
 * @brief       Sends the get cuurent short panID  command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the current using PANID from ZigBee stack
 *
 */
int16_t rsi_zigb_get_short_panid(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetShortPanId, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_extended_panid(void)
 * @brief       Sends the get extended panid  command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the current using extended PANID from ZigBee stack
 *
 */
int16_t rsi_zigb_get_extended_panid(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetExtPanId, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_endpoint_id(uint8_t Index)
 * @brief       Sends the get command Endpoint ID from ZigBee stack
 * @param[in]   Index - Index from which the endpoint need to be read
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the endpoint ID from the ZigBee stack for the given Index
 *
 */
int16_t rsi_zigb_get_endpoint_id(uint8_t Index)
{
  uint8_t pkt_size = 1;
  int16_t ret_val;
  getEndPointIdFrameSnd get_ep;

  pkt_size = sizeof(getEndPointIdFrameSnd);

  get_ep.Index = Index;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetEP, (uint8_t *)&get_ep, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_simple_descriptor(uint8_t endpointId)
 * @brief       Sends the get Simple Descriptor command to ZigBee module
 * @param[in]   endpointId - Simple descriptor for the specific endpointId 
 * 				to be returned
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the simple descriptor for the sepcific endpoint
 * number provided.
 *
 */
int16_t rsi_zigb_get_simple_descriptor(uint8_t endpointId)
{
  uint8_t pkt_size = 1;
  int16_t ret_val;
  getSimpleDescFrameSnd get_simple_desc;

  pkt_size = sizeof(getSimpleDescFrameSnd);

  get_simple_desc.EndPointId = endpointId;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetSimpleDesc, (uint8_t *)&get_simple_desc, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_endpoint_cluster(uint8_t endpointId, 
 * 						uint8_t ClusterType, uint8_t ClusterIndex)
 * @brief       Sends the get endpoint cluster  command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the endpoint cluster 
 *
 */
int16_t rsi_zigb_get_endpoint_cluster(uint8_t EndPointId, uint8_t ClusterType,uint8_t ClusterIndex)
{
  uint8_t pkt_size = 3;
  int16_t ret_val;
  getEPClusterFrameSnd get_ep_cluster;

  pkt_size = sizeof(getEPClusterFrameSnd);

  get_ep_cluster.EndPointId = EndPointId;
  get_ep_cluster.ClusterType = ClusterType;
  get_ep_cluster.ClusterIndex = ClusterIndex;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetEPCluster, (uint8_t *)&get_ep_cluster, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_short_addr_for_specified_ieee_addr(uint8_t
 * 						*pIEEEAddress)
 * @brief       Sends the get cuurent short panID  command to ZigBee module
 * @param[in]   pIEEEAddress - 64-bit extended address
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the short address for specific 64-bit IEEE address
 * from ZigBee stack
 *
 */
int16_t rsi_zigb_get_short_addr_for_specified_ieee_addr(uint8_t * pIEEEAddress)
{
  uint8_t pkt_size = 8;
  int16_t ret_val;
  getShortAddrForIeeeAddrFrameSnd get_short_for_ieee;

  pkt_size = sizeof(getShortAddrForIeeeAddrFrameSnd);

  rsi_zigb_copy_xt_panid(pIEEEAddress, (uint8_t *)&get_short_for_ieee.ieee_Addr);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetShortAddrForIEEE, 
                                 (uint8_t *)&get_short_for_ieee , pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_stack_profile(void)
 * @brief       Sends the get Stack Profile  command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the ZigBee stack profile 
 * 
 */
int16_t rsi_zigb_stack_profile(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetStackProfile, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_ieee_addr_for_specified_short_addr(uint16_t shortAddr)
 * @brief       Sends the get 64-bit extended address using short address command to ZigBee module
 * @param[in]   uint16_t ShortAddr, short address to get 64-bit extended address
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the 64-bit extended address for the given short address.
 * 
 */
int16_t rsi_zigb_get_ieee_addr_for_specified_short_addr(uint16_t shortAddr)
{
  uint8_t pkt_size = 2;
  int16_t ret_val;
  getIeeeAddrForShortAddrFrameSnd get_ieee_for_short;

  pkt_size = sizeof(getIeeeAddrForShortAddrFrameSnd);

  rsi_zigb_uint16to_buffer(shortAddr, (uint8_t *)&get_ieee_for_short.ShortAddr);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetIEEEForShortAddr, (uint8_t *)&get_ieee_for_short, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_read_neighbor_table_entry(uint8_t index)
 * @brief       Sends the neighbor table entry read command to ZigBee module
 * @param[in]   uint16_t index, index of neighbor table
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the neighbor table entry for the given index
 * 
 */
int16_t rsi_zigb_read_neighbor_table_entry(uint8_t Index)
{
  uint8_t pkt_size = 1;
  int16_t ret_val;
  readNeighborTableFrameSnd read_neighbor_table;

  pkt_size = sizeof(readNeighborTableFrameSnd);

  read_neighbor_table.Index = Index;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameReadNeighborTable, (uint8_t *)&read_neighbor_table, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_route_table_entry(uint8_t index)
 * @brief       Sends the route table entry read command to ZigBee module
 * @param[in]   uint16_t index, index of route table
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the route table entry for the given index
 * 
 */
int16_t rsi_zigb_get_route_table_entry(uint8_t Index)
{
  uint8_t pkt_size = 1;
  int16_t ret_val;
  getRouteTableFrameSnd get_route_table;

  pkt_size = sizeof(getRouteTableFrameSnd);

  get_route_table.Index = Index;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetRouteTable, (uint8_t *)&get_route_table, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_tree_depth(void)
 * @brief       Sends the current tree depth read command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to retrieve the current tree depth where the device has joined.
 *
 */
int16_t rsi_zigb_tree_depth(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameTreeDepth, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_neighbor_table_entry_count(void)
 * @brief       Sends the neighbor table entries count read command to ZigBee Module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to know the count of active neighbor table entries.
 *
 */
int16_t rsi_zigb_get_neighbor_table_entry_count(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetNeighborTable, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_child_short_address_for_the_index(uint8_t index)
 * @brief       Sends the child short address read command to ZigBee module
 * @param[in]   uint8_t index, index of the child for which the short address to be read
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to read the 16-bit short address of the child in the specified index.
 *
 */
int16_t rsi_zigb_get_child_short_address_for_the_index(uint8_t ChildIndex)
{
  uint8_t pkt_size = 1;
  int16_t ret_val;
  getChildShortAddrFrameSnd get_child_short_addr;

  pkt_size = sizeof(getChildShortAddrFrameSnd);

  get_child_short_addr.ChildIndex = ChildIndex;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetChildShortAddr, (uint8_t *)&get_child_short_addr, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_child_index_for_specified_short_addr(uint16_t childShortAddr)
 * @brief       Sends the child short address index read command to ZigBee module
 * @param[in]   uint16_t childShortAddr, short address of the device for which the index to be read
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the index for the specified 16-bit child address 
 *
 */
int16_t rsi_zigb_get_child_index_for_specified_short_addr(uint16_t childShortAddr)
{
  uint8_t pkt_size = 2;
  int16_t ret_val;
  getChildIndexForShortAddrFrameSnd get_child_index;

  pkt_size = sizeof(getChildIndexForShortAddrFrameSnd);

  rsi_zigb_uint16to_buffer(childShortAddr, (uint8_t *)&get_child_index.ShortAddr);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetChildIndexForShortAddr, (uint8_t *)&get_child_index, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_child_details(uint8_t index)
 * @brief       Sends the child short address index read command to ZigBee module
 * @param[in]   uint8_t index, index of the child of interest
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the child details for the specified child index.
 *
 */
int16_t rsi_zigb_get_child_details(uint8_t Index)
{
  uint8_t pkt_size = 1;
  int16_t ret_val;
  getChildDetailsFrameSnd get_child_details;

  pkt_size = sizeof(getChildDetailsFrameSnd);
 
  get_child_details.Index = Index;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetChildDetails, (uint8_t *)&get_child_details, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_end_device_poll_for_data(void)
 * @brief       Sends the data request initiate command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to poll the parent for the data. 
 *
 */
int16_t rsi_zigb_end_device_poll_for_data( void )
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_framePollForData, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_end_device_poll_for_data(void)
 * @brief       Sends the data request initiate command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to poll the parent for the data. 
 *
 */
int16_t rsi_zigb_read_count_of_child_devices(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameReadChildCnt, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_read_count_of_child_devices(void)
 * @brief       Sends the get the child count command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to read the number of child devices on the node.
 *
 */
int16_t rsi_zigb_read_count_of_router_child_devices(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameReadRouterChildCnt, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_parent_short_address(void)
 * @brief       Sends the get parent short address command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to read the 16-bit short address of parent.
 *
 */
int16_t rsi_zigb_get_parent_short_address(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetParentShortAddr, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_parent_ieee_address(void)
 * @brief       Sends the get parent IEEE address command to ZigBee module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to read the 64-bit extended address of parent.
 *
 */
int16_t rsi_zigb_get_parent_ieee_address(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetParentIEEEAddr, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_initiate_energy_scan_request(uint16_t DestAddr, uint32_t ScanChannels,
 * 							uint8_t ScanDuration, uint8_t ScanRetry
 * @brief       Sends the Energy detection scan initiate command to ZigBee module
 * @param[in]   uint16_t DestAddr,indicates The network address of the device to perform the scan.
 * 				uint32_t ScanChannels, indicates A mask of the channels to be scanned.
 * 				uint8_t ScanDuration, indicates How long to scan on each channel. Allowed values
 *   			are 0..5, with the scan times as specified by 802.15.4 (0 = 31ms, 1 = 46ms,
 *   			2 = 77 ms, 3 = 138ms, 4 = 261ms, 5 = 507ms).
 *   			ScanRetry, indicates The number of scans to be performed on each channel
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API allows to request energy scan be performed and its
 * results returned. This request may only be sent by the current network manager and
 * must be unicast, not broadcast.
 *
 */
int16_t rsi_zigb_initiate_energy_scan_request(uint16_t DestAddr,uint32_t ScanChannels, uint8_t ScanDuration, uint16_t ScanRetry)
{
  int16_t ret_val;
  uint8_t pkt_size = 9;
  initEnergyScanFrameSnd init_scan;

  pkt_size = sizeof(initEnergyScanFrameSnd);

  rsi_zigb_uint16to_buffer(DestAddr, (uint8_t *)&init_scan.DestAddr);

  rsi_zigb_uint32to_buffer(ScanChannels, (uint8_t *)&init_scan.ScanChannels);

  init_scan.ScanDuration = ScanDuration;

  rsi_zigb_uint16to_buffer(ScanRetry, (uint8_t *)&init_scan.ScanRetry);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameInitEnergyScan, (uint8_t *)&init_scan, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_broadcast_nwk_manager_request(uint16_t NWKManagerShortAddr,
 * 				uint32_t ActiveChannels)
 *
 * @brief       Sends the Broadcast request command to ZigBee module
 *
 * @param[in]   uint16_t NWKManagerShortAddr,indicates The network address of the Network Manager.
 * 				uint32_t ActiveChannels, indicates the new active channel masks.
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This function allows the Application broadcasts a request to set the identity
 * of the network manager and the active channel mask. The mask is used when
 * scanning for the network after missing a channel update.
 *
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_broadcast_nwk_manager_request(uint16_t NWKManagerShortAddr, uint32_t ActiveChannels)
{
  int16_t ret_val;
  uint8_t pkt_size = 6;
  bcastNWKManagerFrameSnd bcast_nwk;

  pkt_size = sizeof(bcastNWKManagerFrameSnd);

  rsi_zigb_uint16to_buffer(NWKManagerShortAddr, (uint8_t *)&bcast_nwk.ShortAddr);

  rsi_zigb_uint32to_buffer(ActiveChannels, (uint8_t *)&bcast_nwk.ActiveChannels);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameBcastNWKManagerReq, (uint8_t *)&bcast_nwk, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_zdp_send_nwk_addr_request(uint8_t *pIEEEAddrOfInterest,
 * 						bool RequestType, uint8_t StartIndex)
 *
 * @brief       Sends the NWK Address request command to ZigBee module
 *
 * @param[in]   uint8_t *pIEEEAddrOfInterest, pointer to location of IEEE address whose 16-bit Network
 * 				address is to be determined.
 * 				BOOL RequestType, if TRUE indicates single device response if FALSE
 * 				indicates extended device response.
 * 				uint8_t StartIndex, start index of the child devices list.
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This function allows the Application to send ZDP network address request to
 * determine the 16-bit short address of the device whose IEEE address is known.
 *
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_zdp_send_nwk_addr_request(uint8_t * pIEEEAddrOfInterest, BOOL RequestType, uint8_t StartIndex)
{
  int16_t ret_val;
  uint8_t pkt_size = 10;
  getZDPNWKShortAddrFrameSnd get_zdp_addr;

  pkt_size = sizeof(getZDPNWKShortAddrFrameSnd);

  rsi_zigb_mcpy(pIEEEAddrOfInterest, (uint8_t *)&get_zdp_addr.ieee_Addr, 8);

  get_zdp_addr.RequestType = RequestType;
  get_zdp_addr.StartIndex = StartIndex;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetZDPNWKShortAddr, (uint8_t *)&get_zdp_addr, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_zdp_send_ieee_addr_request(uint16_t shortAddress,
 * 					BOOL RequestType, uint8_t StartIndex, BOOL APSAckRequired)
 * 					
 *
 * @brief       Sends the NWK Address request command to ZigBee module
 *
 * @param[in]   uint16_t shortAddress, shortAddress whose 64-bit extended address
 * 				is to be determined.
 * 				BOOL RequestType, if TRUE indicates single device response if FALSE
 * 				indicates extended device response.
 * 				uint8_t StartIndex, start index of the child devices list.
 * 				BOOL APSAckRequired, if TRUE indicates APS ack is required if FALSE
 * 				indicates APS ack is not required
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send ZDP IEEE address request to determine the 64-bit 
 * IEEE address of the device whose short address is known.
 *
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_zdp_send_ieee_addr_request(uint16_t shortAddress, BOOL RequestType,uint8_t StartIndex, BOOL APSAckRequired)
{
  int16_t ret_val;
  uint8_t pkt_size = 5;
  getZDPIEEEAddrFrameSnd get_zdp_ieee_addr;

  pkt_size = sizeof(getZDPIEEEAddrFrameSnd);

  rsi_zigb_uint16to_buffer(shortAddress, (uint8_t *)&get_zdp_ieee_addr.ShortAddr);

  get_zdp_ieee_addr.RequestType = RequestType;
  get_zdp_ieee_addr.StartIndex = StartIndex;
  get_zdp_ieee_addr.APSAckRequired = APSAckRequired;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetZDPIEEEAddr, (uint8_t *)&get_zdp_ieee_addr, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_zdp_send_device_announcement(void)
 *
 * @brief       Sends the Device Announcement command to ZigBee module
 *
 * @param[in]   none
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send a broadcast for a ZDO Device announcement. Normally, 
 * it is NOT required to call this as the stack automatically sends a device 
 * announcement during joining or rejoing, as per the spec. However if the device
 * wishes to re-send its device announcement they can use this call.
 * 
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_zdp_send_device_announcement(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameZDPDevAnnounce, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_active_endpoints_request(uint16_t shortAddress,
 * 						uint8_t APSAckRequired)
 *
 * @brief       Sends the Active Endpoint Request command to ZigBee module
 *
 * @param[in]   uint16_t shortAddress, Device short Address whose active endpoints
 * 				need to be obtained.
 * 				uint8_t APSAckRequired, if TRUE indicates APS ack is required if FALSE
 * 				indicates APS ack is not required
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send a Active Endpoint request for the specified short address. 
 * 
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_active_endpoints_request(uint16_t shortAddress, uint8_t APSAckRequired)
{
  int16_t ret_val;
  uint8_t pkt_size = 3;
  activeEPOfShortAddrFrameSnd act_ep_req;

  pkt_size = sizeof(activeEPOfShortAddrFrameSnd);
  
  rsi_zigb_uint16to_buffer(shortAddress, (uint8_t *)&act_ep_req.ShortAddr);

  act_ep_req.APSAckRequired = (uint8_t)APSAckRequired;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameActiveEPreq, (uint8_t *)&act_ep_req, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_zdp_send_power_descriptor_request(uint16_t shortAddress,
 * 						uint8_t APSAckRequired)
 *
 * @brief       Sends the power descriptor request command to ZigBee module
 *
 * @param[in]   uint16_t shortAddress, Device short Address whose active endpoints
 * 				need to be obtained.
 * 				uint8_t APSAckRequired, if TRUE indicates APS ack is required if FALSE
 * 				indicates APS ack is not required
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send a Power Descriptor request for the specified short address. 
 * 
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_zdp_send_power_descriptor_request(uint16_t shortAddress, uint8_t APSAckRequired)
{
  int16_t ret_val;
  uint8_t pkt_size = 3;
  powerDescFrameSnd pwr_desc_req;

  pkt_size = sizeof(powerDescFrameSnd);

  rsi_zigb_uint16to_buffer(shortAddress, (uint8_t *)&pwr_desc_req.ShortAddr);

  pwr_desc_req.APSAckRequired = (uint8_t)APSAckRequired;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameZDPPwrDesc, (uint8_t *)&pwr_desc_req, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_zdp_send_node_descriptor_request(uint16_t shortAddress,
 * 						uint8_t APSAckRequired)
 *
 * @brief       Sends the Node descriptor request command to ZigBee module
 *
 * @param[in]   uint16_t shortAddress, Device short Address whose active endpoints
 * 				need to be obtained.
 * 				uint8_t APSAckRequired, if TRUE indicates APS ack is required if FALSE
 * 				indicates APS ack is not required
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send a Node Descriptor request for the specified short address. 
 * 
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_zdp_send_node_descriptor_request(uint16_t shortAddress, uint8_t APSAckRequired)
{
  int16_t ret_val;
  uint8_t pkt_size = 3;
  nodeDescFrameSnd node_desc_req;

  pkt_size = sizeof(nodeDescFrameSnd);

  rsi_zigb_uint16to_buffer(shortAddress, (uint8_t *)&node_desc_req.ShortAddr);

  node_desc_req.APSAckRequired = (uint8_t)APSAckRequired;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameZDPNodeDesc, (uint8_t *)&node_desc_req, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_simple_descriptor_request(uint16_t shortAddress,
 * 						uint8_t EndpointId)
 *
 * @brief       Sends the Simple descriptor request command to ZigBee module
 *
 * @param[in]   uint16_t shortAddress, Device short Address whose active endpoints
 * 				need to be obtained.
 * 				uint8_t EndpointId, Endpoint whose simple descriptor to be obtained.
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to get Simple Descriptor request from the specified short address. 
 * 
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_simple_descriptor_request(uint16_t shortAddress, uint8_t EndPointId)
{
  int16_t ret_val;
  uint8_t pkt_size = 3;
  getSimpleDescOfShortAddrFrameSnd get_simple_desc;

  pkt_size = sizeof(getSimpleDescOfShortAddrFrameSnd);

  rsi_zigb_uint16to_buffer(shortAddress, (uint8_t *)&get_simple_desc.ShortAddr);

  get_simple_desc.EndPointId = (uint8_t)EndPointId;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSimpleDescReq, (uint8_t *)&get_simple_desc, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_send_unicast_data(ZigBee_Outgoing_Msg_Type msgType,
 * 						Address DestAddress, ZigBeeAPSDEDataRequest_t *pAPSDERequest)
 *
 * @brief       Sends the unicast data command to ZigBee module
 *
 * @param[in]   ZigBee_Outgoing_Msg_Type msgType, outgoing message type.
 * 				Address DestAddress, union of 64-bit extended address and 16-bit 
 * 				short address
 * 				ZigBeeAPSDEDataRequest_t *pAPSDERequest, pointer to APSDE_Data_Request_t
 * 				structure
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send a simple unicast data for the specified short address. 
 * 
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_send_unicast_data(ZigBee_Outgoing_Msg_Type msgType, Address DestAddress, ZigBeeAPSDEDataRequest_t *pAPSDERequest)
{
  int16_t ret_val;
  uint8_t i = 0, pkt_size = 0;
  unicastDataFrameSnd ucast_buf;

  pkt_size = (sizeof(unicastDataFrameSnd) - 120)+ pAPSDERequest->AsduLength;
  
  ucast_buf.msgType  = msgType;

  rsi_zigb_mcpy(DestAddress.IEEE_address, (uint8_t *)&ucast_buf.ieee_Addr, 8);

  ucast_buf.DestEndpoint = pAPSDERequest->DestEndpoint;
  ucast_buf.SrcEndpoint = pAPSDERequest->SrcEndpoint;

  rsi_zigb_uint16to_buffer(pAPSDERequest->ProfileId, (uint8_t *)&ucast_buf.ProfileId);

  rsi_zigb_uint16to_buffer(pAPSDERequest->ClusterId, (uint8_t *)&ucast_buf.ClusterId);

  ucast_buf.AsduLength = pAPSDERequest->AsduLength;
  ucast_buf.TxOptions = pAPSDERequest->TxOptions;
  ucast_buf.Radius = pAPSDERequest->Radius;

  for(i = 0; i < pAPSDERequest->AsduLength; i++)
    ucast_buf.Data[i] = pAPSDERequest->aPayload[i];

  /*Size of pkt is sizeof(unicastDataFrameSnd) + Desc_len + AsduLength - (length of Data pointer)*/

  /* Updating the length */

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameUcastData, (uint8_t *)&ucast_buf, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_send_unicast_data(GroupID GroupAddress, 
 * 						ZigBeeAPSDEDataRequest_t *pAPSDERequest)
 *
 * @brief       Sends the group data command to ZigBee module
 *
 * @param[in]   GroupID GroupAddress, union of 64-bit extended address and 16-bit 
 * 				short address
 * 				ZigBeeAPSDEDataRequest_t *pAPSDERequest, pointer to APSDE_Data_Request_t
 * 				structure
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send a group data for the specified short address. 
 * 
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_send_group_data( GroupID GroupAddress, ZigBeeAPSDEDataRequest_t * pAPSDERequest)
{
  int16_t ret_val;
  uint8_t i = 0, pkt_size = 0;
  groupDataFrameSnd group_buf;

  pkt_size = (sizeof(groupDataFrameSnd) - 120)+ pAPSDERequest->AsduLength;
  group_buf.group_addr = GroupAddress;

  group_buf.DestEndpoint = pAPSDERequest->DestEndpoint;
  group_buf.SrcEndpoint = pAPSDERequest->SrcEndpoint;

  rsi_zigb_uint16to_buffer(pAPSDERequest->ProfileId, (uint8_t *)&group_buf.ProfileId);

  rsi_zigb_uint16to_buffer(pAPSDERequest->ClusterId, (uint8_t *)&group_buf.ClusterId);

  group_buf.AsduLength = pAPSDERequest->AsduLength;
  group_buf.TxOptions = pAPSDERequest->TxOptions;
  group_buf.Radius = pAPSDERequest->Radius;

  for(i = 0; i < pAPSDERequest->AsduLength; i++)
    group_buf.Data[i] = pAPSDERequest->aPayload[i];

  /*Size of pkt is sizeof(unicastDataFrameSnd) + Desc_len + AsduLength - (length of Data pointer)*/

  /* Updating the length */

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGroupData, (uint8_t *)&group_buf, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_send_broadcast_data(ZigBeeAPSDEDataRequest_t 
 * 																*pAPSDERequest)
 *
 * @brief       Sends the Broadcast data command to ZigBee module
 *
 * @param[in]   ZigBeeAPSDEDataRequest_t *pAPSDERequest, pointer to APSDE_Data_Request_t
 * 				structure
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send a broadcast data for the specified short address. 
 * 
 * @section Prerequisite
 * The device should be part of the network to transmit this command successfully. 
 *
 */
int16_t rsi_zigb_send_broadcast_data(ZigBeeAPSDEDataRequest_t * pAPSDERequest)
{
  int16_t ret_val;
  uint8_t i = 0, pkt_size = 0;
  bcastDataFrameSnd bcast_buf;
  pkt_size = (sizeof(bcastDataFrameSnd) - 120)+ pAPSDERequest->AsduLength;


  bcast_buf.DestEndpoint = pAPSDERequest->DestEndpoint;
  bcast_buf.SrcEndpoint = pAPSDERequest->SrcEndpoint;

  rsi_zigb_uint16to_buffer(pAPSDERequest->ProfileId, (uint8_t *)&bcast_buf.ProfileId);

  rsi_zigb_uint16to_buffer(pAPSDERequest->ClusterId, (uint8_t *)&bcast_buf.ClusterId);

  bcast_buf.AsduLength = pAPSDERequest->AsduLength;
  bcast_buf.TxOptions = pAPSDERequest->TxOptions;
  bcast_buf.Radius = pAPSDERequest->Radius;

  for(i = 0; i < pAPSDERequest->AsduLength; i++)
    bcast_buf.Data[i] = pAPSDERequest->aPayload[i];

  /*Size of pkt is sizeof(unicastDataFrameSnd) + Desc_len + AsduLength - (length of Data pointer)*/

  /* Updating the length */

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameBcastData, (uint8_t *)&bcast_buf, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_max_aps_payload_length(void)
 *
 * @brief       Sends the get maximum payload length command to ZigBee module
 *
 * @param[in]   none
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to get the maximum payload length Set in ZigBee stack.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_get_max_aps_payload_length(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetMaxAPSSize, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_binding_indices(void)
 *
 * @brief       Sends the get active binding indices command to ZigBee module
 *
 * @param[in]   none
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to get the active binding indices in ZigBee stack.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_get_binding_indices(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetBindingIndices, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_binding_entry(ZDP_Bind_Request_t *pSetBindingEntry)
 *
 * @brief       Sends the set binding entry command to ZigBee module
 *
 * @param[in]   ZDP_Bind_Request_t *pSetBindingEntry, pointer loaction to binding entry
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to set the binding entry in ZigBee stack.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_set_binding_entry(ZDP_Bind_Request_t * pSetBindingEntry)
{
  int16_t ret_val;
  uint8_t pkt_size = 21;
  setBindEntryFrameSnd bind_entry;

  pkt_size = sizeof(setBindEntryFrameSnd);

  rsi_zigb_mcpy(pSetBindingEntry->a_src_addr, (uint8_t *)&bind_entry.SrcIEEEAddr, 8);

  bind_entry.SrcEndpoint = pSetBindingEntry->src_endpoint;

  rsi_zigb_mcpy(pSetBindingEntry->a_cluster_id,(uint8_t *)&bind_entry.ClusterId, 2);

  bind_entry.DestAddrMode = pSetBindingEntry->dest_addr_mode;

  rsi_zigb_mcpy(pSetBindingEntry->a_dest_addr,(uint8_t *)&bind_entry.DestIEEEAddr, 8);

  bind_entry.DestEndpoint = pSetBindingEntry->dest_endpoint;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetBindEntry, (uint8_t *)&bind_entry, pkt_size);

  return ret_val;
}


/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_delete_binding(uint8_t bindIndex)
 *
 * @brief       Sends the delete bind entry command to ZigBee module
 *
 * @param[in]   bindIndex, index whose entry to be deleted
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to delete the binding entry for the specified index in ZigBee stack.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_delete_binding(uint8_t BindIndex)
{
  int16_t ret_val;
  uint8_t pkt_size = 1;
  delBindEntryFrameSnd del_bind_entry;

  pkt_size = sizeof(delBindEntryFrameSnd);

  del_bind_entry.BindIndex = BindIndex;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameDelBindEntry, (uint8_t *)&del_bind_entry, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_is_binding_entry_active(uint8_t bindIndex)
 *
 * @brief       Sends the bind entry active command to ZigBee module
 *
 * @param[in]   bindIndex, index of the binding entry.
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to check the binding entry is active or not for the given index.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_is_binding_entry_active(uint8_t BindIndex)
{
  int16_t ret_val;
  uint8_t pkt_size = 1;
  isBindEntryActiveFrameSnd bind_entry_buf;

  pkt_size = sizeof(isBindEntryActiveFrameSnd);

  bind_entry_buf.BindIndex = BindIndex;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameIsBindEntryActive, (uint8_t *)&bind_entry_buf, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_clear_binding_table(void)
 *
 * @brief       Sends the clear bind table command to ZigBee module
 *
 * @param[in]   bindIndex, index of the binding entry.
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to clear the binding table.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_clear_binding_table(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameClearBindTable, NULL, 0);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_bind_request
 *
 * @brief       Sends the Binding Request command to ZigBee module
 *
 * @param[in]  
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to create the binding entry between pair of devices
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_bind_request(uint16_t shortAddress, uint8_t *pIEEEAddrOfSource, uint8_t sourceEndpoint, uint16_t ClusterId, uint8_t destAddrMode,
    Address destAddress, uint8_t destinationEndpoint, BOOL APSAckRequired)
{
  int16_t ret_val;
  uint8_t pkt_size = 0;
  bindReqFrameSnd bind_req;

  pkt_size = sizeof(bindReqFrameSnd);
  
  rsi_zigb_uint16to_buffer(shortAddress, (uint8_t *)&bind_req.SrcShortAddr);

  rsi_zigb_mcpy(pIEEEAddrOfSource, (uint8_t *)&bind_req.SrcIEEEAddr, 8);

  bind_req.SrcEndpoint = sourceEndpoint;

  rsi_zigb_uint16to_buffer(ClusterId, (uint8_t *)&bind_req.ClusterId);

  bind_req.DestAddrMode = destAddrMode;

  if( (destAddrMode == 1) || (destAddrMode == 2)) {
    rsi_zigb_mcpy((uint8_t *)&destAddress.short_address, (uint8_t *)&bind_req.DestAddress, 2);
  }

  if(destAddrMode == 3) {
    rsi_zigb_mcpy(destAddress.IEEE_address, (uint8_t *)&bind_req.DestAddress, 8);
  }

  bind_req.DestEndpoint = destinationEndpoint;

  bind_req.APSAckRequired = (uint8_t)APSAckRequired;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameBindReq, (uint8_t *)&bind_req, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_enddevice_bind_request(uint8_t EndPointId,
 * 								BOOL APSAckRequired)
 *
 * @brief       Sends the End Device bind request command to ZigBee module
 *
 * @param[in]   uint8_t EndPointId, Endpoint to which the bindind to be done.
 * 				BOOL APSAckRequired, if TRUE indicates APS ack is required if FALSE
 * 				indicates APS ack is not required
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to initiate ZigBee Binding Request . By default the EndDevice Binding 
 * request timeout is configured to 10 seconds.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_enddevice_bind_request(uint8_t EndPointId, BOOL APSAckRequired)
{
  int16_t ret_val;
  uint8_t pkt_size = 2;
  endDevBindFrameSnd enddev_bind;

  pkt_size = sizeof(endDevBindFrameSnd);
  
  enddev_bind.EndPointId = EndPointId;
  enddev_bind.APSAckRequired = APSAckRequired;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameEndDevBind, (uint8_t *)&enddev_bind, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_unbind_request
 *
 * @brief       Sends the unbind request command to ZigBee module
 *
 * @param[in]   
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to remove the binding entry between pair of devices
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_unbind_request(uint16_t shortAddress, uint8_t *pIEEEAddrOfSource, uint8_t sourceEndpoint, uint16_t ClusterId, uint8_t destAddrMode,
    Address destAddress, uint8_t destinationEndpoint, BOOL APSAckRequired)
{
  int16_t ret_val;
  uint8_t pkt_size = 0;
  unbindReqFrameSnd unbind_buf;

  pkt_size = sizeof(unbindReqFrameSnd);

  rsi_zigb_uint16to_buffer(shortAddress, (uint8_t *)&unbind_buf.SrcShortAddr);

  rsi_zigb_mcpy(pIEEEAddrOfSource, (uint8_t *)&unbind_buf.SrcIEEEAddr, 8);

  unbind_buf.SrcEndpoint = sourceEndpoint;

  rsi_zigb_uint16to_buffer(ClusterId, (uint8_t *)&unbind_buf.ClusterId);

  unbind_buf.DestAddrMode = destAddrMode;

  if( (destAddrMode == 1) || (destAddrMode == 2))	{
    rsi_zigb_mcpy((uint8_t *)&destAddress.short_address, (uint8_t *)&unbind_buf.DestAddress, 2);
  }

  if(destAddrMode == 3) {
    rsi_zigb_mcpy(destAddress.IEEE_address, (uint8_t *)&unbind_buf.DestAddress, 8);
  }

  unbind_buf.DestEndpoint = destinationEndpoint;

  unbind_buf.APSAckRequired = (uint8_t)APSAckRequired;


  /* Updating the length */

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameUnbind, (uint8_t *)&unbind_buf, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_simple_descriptor(uint8_t endpointId, 
 * 						Simple_Descriptor_t *SimpleDesc)
 *
 * @brief       Sends the set simple descriptor command to ZigBee
 *
 * @param[in]   uint8_t endpointId, endpoint to which simple descriptor need to store
 * 				Simple_Descriptor_t *SimpleDesc, pointer of Simple descriptor to store
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to set the simple descriptor for the given endpoint.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_set_simple_descriptor(uint8_t EndPointId, Simple_Descriptor_t *SimpleDesc)
{
  int16_t ret_val;
  uint8_t pkt_size = 0,in_cluster_offset = 0;
  setSimpleDescFrameSnd set_simple_desc; 

  pkt_size = sizeof(setSimpleDescFrameSnd);

  set_simple_desc.EndPointId = EndPointId;

  rsi_zigb_uint16to_buffer(SimpleDesc->app_profile_id, (uint8_t *)&set_simple_desc.ProfileId);

  rsi_zigb_uint16to_buffer(SimpleDesc->app_device_id, (uint8_t *)&set_simple_desc.DevId);

  set_simple_desc.DevVersion = SimpleDesc->app_device_version;

  set_simple_desc.InClusterCnt = SimpleDesc->incluster_count;
  
  set_simple_desc.OutClusterCnt = SimpleDesc->outcluster_count;
  rsi_zigb_mcpy((uint8_t *)SimpleDesc->p_incluster_list,  (uint8_t *)&set_simple_desc.ClusterInfo, (SimpleDesc->incluster_count * sizeof(cluster_id_t)));
  
  in_cluster_offset = (SimpleDesc->incluster_count * sizeof(cluster_id_t));
  rsi_zigb_mcpy((uint8_t *)SimpleDesc->p_outcluster_list, (uint8_t *)&set_simple_desc.ClusterInfo[in_cluster_offset], (SimpleDesc->outcluster_count * sizeof(cluster_id_t)));

  
  /* Updating the length */
  pkt_size = pkt_size - (sizeof(set_simple_desc.ClusterInfo) - ((SimpleDesc->incluster_count + SimpleDesc->outcluster_count) * 2));

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetSimpleDesc, (uint8_t *)&set_simple_desc, pkt_size);

  return ret_val;
} 

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_send_match_descriptors_request
 *
 * @brief       Sends the Match Descriptor Request command to ZigBee module
 *
 * @param[in]   
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to send the Match Descriptor Request.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_send_match_descriptors_request(uint16_t shortAddress, uint16_t ProfileId, uint8_t *InClusterList, uint8_t InClusterCnt,
    uint8_t *OutClusterList, uint8_t OutClusterCnt, BOOL APSAckRequired,uint16_t dstAddress)

{
  int16_t ret_val;
  uint8_t pkt_size = 0;
  uint8_t in_cluster_offset= 0;
  sendMatchDescFrameSnd match_desc;

  pkt_size = sizeof(sendMatchDescFrameSnd);

  rsi_zigb_uint16to_buffer(shortAddress, (uint8_t *)&match_desc.ShortAddr);

  rsi_zigb_uint16to_buffer(ProfileId, (uint8_t *)&match_desc.ProfileId);

  match_desc.InClusterCnt = InClusterCnt;

  match_desc.OutClusterCnt = OutClusterCnt;
  match_desc.APSAckRequired = APSAckRequired;

  rsi_zigb_uint16to_buffer(dstAddress, (uint8_t *)&match_desc.DestAddress);
  
  rsi_zigb_mcpy(InClusterList,  (uint8_t *)&match_desc.ClusterInfo, (InClusterCnt * sizeof(cluster_id_t)));
  in_cluster_offset = InClusterCnt * sizeof(cluster_id_t);
  rsi_zigb_mcpy(OutClusterList, (uint8_t *)&match_desc.ClusterInfo[in_cluster_offset], (OutClusterCnt * sizeof(cluster_id_t)));



  /* Updating the length */
  pkt_size = pkt_size - sizeof(match_desc.ClusterInfo) +
                        (sizeof(cluster_id_t) * (InClusterCnt + OutClusterCnt));
       
  
  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSendMatchDesc, (uint8_t *)&match_desc, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_address_map_table_entry(uint8_t index)
 *
 * @brief       Sends the get the address map table entry command to ZigBee
 *
 * @param[in]   Index, index of the address map table entry
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to get the address map table entry for the specified index
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_get_address_map_table_entry(uint8_t Index)
{
  uint8_t pkt_size = 1;
  int16_t ret_val;
  getAddrMapTableFrameSnd addr_map;

  pkt_size = sizeof(getAddrMapTableFrameSnd);

  addr_map.Index = (uint8_t)Index;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetAddrMapTable, (uint8_t *)&addr_map, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_key(Security_Key_Types keytype)
 *
 * @brief       Sends the Get key command to ZigBee module
 *
 * @param[in]   Security_Key_Types keytype, type of the key to get
 *
 * @param[out]  none
 *
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 *
 * @section description 
 * This API is used to get the specified key and its associated data. This can retrieve
 * the Link Key, Current Network Key, or Next Network Key.
 * 
 * @section Prerequisite
 *
 */
int16_t rsi_zigb_get_key(Security_Key_Types KeyType)
{
  int16_t ret_val;
  uint8_t pkt_size = 1;
  getKeyFrameSnd get_key;

  pkt_size = sizeof(getKeyFrameSnd);

  get_key.KeyType = KeyType;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetKey, (uint8_t *)&get_key, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_request_link_key(uint8_t *trustCentreIEEEAddr,  
 * 						uint8_t *partnerIEEEAddr)
 * @brief       Sends the Reuqest link key command to ZigBee module
 * @param[in]   trustCentreIEEEAddr - Trust center 64-bit extended address
 * 				partnerIEEEAddr - address of the child
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get trust center linkkey from our device so that we are
 * securing our messages  sent to the child joined
 *
 */
int16_t rsi_zigb_have_link_key(uint8_t *pRemoteDeviceIEEEAddr)
{
  int16_t ret_val;
  uint8_t i = 0, pkt_size = 8;
  haveLinkKeyFrameSnd link_key;

  pkt_size = sizeof(haveLinkKeyFrameSnd);

  for(i =0; i < 8; i++) {
    link_key.ieee_Addr[i] = pRemoteDeviceIEEEAddr[i];
  }

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameHaveLinkKey, (uint8_t *)&link_key, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_request_link_key(uint8_t *trustCentreIEEEAddr,  
 * 						uint8_t *partnerIEEEAddr)
 * @brief       Sends the Reuqest link key command to ZigBee module
 * @param[in]   trustCentreIEEEAddr - Trust center 64-bit extended address
 * 				partnerIEEEAddr - address of the child
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get trust center linkkey for the child joined
 *
 */
int16_t rsi_zigb_request_link_key(uint8_t* TrustCenterIEEEAddr, uint8_t* PartnerIEEEAddr)
{
  int16_t ret_val;
  uint8_t pkt_size = 16;
  reqLinkKeyFrameSnd req_link_key;

  pkt_size = sizeof(reqLinkKeyFrameSnd);

  rsi_zigb_mcpy(TrustCenterIEEEAddr, (uint8_t *)&req_link_key.TrustCenterIEEEAddr, 8);

  rsi_zigb_mcpy(PartnerIEEEAddr, (uint8_t *)&req_link_key.PartnerIEEEAddr, 8);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameReqLinkKey, (uint8_t *)&req_link_key, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_get_key_table_entry(uint8_t index,  
 * 						ZigBeeKeyStructure_t *KeyStruct)
 * @brief       Sends the get key table entry command to ZigBee module
 * @param[in]   Index - index to set the key
 * 				KeyStruct - Key Structure to update
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to get the key configuration for the given index
 *
 */
int16_t rsi_zigb_get_key_table_entry (uint8_t Index, ZigBeeKeyStructure_t *keyStruct)
{
  int16_t ret_val;
  getKeyTableFrameSnd key_table;
  uint8_t pkt_size = 1;

  pkt_size = sizeof(getKeyTableFrameSnd);

  key_table.Index = Index;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameGetKeyTable, (uint8_t *)&key_table, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_key_table_entry(uint8_t index, uint8_t 
 * 						*IeeeAddr, Bool linkkey, uint8_t *pKeyData)
 * @brief       Sends the set key table entry command to ZigBee module
 * @param[in]   Index - index to set the key
 * 				IeeeAddr - 64-bit extended addreess
 *              linkkey -  Checking for Link key or not
 *				pKeyData - key to update
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to set the key configuration in the given index.
 *
 */
int16_t rsi_zigb_set_key_table_entry( uint8_t Index, uint8_t * pIEEEAddress, BOOL LinkKey, uint8_t * pKeyData )
{
  int16_t ret_val;
  uint8_t pkt_size = 26;
  setKeyTableFrameSnd key_table;

  pkt_size = sizeof(setKeyTableFrameSnd);

  key_table.Index = Index;

  rsi_zigb_mcpy(pIEEEAddress, (uint8_t *)&key_table.ieee_Addr, 8);

  key_table.LinkKey = (uint8_t)LinkKey;

  rsi_zigb_mcpy(pKeyData, (uint8_t *)&key_table.KeyData, 16);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetKeyTable, (uint8_t *)&key_table, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_add_or_update_key_table_entry(uint8_t *IeeeAddr,
 * 						Bool linkkey, uint8_t *pKeyData, uint8_t Index)
 * @brief       Sends the key update command to ZigBee module
 * @param[in]   IeeeAddr - 64-bit extended addreess
 *              linkkey -  Checking for Link key or not
 *				pKeyData - key to update
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to add any new entry or update the already presented entries
 * in Key Table.
 *
 */
int16_t rsi_zigb_add_or_update_key_table_entry(uint8_t *pIEEEAddress, BOOL LinkKey, uint8_t *pKeyData, uint8_t *indx)
{
  int16_t ret_val;
  uint8_t pkt_size = 25;
  addKeyTableFrameSnd key_table;

  pkt_size = sizeof(addKeyTableFrameSnd);
  
  rsi_zigb_mcpy(pIEEEAddress, (uint8_t *)&key_table.ieee_Addr, 8);

  key_table.LinkKey = (uint8_t)LinkKey;

  rsi_zigb_mcpy(pKeyData, (uint8_t *)&key_table.KeyData, 16);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameAddKeyTable, (uint8_t *)&key_table, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_find_key_table_entry(uint8_t *IeeeAddr, Bool Linkkey)
 * @brief       Sends the find key table entry command to ZigBee module
 * @param[in]   IeeeAddr - 64-bit extended address
 *              Linkkey -  Checking for Link key
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to find the table entry using on 64-bit extended address
 *
 */
int16_t rsi_zigb_find_key_table_entry(uint8_t * pIEEEAddress, BOOL LinkKey)
{
  int16_t ret_val;
  findKeyTableFrameSnd key_table;
  uint8_t pkt_size = 9;

  pkt_size = sizeof(findKeyTableFrameSnd);

  rsi_zigb_mcpy(pIEEEAddress, (uint8_t *)&key_table.ieee_Addr, 8);

  key_table.LinkKey = (uint8_t)LinkKey;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameFindKeyTable, (uint8_t *)&key_table, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_erase_key_table_entry(uint8_t Index)
 * @brief       Sends the Key table deletion entry from ZigBee module
 * @param[in]   Index - key table Index to erase
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to erase the key table entry based on given Index
 *
 */
int16_t rsi_zigb_erase_key_table_entry(uint8_t Index)
{
  int16_t ret_val;
  uint8_t pkt_size = 1;
  eraseKeyTableFrameSnd key_table;

  pkt_size = sizeof(eraseKeyTableFrameSnd);

  key_table.Index = Index;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameEraseKeyTable, (uint8_t *)&key_table, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_set_extended_panid(uint8_t *ExPanId)
 * @brief       Sends the Extended PANID command to ZigBee module
 * @param[in]   exPanId - pointer to extended panID
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to set the extended panid in ZigBee Stack
 *
 */
int16_t rsi_zigb_set_extended_panid(uint8_t *ExPanId)
{
  int16_t ret_val;
  setExtPanIdFrameSnd ext_panid;
  uint8_t pkt_size = 8;

  pkt_size = sizeof(setExtPanIdFrameSnd);

  rsi_zigb_mcpy(ExPanId, (uint8_t *)&ext_panid.ExPanId, 8);

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameSetExtPanId, (uint8_t *)&ext_panid, pkt_size);

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_update_sas(Startup_Attribute_Set_t *Startup)
 * @brief       Sends the update SAS frame to Module
 * @param[in]   Startup - Startup Attribute Structure
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to update the startup Attributes in module.
 *
 */
int16_t rsi_zigb_update_sas(Startup_Attribute_Set_t *Startup)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameUpdateSAS, (uint8_t *)Startup, 
                                  sizeof(Startup_Attribute_Set_t));

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_update_zdo_configuration(ZDO_Configuration_Table_t *zdo_cnf)
 * @brief       Sends the update ZDO configuration to module
 * @param[in]   zdo_cnf - ZDO configuration structure
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to update the ZDO parameters avialble in ZigBee Stack.
 *
 * This API is called only after updating SAS parameters.
 *
 */
int16_t rsi_zigb_update_zdo_configuration(ZDO_Configuration_Table_t *zdo_cnf)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameUpdateZDO, (uint8_t *)zdo_cnf, 
                                  sizeof(ZDO_Configuration_Table_t));

  return ret_val;
}

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_deinit_stack(void)
 * @brief       Sends the ZigBee Stack de-initialization command to the ZigBee 
 * module
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to de-initialize the ZigBee stack. 
 *
 */
int16_t rsi_zigb_deinit_stack(void)
{
  int16_t ret_val;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_frameDeInitStack, NULL, 0);
  
  return ret_val;
}

/*===========================================================================
 *
 * @fn          rsi_zigb_send_pwrmode (uint8_t ps_en,uint8_t deep_sleep_wakeup_period ,uint8_t slp_mode) 
 * @brief       Sends the ZigBee power mode command to ZigBee Module
 * @param[in]   ps_en: 0->Disable,1->Enable
 *              deep_sleep_wakeup_period: deep sleep wkp period in sec.
 *              slp_mode: 1->LP_mode,2-> ULP_mode 
 * @param[out]  none
 * @return      errCode
 *              -2 = Command execution failure
 *              -1 = Buffer Full
 *              0  = SUCCESS
 * @section description 
 * This API is used to send power mode cmd to the ZigBee Fw. 
 *
 */
int16_t rsi_zigb_send_pwrmode (uint8_t ps_en,uint8_t deep_sleep_wakeup_period ,uint8_t slp_mode)
{
  uint8_t pkt_size;
  int16_t ret_val;
  pwrModeFrameSnd pwr_mode;

  pkt_size = sizeof(pwrModeFrameSnd);
  pwr_mode.ps_en = ps_en;
  pwr_mode.deep_sleep_wkp_period = deep_sleep_wakeup_period;
  pwr_mode.slp_mode = slp_mode;

  ret_val = rsi_zigb_execute_cmd((uint8_t *)rsi_zigb_framePwrMode, (uint8_t *)&pwr_mode, pkt_size);

  return ret_val;
}

