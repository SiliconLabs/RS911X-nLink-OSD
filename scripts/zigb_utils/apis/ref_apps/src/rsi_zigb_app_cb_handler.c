/**
 *  @file     rsi_zigb_app_cb_handler.c
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
 *  @brief API: Definitions of various data and interface handlers
 *
 *  @section Description
 *  This file contain definitions for different ZigBee INTERFACE callbacks. 
 *  These are various data and mgmt handler functions
 *
 *  @section Improvements
 *
 */


/**
 * Includes
 * */
 

#include "rsi_zigb_types.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_interfaces.h"
#include "rsi_zigb_app_sm.h"
#include "rsi_zigb_app.h"
#include "rsi_zigb_callbacks.h"
#ifdef LINUX_PLATFORM
#include "stdio.h"
#endif

extern rsi_zigb_app_info_t rsi_zigb_app_info;
extern rsi_zigb_app_cb_t rsi_zigb_app_cb;
extern uint16_t matchDescReqSent;
///////////////Call back Handlers called from Stack. These are to be filled by App developer

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_scan_complete_handler (uint32_t channel, 
 *                                                       uint8_t status )
 * @brief       Scan complete handler 
 * @param[in]   Channel
 * @param[in]   Status of channel whether beacons are found
 * @return      none
 * @section description
 * This API is used to handle ZigBee scan complete state
 * It provides infromation of the channel whether beacons are found or not
 * Updating few app_info variables 
 *
 * ===========================================================================*/
  
void rsi_zigb_app_scan_complete_handler ( uint32_t channel, uint8_t status )
{
  rsi_zigb_app_info.scan_done_cb.channel = channel; 
  rsi_zigb_app_info.scan_done_cb.scan_status = status; 
  rsi_zigb_app_info.status_var.scanReqSent = 0;
#ifdef ENABLE_DEBUG_PRINTS  
  RSI_DPRINT(RSI_PL1,"\n Called AppScanCompleteHandler \n ");
#endif  
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_energy_scan_result_handler( uint32_t channel,
 *                                                       uint8_t *pEnergyValue)
 * @brief       Energy Scan complete handler 
 * @param[in]   Channel
 * @param[in]   Energy Value (RSSI)
 * @return      none
 * @section description
 * This API is used to handle ZigBee Energy scan complete state
 * Here Energy in each channel is received, for the provided channels 
 * issued by user to scan
 *
 * ===========================================================================*/
void rsi_zigb_app_energy_scan_result_handler( uint32_t channel,uint8_t *pEnergyValue)
{
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_network_found_handler(ZigBeeNetworkDetails)
 * @brief       Network found indication handler 
 * @param[in]   NetworkInformation data 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle network found indication frame  
 * Infromation about the found network is updated 
 *
 * ===========================================================================*/

void rsi_zigb_app_network_found_handler(ZigBeeNetworkDetails networkInformation)
{
  ZigBeeNetworkDetails *nwk_details = &(rsi_zigb_app_info.nwkinfo);
  rsi_zigb_app_cb.num_nwk_found++;
  /* Currently we are checking for any coordinator, if you know the specific 
   * extended panid, then check here for specific panid */
  rsi_zigb_mcpy((uint8_t *)&networkInformation, (uint8_t *)nwk_details, sizeof(ZigBeeNetworkDetails));
#ifdef ENABLE_DEBUG_PRINTS  
  RSI_DPRINT(RSI_PL1,"\n ##Called APPNETWORKFOUNDHANDLER \n ");
  RSI_DPRINT(RSI_PL1,"\n ## channel: %d  ",networkInformation.channel);
  RSI_DPRINT(RSI_PL1,"\n ## short panid: %d  ",networkInformation.shortPanId);
  
#endif  
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_stack_status_handler(ZigBeeNWKStatusInfo *statusInfo)
 * @brief       Stack status Indication
 * @param[in]   Network status Information 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle network/stack status
 * Infromation about network status (If connection successful of failed) 
 *
 * ===========================================================================*/

void rsi_zigb_app_stack_status_handler(ZigBeeNWKStatusInfo *statusInfo)
{
  rsi_zigb_app_info.stack_status = *statusInfo;
#ifdef ENABLE_DEBUG_PRINTS  
  RSI_DPRINT(RSI_PL1,"\n Stack Status = %x \n", *statusInfo);
#endif  
  //rsi_zigb_state_machine(EVENT_NETWORK_CHANGE_STATUS);
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_incoming_many_to_one_route_req_handler(uint16_t SourceAddr,
 *                                                  uint8_t * pSrcIEEEAddr,uint8_t PathCost )
 * @brief       Many to one route request handler
 * @param[in]   Source short Addr
 * @param[in]   Source IEEE address
 * @param[in]   Path cost
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle Many to one route request
 * We have to decide which route to accept based on path cost 
 *
 * ===========================================================================*/

void rsi_zigb_app_incoming_many_to_one_route_req_handler( uint16_t SourceAddr, uint8_t * pSrcIEEEAddr,uint8_t PathCost )
{
#ifdef ENABLE_DEBUG_PRINTS  
  RSI_DPRINT(RSI_PL1,"\n Called rsi_zigb_app_incoming_many_to_one_route_req_handler \n ");
  RSI_DPRINT(RSI_PL1,"\n SorceAddr: 0x%x",SourceAddr);
  RSI_DPRINT(RSI_PL1,"\n PathCost: %x",PathCost);
#endif  
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_handle_data_indication(
 *                                   APSDE_Data_Indication_t * pDataIndication )
 * @brief       Handle data indication frame
 * @param[in]   Data indication info struct 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle received data indication frame
 *
 * ===========================================================================*/

void rsi_zigb_app_handle_data_indication(APSDE_Data_Indication_t *pDataIndication)
{
  rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;
  
  if( pDataIndication->cluster_id == 0x8003)//0x8003: power descriptor response
  { 
    if(pDataIndication->a_asdu[1] == 0x00)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
                     app_info->zb_resp_info.powerDescResp, 
                     pDataIndication->asdulength);
      app_info->status_var.powerDescRspStatus = 0x00;
    }
  }

  if( pDataIndication->cluster_id == 0x8002)//0x8003: node descriptor response
  {
    if(pDataIndication->a_asdu[1] == 0x00)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
                     app_info->zb_resp_info.nodeDescResp, 
                     pDataIndication->asdulength);
      app_info->status_var.nodeDescRspStatus = 0x00;
    }
  }

  if( pDataIndication->cluster_id == 0x8021)//0x8021: bind response
  {
    if(pDataIndication->a_asdu[1] == 0x84)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
                     app_info->zb_resp_info.bindResp, 
                     pDataIndication->asdulength);
      app_info->status_var.bindRspStatus = 0x00;
    }
  }

  if( pDataIndication->cluster_id == 0x8022)//0x8022: unbind response
  {
    if(pDataIndication->a_asdu[1] == 0x84)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
                     app_info->zb_resp_info.unbindResp, 
                     pDataIndication->asdulength);
      app_info->status_var.unbindRspStatus = 0x00;
    }
  }

  if( pDataIndication->cluster_id == 0x8005)//0x8005: active endpoint response
  {
    if(pDataIndication->a_asdu[1] == 0x00)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
                     app_info->zb_resp_info.actepResp, 
                     pDataIndication->asdulength);
      app_info->status_var.actepRspStatus = 0x00;
    }
  }

  if( pDataIndication->cluster_id == 0x8001)//0x8001: ieee addr response
  {
    if(pDataIndication->a_asdu[1] == 0x00)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
                     app_info->zb_resp_info.ieeeaddrResp, 
                     pDataIndication->asdulength);
      app_info->status_var.ieeeaddrRspStatus = 0x00;
    }
  }

  if( pDataIndication->cluster_id == 0x8004)//0x8004: simple desc response
  {
    if(pDataIndication->a_asdu[1] == 0x00)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
                     app_info->zb_resp_info.simpledespResp, 
                     pDataIndication->asdulength);
      app_info->status_var.simpledescRspStatus = 0x00;
    }
  }

  if( pDataIndication->cluster_id == 0x8000)//0x8000: network addr response
  {
    if(pDataIndication->a_asdu[1] == 0x00)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
                     app_info->zb_resp_info.networkaddrResp, 
                     pDataIndication->asdulength);
      app_info->status_var.networkaddrRspStatus = 0x00;
    }
  }

  if( pDataIndication->cluster_id == 0x8006)//0x8006: Match decs response
  {
    if(pDataIndication->a_asdu[1] == 0x00)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu, 
                     app_info->zb_resp_info.matchDescResp,
                     pDataIndication->asdulength);
      app_info->status_var.matchDescRspStatus = 0x00;
    }
  }
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_handle_data_confirmation (
 *                                   APSDE_Data_Confirmation_t* pDataConfirmation )
 * @brief       Handle data confirmation frame
 * @param[in]   Buffer Index of actual data from the pointer
 * @param[in]   Data confirmation info struct 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle received data confirmation frame for the 
 * data request sent
 *
 * ===========================================================================*/
#ifdef ENABLE_DEBUG_PRINTS
uint32_t dataConfcnt = 0;
#endif
void rsi_zigb_app_handle_data_confirmation (APSDE_Data_Confirmation_t *pDataConfirmation)
{
  APSDE_Data_Confirmation_t *data_cnf = &(rsi_zigb_app_info.data_conf);      
  rsi_zigb_mcpy((uint8_t *)pDataConfirmation, 
      (uint8_t *)data_cnf, sizeof(APSDE_Data_Confirmation_t));

#ifdef ENABLE_DEBUG_PRINTS  
  dataConfcnt++;
  RSI_DPRINT(RSI_PL1,"\ndata confirmation status = %x , cnt = %d \n", data_cnf->status, dataConfcnt);
#endif  

  rsi_zigb_app_info.status_var.dataConfWait = 0;
  rsi_zigb_app_cb.send_data = 1;
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_child_join_handler(uint16_t short_address,
 *                                                   BOOL joining)
 * @brief       Child join handler 
 * @param[in]   Short_addr of child
 * @param[in]   Status of child joining/leaving 
 * @return      none
 * @section description
 * This API is used to handle child join/leave status
 *
 * ===========================================================================*/

void rsi_zigb_app_child_join_handler(uint16_t short_address, BOOL joining)
{
  rsi_zigb_app_cb.short_addr = short_address;
#ifdef ENABLE_DEBUG_PRINTS  
  RSI_DPRINT(RSI_PL1,"\n Called rsi_zigb_app_child_join_handler \n ");
  RSI_DPRINT(RSI_PL1,"ShortAddr: 0x%x",short_address);
  RSI_DPRINT(RSI_PL1,"\n Joining: %x\n",joining);
#endif  
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_cb_handler(uint8_t cmd_id, uint8_t *buffer)
 * @brief       Handler for asyncronous data and interface pkts
 * @param[in]   cmd type
 * @param[in]   Buffer 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle different interface pkts
 * For eg: In this handler Scancomplete , network info/status, data Indication 
 * and Confirmation Pkts will be handled
 *
 * ===========================================================================*/

void rsi_zigb_app_cb_handler(uint8_t cmd_id, uint8_t *buffer)
{
  uint8_t i = 0;
  rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;
  
  switch(cmd_id)
  {
    case APPSCANCOMPLETEHANDLER:
    {
      uint32_t channel = *(uint32_t *)buffer;
      uint8_t status = *(buffer + 4);

      rsi_zigb_app_scan_complete_handler (channel, status);
    }
    break;

    case APPENERGYSCANRESULTHANDLER:
    {
      uint32_t channel = *(uint32_t *)buffer;
      uint8_t pEnergyValue[16];

      rsi_zigb_mcpy((buffer + 4), pEnergyValue, 16);

      rsi_zigb_app_energy_scan_result_handler(channel, pEnergyValue); 
    }
    break;

    case APPNETWORKFOUNDHANDLER:
    {
      ZigBeeNetworkDetails networkInformation;
      networkInformation.shortPanId = rsi_zigb_bytes2R_to_uint16(buffer);
      buffer += SHORT_PANID_SIZE;
      networkInformation.channel = *buffer++;

      rsi_zigb_mcpy(buffer, networkInformation.extendedPanId, EXTENDED_PANID_SIZE);
      buffer += EXTENDED_PANID_SIZE;

      networkInformation.stackProfile = *buffer++;
      networkInformation.nwkUpdateId = *buffer++;
      networkInformation.allowingJoining = (BOOL)*buffer++;

      rsi_zigb_app_network_found_handler(networkInformation);
    }
    break;

    case APPZIGBEESTACKSTATUSHANDLER:
    {
      ZigBeeNWKStatusInfo statusInfo;
      statusInfo = (ZigBeeNWKStatusInfo)*buffer;
      rsi_zigb_app_stack_status_handler(&statusInfo);
    }
    break;

    case APPINCOMINGMANYTOONEROUTEREQUESTHANDLER:
    {
      uint8_t pSrcIEEEAddr[8], PathCost;
      uint16_t SourceAddr;

      SourceAddr = rsi_zigb_bytes2R_to_uint16(buffer);
      buffer += 2;

      rsi_zigb_mcpy(buffer, pSrcIEEEAddr, EXTENDED_ADDR_SIZE);
      buffer += EXTENDED_ADDR_SIZE;

      PathCost = *buffer;

      rsi_zigb_app_incoming_many_to_one_route_req_handler(SourceAddr, pSrcIEEEAddr, PathCost);
    }
    break;

    case APPHANDLEDATAINDICATION:
    {  
      APSDE_Data_Indication_t pDataIndication;
      pDataIndication.dest_addr_mode = *buffer++;
      
      if(pDataIndication.dest_addr_mode == g_SHORT_ADDR_MODE_c) {
        pDataIndication.dest_address.short_address = rsi_zigb_bytes2R_to_uint16(buffer);
        buffer += 2;
      } else if(pDataIndication.dest_addr_mode == g_EXTENDED_ADDR_MODE_c) {
        for(i =0; i < 8; i++) {
          pDataIndication.dest_address.IEEE_address[i] = *buffer++;
        }
      }

      pDataIndication.dest_endpoint = *buffer++;
      pDataIndication.src_addr_mode = *buffer++;

      if(pDataIndication.src_addr_mode == g_SHORT_ADDR_MODE_c) {
        pDataIndication.src_address.short_address = rsi_zigb_bytes2R_to_uint16(buffer);
        buffer += 2;
      } else if(pDataIndication.src_addr_mode == g_EXTENDED_ADDR_MODE_c) {
        for(i =0; i < 8; i++) {
          pDataIndication.src_address.IEEE_address[i] = *buffer++;
        }
      }
      pDataIndication.src_endpoint = *buffer++;
      pDataIndication.profile_id = rsi_zigb_bytes2R_to_uint16(buffer);
      buffer += 2;

      pDataIndication.cluster_id = rsi_zigb_bytes2R_to_uint16(buffer);
      buffer += 2;

      pDataIndication.asdulength = *buffer++;
      pDataIndication.was_broadcast = *buffer++;
      pDataIndication.security_status = *buffer++;
      pDataIndication.link_quality = *buffer++;
      (pDataIndication.a_asdu) = app_info->zb_resp_info.asdu_pkt;

      for(i = 0; i < pDataIndication.asdulength; i++) {
        pDataIndication.a_asdu[i] = buffer[i];
      }
#ifndef ZB_API_TEST      
      rsi_zigb_app_handle_data_indication(&pDataIndication );
#else
      rsi_zigb_api_test_data_indication_handler(&pDataIndication);
#endif      
    }
    break;

    case APPHANDLEDATACONFIRMATION:
    {
      APSDE_Data_Confirmation_t DataConfirmation;
      DataConfirmation.dest_addr_mode = *buffer++;

      if(DataConfirmation.dest_addr_mode == g_SHORT_ADDR_MODE_c)
      {
        DataConfirmation.dest_address.short_address = rsi_zigb_bytes2R_to_uint16(buffer);
        buffer += 2;
      } else if(DataConfirmation.dest_addr_mode == g_EXTENDED_ADDR_MODE_c) {
        for(i =0; i < 8; i++) {
          DataConfirmation.dest_address.IEEE_address[i] = *buffer++;
        }
      }

      DataConfirmation.dest_endpoint = *buffer++;
      DataConfirmation.src_endpoint = *buffer++;
      DataConfirmation.status = *buffer++;

      rsi_zigb_app_handle_data_confirmation(&DataConfirmation);
    }
    break;

    case APPCHILDJOINHANDLER:
    {
      uint16_t Short_address = rsi_zigb_bytes2R_to_uint16(buffer);
      buffer += 2;
      BOOL Joining = *buffer;
      rsi_zigb_app_child_join_handler(Short_address, Joining);
    }
    break;

    default:
    break;
  }
}
