/**
 *  @file     zigb_main.c
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
 *  @brief API: ZigBee main function
 *
 *  @section Description
 *  This file describes how ZigBee is initiated, how it is
 *  forming/Joining a Network and how data is exchanged 
 *
 *  @section Improvements
 *
 */


/**
 * Includes
 * */
 
#include "rsi_zigb_app.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_interfaces.h"
#include "rsi_zigb_callbacks.h"
#include "rsi_zigb_app_sm.h"
#include "rsi_zigb_types.h"
#include "rsi_zigb_config.h"
#include "rsi_zigb_onoff.h"
#include "rsi_zigb_global.h"
#ifdef LINUX_PLATFORM
#include "stdio.h"
#include "platform_specific.h"
#endif

#ifdef ZB_API_TEST
#include "rsi_zigb_api_test.h"
rsi_zigb_apitest_t  rsi_zigb_apitest;
#endif

rsi_zigb_app_cb_t   rsi_zigb_app_cb;
rsi_zigb_app_info_t rsi_zigb_app_info;

/*===========================================================================
 *
 * @fn          int zigb_main()
 * @brief       Initiate ZigBee device
 * @param[in]   none
 * @param[out]  none
 * @return      Status
 * 
 * @section description
 * This is the main function which initiates the ZigBee Switch application. 
 * This application starts the device in ZigBee End_Device/Router/Coordinator mode
 * and will do the following actions.
 *
 *   1) Issues active scan
 *   2) Receives found networks though the AppNetworkFoundHandler(). 
 *   3) Joins the selected network
 *   4) Do the data transfer to the parent device.
 *   4) Receives the confirmation from the AppHandleDataConfirmation() handler
 *   5) This status will be stared to the main() using the variable "dataConfirm_status" 
 *   6) If the status is success then do the data tranafer again(repeats the steps from step 4)
 *
 * ===========================================================================*/
  
int zigb_main(uint32_t channel)
{
  rsi_zigb_app_cb_t     *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_app_info_t   *app_info = &rsi_zigb_app_info;
  ZigBeeNetworkDetails  *nwk_details = &(rsi_zigb_app_info.nwkinfo); 
  ZigBeeDeviceType      device_type = app_cb_ptr->device_type;
  RSI_ZB_STATUS         rsi_status = RSI_ZB_SUCCESS;
  uint8_t               cmd_type, intf_type;
  uint8_t               scan_attempts = 0;
  uint32_t              zb_channel    = 0;
  rsi_zigb_uCmdRsp      *resp;

#ifdef ZB_ONLY
  rsi_status = rsi_zigb_init_oper_mode();
  if (rsi_status != RSI_ZB_SUCCESS) {
#ifdef ENABLE_DEBUG_PRINTS    
    RSI_DPRINT(RSI_PL1,"Opermode Initialization failed \n");
#endif    
  }
#endif  
  zb_channel = channel;
  zb_channel =  1 << zb_channel;
  rsi_zb_app_init(zb_channel);
  app_cb_ptr->fsm_state = FSM_INIT_STACK; 
  rsi_zigb_init_stack();
  while (1) 
  {
    if (app_cb_ptr->pkt_pending == RSI_TRUE)
    {
      rsi_status = rsi_frame_read(app_cb_ptr->read_packet_buffer);
      if (rsi_status != RSI_ZB_SUCCESS)
      {
#ifdef ENABLE_DEBUG_PRINTS    
        RSI_DPRINT(RSI_PL1,"Error in dequeuing pkt \n");
#endif      
        return RSI_ZB_FAIL;
      }
#ifdef ENABLE_DEBUG_PRINTS    
      RSI_DPRINT(RSI_PL1,"PKT Parsing  \n");
#endif      
      resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);

      if (!resp) {
#ifdef ENABLE_DEBUG_PRINTS    
        RSI_DPRINT(RSI_PL1,"resp is NULL Error in processing pkt1 \n");
#endif      
        return RSI_ZB_FAIL;
      }

#if 0 //ZB_ONLY      
      if (resp == (rsi_zigb_uCmdRsp *)0xFE) {
#ifdef ENABLE_DEBUG_PRINTS    
        RSI_DPRINT(RSI_PL1,"OperMode Response \n");
#endif 
      }
#endif      

#ifdef ENABLE_DEBUG_PRINTS    
      RSI_DPRINT(RSI_PL1,"cmd_id : %x\n", resp->cmd_id);
      RSI_DPRINT(RSI_PL1,"intf_id : %x\n", resp->intf_id);
#endif      
      cmd_type = resp->cmd_id;
      intf_type = resp->intf_id;
      switch (app_cb_ptr->fsm_state)
      {
        case FSM_CARD_NOT_READY:
        {
#ifdef ZB_ONLY
          if (cmd_type == ZIGBEE_CARD_READY) 
          {
            break;
          } 
          if (cmd_type == ZIGBEE_OPER_MODE_RSP)
#else
          if (cmd_type == ZIGBEE_CARD_READY)
#endif          
          {
            app_cb_ptr->fsm_state = FSM_INIT_STACK; 
            rsi_zigb_init_stack();
            break;
          }
          goto UNKNOWN_STATE;
        }
        break;

        case FSM_INIT_STACK:
        {
#ifdef ENABLE_DEBUG_PRINTS    
          RSI_DPRINT(RSI_PL1,"FSM_INIT_STACK\n");
#endif              
          if (cmd_type == ZIGBEESTACKINIT)
          {
            app_cb_ptr->fsm_state = FSM_RESET_STACK; 
            rsi_zigb_reset_stack();
#if 0            
            /* Get the self 64 bit address */
            rsi_zigb_get_self_ieee_address();
            app_cb_ptr->fsm_state = FSM_GET_DEV_TYPE; 
#endif            
            break;
          }
          goto UNKNOWN_STATE;
        }

        case FSM_RESET_STACK:
        {
          if (cmd_type == ZIGBEESTACKRESET)
          {
#ifdef ENABLE_DEBUG_PRINTS    
          RSI_DPRINT(RSI_PL1,"FSM_RESET_STACK\n");
#endif              
            rsi_zigb_update_sas(&Startup_Attribute_Set_Default); 
            break;
          }
          else if (cmd_type == ZIGBEEUPDATESAS) 
          {
            rsi_zigb_update_zdo_configuration(&g_Table_Default);
            break;
          }
          else if (cmd_type == ZIGBEEUPDATEZDO) 
          {
            /* Get the self 64 bit address */
            rsi_zigb_get_self_ieee_address();
            app_cb_ptr->fsm_state = FSM_GET_DEV_TYPE; 
            break;
          }
          goto UNKNOWN_STATE;
        }
        break;

        case FSM_GET_DEV_TYPE:
        {
          if (cmd_type == ZIGBEEGETSELFIEEEADDRESS)
          {
            rsi_zigb_mcpy(resp->uCmdRspPayLoad.IEEEAddrResp.Addr,
                app_cb_ptr->mac_addr, 8);
            /* Get the Device Type */
            rsi_zigb_get_device_type();
            break;
          }
          else if (cmd_type == ZIGBEEGETDEVICETYPE)
          {
            device_type = resp->uCmdRspPayLoad.GetDevResp.type; 
            app_cb_ptr->device_type = device_type;
            if((device_type == ZigBeeEndDevice) || 
                (device_type == ZigBeeRouter)) {
#ifdef ENABLE_DEBUG_PRINTS    
              RSI_DPRINT(RSI_PL1,"Send  power save command\n");
#endif              
              app_cb_ptr->fsm_state = FSM_ZB_PS_CMD;
              rsi_zigb_send_pwrmode(PWR_MODE,DEEPSLEEP_WKP_PERIOD,SLP_TYPE);
            } else {
              app_cb_ptr->fsm_state = FSM_INIT_COORDINATOR; 
#ifdef ENABLE_DEBUG_PRINTS        
              RSI_DPRINT(RSI_PL1,"Channel:%d \n",app_cb_ptr->channel);
#endif   
	      rsi_zigb_form_network(app_cb_ptr->channel, app_cb_ptr->power, 
                  app_cb_ptr->mac_addr);
            }
            break;
          }
          goto UNKNOWN_STATE;
        }
        break;

        case FSM_ZB_PS_CMD:
        {
          if (cmd_type == ZIGBEEINITPS)
          {
            rsi_status = resp->uCmdRspPayLoad.SetSimpleDescResp.status; 
            if( rsi_status == RSI_ZB_FAIL) {
#ifdef ENABLE_DEBUG_PRINTS    
              RSI_DPRINT(RSI_PL1,"Unable to Set Power Mode \n");
#endif              
              return RSI_ZB_FAIL;
            }

              if(device_type == ZigBeeEndDevice)  {
              rsi_zigb_set_simple_descriptor(ONOFF_SWITCH_END_POINT, 
                   app_info->DeviceSimpleDesc);
              }
              else  {
                 /*For Router the descriptor set is ON/Off Light descriptor 
                app_info->DeviceSimpleDesc = &On_Off_Light_Simple_Desc;
                rsi_zigb_set_simple_descriptor(ONOFF_LIGHT_END_POINT, 
                  app_info->DeviceSimpleDesc);
                  */
                rsi_zigb_stack_is_up();    
              }
           
              app_cb_ptr->fsm_state = FSM_INIT_ENDDEV_ROUTER;
          }
        }
        break;

        case FSM_INIT_ENDDEV_ROUTER:
        {
#if 0          
          if (cmd_type == ZIGBEESETSIMPLEDESCRIPTOR || cmd_type == ZIGBEESTACKISUP)
          {
            rsi_status = resp->uCmdRspPayLoad.SetSimpleDescResp.status; 
            if( rsi_status == RSI_ZB_FAIL) {
#ifdef ENABLE_DEBUG_PRINTS    
              RSI_DPRINT(RSI_PL1,"Unable to Set Simple Desc \n");
#endif              
              return RSI_ZB_FAIL;
            }
            /* Setting the Default Trust center Link Key as defined by 
             * ZigBee HA1.2 specification */
            rsi_zigb_set_preconfigured_link_key(HA_linkKey);
            break;
          }
          else if (cmd_type == ZIGBEESETPRECONFIGUREDLINKKEY)
#endif            
          if (cmd_type == ZIGBEESETSIMPLEDESCRIPTOR || cmd_type == ZIGBEESTACKISUP)
          {
            rsi_status = resp->uCmdRspPayLoad.SetSimpleDescResp.status; 
            if( rsi_status == RSI_ZB_FAIL) {
              RSI_DPRINT(RSI_PL1,"Unable to Set Simple descriptor key \n");
              return RSI_ZB_FAIL;
            }
            if (!app_info->status_var.scanReqSent)
            {
              app_cb_ptr->fsm_state = FSM_INIT_SCAN;
              rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c,
                  zb_channel, g_SCAN_DURATION_c);
              rsi_zigb_delay(1);
              rsi_zigb_app_info.status_var.scanReqSent = 1;
            }
            break;
          }
          goto UNKNOWN_STATE;
        }
        break;

        case FSM_INIT_COORDINATOR:
        {
            rsi_zigb_handle_form_state(intf_type, cmd_type);

        }
        break;

        case FSM_ZB_FORMED:
        {
          if (cmd_type == ZIGBEEPERMITJOIN)
          {
            rsi_status = resp->uCmdRspPayLoad.statusResp.status; 
            if( !(rsi_status == RSI_ZB_SUCCESS)) {
#ifdef ENABLE_DEBUG_PRINTS    
              RSI_DPRINT(RSI_PL1,"Permit Join failed \n");
#endif              
              return RSI_ZB_FAIL;
            }
                rsi_zigb_set_simple_descriptor(ONOFF_LIGHT_END_POINT, 
                  app_info->DeviceSimpleDesc);
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"Permit Join Success \n");
#endif              
          }
#ifdef ZB_API_TEST
          rsi_zigb_cord_api_test();
#ifdef ENABLE_DEBUG_PRINTS    
          RSI_DPRINT(RSI_PL1,"TestCasesExecuted: %d \nTestCasesFailed: %d\nTestCasesSucceded %d\n ",TestCasesExecuted, TestCasesFailed,TestCasesSucceded );
          RSI_DPRINT(RSI_PL1,"\n ****** End of API_TEST ******* \n");
#endif          
          return 0;
#else
          rsi_zigb_app_cb_handler(cmd_type, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
#endif          
        }
        break;

        case FSM_INIT_SCAN:
        {
          rsi_status = rsi_zigb_handle_scan_state(intf_type, cmd_type);
          if( rsi_status != RSI_ZB_SUCCESS) {
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"Error in handling scan request \n");
#endif              
            return RSI_ZB_FAIL;
          }
        }
        break;

        case FSM_SCAN_DONE:
        {
          rsi_status = rsi_zigb_handle_scan_state(intf_type, cmd_type);
          if( rsi_status != RSI_ZB_SUCCESS) {
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"Error in handling scan request \n");
#endif              
            return RSI_ZB_FAIL;
          }
        }
        break;

        case FSM_JOIN_NETWORK:
        {
          scan_attempts = 0;
          rsi_status = rsi_zigb_handle_join_state(intf_type,
              cmd_type);
          if( rsi_status != RSI_ZB_SUCCESS) 
          {
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"Join state failed \n");
#endif              
            return RSI_ZB_FAIL;
          }
          if (app_cb_ptr->fsm_state == FSM_ZB_CONNECTED)
          {
#ifdef ZB_API_TEST
            app_cb_ptr->fsm_state = FSM_API_TEST;

            if(device_type == ZigBeeEndDevice) 
              rsi_zigb_network_state();
            else
              rsi_zigb_permit_join(0xFF);
#else
            if(device_type == ZigBeeRouter) {
              rsi_zigb_permit_join(0xFF);
              app_cb_ptr->fsm_state = FSM_ZB_HANDLE_ROUTER;
            }
            else
            {
#ifdef ENABLE_DEBUG_PRINTS    
              RSI_DPRINT(RSI_PL1,"sending Match Desc Req\n");
#endif         
              rsi_zigb_send_match_descriptors_request(0x0, 
                  app_info->DeviceSimpleDesc->app_profile_id, 
                  (uint8_t*)app_info->DeviceSimpleDesc->p_incluster_list,
                  app_info->DeviceSimpleDesc->incluster_count, 
                  (uint8_t*) app_info->DeviceSimpleDesc->p_outcluster_list,
                  app_info->DeviceSimpleDesc->outcluster_count, 1, 0xFFFF);
              rsi_status = rsi_zigb_handle_data();
              if(rsi_status != RSI_ZB_SUCCESS) 
              {
#ifdef ENABLE_DEBUG_PRINTS    
                RSI_DPRINT(RSI_PL1,"Error in handling scan request \n");
#endif          
                return RSI_ZB_FAIL;
              }
            }
#endif
          }
        }
        break;

        case FSM_ZB_CONNECTED:
        {
#ifdef ENABLE_DEBUG_PRINTS    
          RSI_DPRINT(RSI_PL1,"No pkt should be received by this time \n");
#endif          
          rsi_status = rsi_zigb_handle_data();
          if(rsi_status != RSI_ZB_SUCCESS) 
          {
            RSI_DPRINT(RSI_PL1,"Error in handling scan request \n");
            return RSI_ZB_FAIL;
          }
        }
        break;

        case FSM_ZB_REJOIN_SEND:
        {
          if(cmd_type == ZIGBEEREJOINNETWORK) {
            if(resp->uCmdRspPayLoad.statusResp.status != RSI_ZB_SUCCESS)  {
#ifdef ENABLE_DEBUG_PRINTS    
              RSI_DPRINT(RSI_PL1,"Rejoin response Fail \n");
#endif              
              return RSI_ZB_FAIL;
            }
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"Rejoin:Success \n");
#endif 
                      
          rsi_status = rsi_zigb_handle_data();
          if(rsi_status != RSI_ZB_SUCCESS) 
          {
            RSI_DPRINT(RSI_PL1,"Error in handling scan request \n");
            return RSI_ZB_FAIL;
          }

          }

        }
        break;
  
        case FSM_ZB_REJOINED:
        {
          rsi_status = rsi_zigb_handle_data();
          if(rsi_status != RSI_ZB_SUCCESS) 
          {
            RSI_DPRINT(RSI_PL1,"Error in handling scan request \n");
            return RSI_ZB_FAIL;
          }
          
        }
        break;


        case FSM_ZB_HANDLE_ROUTER:
        {
          if(cmd_type == ZIGBEEPERMITJOIN) {
            if(resp->uCmdRspPayLoad.statusResp.status != RSI_ZB_SUCCESS)  {
#ifdef ENABLE_DEBUG_PRINTS    
              RSI_DPRINT(RSI_PL1,"permit Join response Fail \n");
#endif              
              return RSI_ZB_FAIL;
            }
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"Permit Join response:Success \n");
#endif              

          }
          else  {
            rsi_zigb_app_cb_handler(cmd_type, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
        }
        break;
#ifdef ZB_API_TEST        
        case FSM_API_TEST:
        {
          if (cmd_type == ZIGBEENETWORKSTATE) {
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1, "rsi_zigb_network_state:");
#endif          
            if(resp->uCmdRspPayLoad.statusResp.status == 2)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            
            /*Start API test cases for End Device  mode*/
            rsi_zigb_api_test();
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"TestCasesExecuted: %d \nTestCasesFailed: %d\nTestCasesSucceded %d\n ",TestCasesExecuted, TestCasesFailed,TestCasesSucceded );
#endif          
            return RSI_ZB_SUCCESS;
          }
          else if (cmd_type == ZIGBEEPERMITJOIN) {
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1, "rsi_zigb_permit_join:");
#endif          
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }

            /*Start API test cases for Router mode*/
            rsi_zigb_router_api_test();
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"TestCasesExecuted: %d \nTestCasesFailed: %d\nTestCasesSucceded %d\n ",TestCasesExecuted, TestCasesFailed,TestCasesSucceded );
#endif          
            return RSI_ZB_SUCCESS;
          }
          else
          {
#ifdef ENABLE_DEBUG_PRINTS    
            RSI_DPRINT(RSI_PL1,"Unknown cmd pkt: %d in unknown state: %x\n ",cmd_type, app_cb_ptr->fsm_state);
#endif          
          }
        }
        break;
#endif

        default:
        {
#ifdef ENABLE_DEBUG_PRINTS    
          RSI_DPRINT(RSI_PL1,"Unknown state for the received resp. cmd_id: %x : state: %d  \n",cmd_type,app_cb_ptr->fsm_state);
#endif          
          goto UNKNOWN_STATE;
        }
        break;
      }
    }
    else
    {
      if (app_cb_ptr->fsm_state == FSM_SCAN_DONE)
      {
        if (scan_attempts >= MAX_SCAN_ATTEMPTS) 
        {
#ifdef ZB_DEBUG    
          RSI_DPRINT(RSI_PL1,"\n ERROR: Required network NOT FOUND!!!!!\n"); //App developer can make a decision what to do if required 
#endif  
          return RSI_ZB_FAIL;
        }
        if (app_cb_ptr->num_nwk_found) 
        {
          app_cb_ptr->fsm_state = FSM_JOIN_NETWORK;
          /* Check for a specific network and then Join.
           * Return/ Rescan if nwk not found */
          rsi_zigb_join_network(device_type, nwk_details->channel, 0x0f, nwk_details->extendedPanId);
#ifdef ENABLE_DEBUG_PRINTS    
          RSI_DPRINT(RSI_PL1,"Sending JOIN req \n");
#endif  
          rsi_zigb_delay(1);
          continue;
        }
        if (!app_info->status_var.scanReqSent)
        {
          scan_attempts++;
          app_cb_ptr->fsm_state = FSM_INIT_SCAN;
          rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c,
              zb_channel, g_SCAN_DURATION_c);
          rsi_zigb_app_info.status_var.scanReqSent = 1;
        }
        rsi_zigb_delay(1);
        continue;
        return RSI_ZB_FAIL;
      }
      if (app_cb_ptr->fsm_state == FSM_INIT_SCAN)
      {
        if (scan_attempts >= MAX_SCAN_ATTEMPTS) 
        {
#ifdef ENABLE_DEBUG_PRINTS    
          RSI_DPRINT(RSI_PL1,"\n ERROR: Required network NOT FOUND!!!!!\n"); //App developer can make a decision what to do if required 
#endif  
          return RSI_ZB_FAIL;
        }
        if (!app_info->status_var.scanReqSent)
        {
          scan_attempts++;
          app_cb_ptr->fsm_state = FSM_INIT_SCAN;
          rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c,
              zb_channel, g_SCAN_DURATION_c);
          rsi_zigb_app_info.status_var.scanReqSent = 1;
        }
        rsi_zigb_delay(1);
        continue;
      }
#if 0
      else if (state == FSM_JOIN_NETWORK)
      {
        if (scan_attempts >= MAX_SCAN_ATTEMPTS) 
        {
          RSI_DPRINT(RSI_PL1,"\n ERROR: Required network NOT FOUND!!!!!\n"); //App developer can make a decision what to do if required 
          return RSI_ZB_FAIL;
        }
        if (app_cb_ptr->num_nwk_found) 
        {
          state = FSM_JOIN_NETWORK;
          /* Check for a specific network and then Join.
           * Return/ Rescan if nwk not found */
          rsi_zigb_join_network(device_type, nwk_details->channel, 0x0f, nwk_details->extendedPanId);
          break;
        }
        scan_attempts++;
        state = FSM_INIT_SCAN;
        rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c,
                               g_CHANNEL_MASK_c, g_SCAN_DURATION_c);
        break;
        return RSI_ZB_FAIL;
      }
#endif
    }
  }   
UNKNOWN_STATE: 
#ifdef ENABLE_DEBUG_PRINTS    
  RSI_DPRINT(RSI_PL1,"Unknown state for the received resp. cmd_id: %x : state: %d  \n",cmd_type,app_cb_ptr->fsm_state);
#endif  
  return RSI_ZB_FAIL;
}

/*===========================================================================
 *
 * @fn          void rsi_zb_app_init(void)
 * @brief       Initialize ZigBee structues and variables
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to Initialize ZigBee specific variables and structures
 * to default values
 *
 * ===========================================================================*/
  
void rsi_zb_app_init(uint32_t zb_channel)
{
  rsi_zigb_app_cb_t *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;
#ifdef ZB_API_TEST  
  rsi_zigb_apitest_t  *api_test_info = &rsi_zigb_apitest;
#endif  

  app_cb_ptr->pkt_pending = RSI_FALSE;
  app_cb_ptr->fsm_state = FSM_CARD_NOT_READY;
  app_info->status_var.scanReqSent = 0;
 
  /*Initializing status variables */
  app_info->status_var.matchDescRspStatus = 0x0f;
  app_info->status_var.powerDescRspStatus = 0x0f;
  app_info->status_var.nodeDescRspStatus = 0x0f;
  app_info->status_var.bindRspStatus = 0x0f;
  app_info->status_var.unbindRspStatus = 0x0f;
  app_info->status_var.actepRspStatus = 0x0f;
  app_info->status_var.ieeeaddrRspStatus = 0x0f;
  app_info->status_var.simpledescRspStatus = 0x0f;
  app_info->status_var.networkaddrRspStatus = 0x0f;

  app_info->DeviceSimpleDesc = &SimpleDesc;
  
  app_info->channel_mask = zb_channel;
  app_info->scan_duration = g_SCAN_DURATION_c;
  app_cb_ptr->channel = rsi_zigb_channel_ext(zb_channel);
  app_cb_ptr->power = 10;

#ifdef ZB_API_TEST
  api_test_info->Source_EP = CUSTOM_EP;
  api_test_info->Dest_EP = 10; 
  api_test_info->DestAddrMode = 3;
  api_test_info->ClusterId =1; 
  api_test_info->APSAckRequired = TRUE;
  TestCasesExecuted = 0;
  TestCasesFailed = 0;
  TestCasesSucceded = 0;
#endif  
}

/*===========================================================================
 *
 * @fn          RSI_ZB_STATUS rsi_zigb_handle_scan_state(uint8_t intf, uint8_t cmd)
 * @brief       Handles scan state
 * @param[in]   Command type
 * @param[in]   Interface type
 * @param[out]  none
 * @return      Status
 * @section description
 * This API is used to handle scan related pkts in scan state
 *
 * ===========================================================================*/
  
RSI_ZB_STATUS rsi_zigb_handle_scan_state(uint8_t intf, uint8_t cmd)
{
  rsi_zigb_app_cb_t *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;
  rsi_zigb_uCmdRsp *resp;
  RSI_ZB_STATUS rsi_status;

  resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);
  if (!resp) {
#ifdef ENABLE_DEBUG_PRINTS    
    RSI_DPRINT(RSI_PL1,"resp is NULL Error in processing pkt2 \n");
#endif      
    return RSI_ZB_FAIL;
  }

  if (intf == INTERFACE_CALLBACK)
  {
    if (cmd == APPSCANCOMPLETEHANDLER)
    {
      rsi_status = resp->uCmdRspPayLoad.scandoneResp.status;
      if( rsi_status != RSI_ZB_SUCCESS) {
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"No beacon found: %d \n",rsi_status);
#endif  
        app_cb_ptr->num_nwk_found = 0;
      }
      else
      {
        app_cb_ptr->num_nwk_found = 1;
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Network found handler\n");
#endif        
      }
      app_info->status_var.scanReqSent = 0;
      app_cb_ptr->fsm_state = FSM_SCAN_DONE;
      rsi_zigb_app_cb_handler(cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
      rsi_zigb_delay(1);
      return RSI_ZB_SUCCESS;
    }
    rsi_zigb_app_cb_handler(cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
  }
  else if (intf == MANAGEMENT_INTERFACE)
  {
    if (cmd == ZIGBEEINITIATESCAN)
    {
      rsi_status = resp->uCmdRspPayLoad.statusResp.status; 
      if( rsi_status == RSI_ZB_FAIL) {
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Unable to initiate Scan \n");
#endif      
        return RSI_ZB_FAIL;
      }
    }
  }
  else
  {
#ifdef ENABLE_DEBUG_PRINTS        
    RSI_DPRINT(RSI_PL1,"Unknown Interface type received in scan state \n");
#endif    
    return RSI_ZB_FAIL;
  }
  return RSI_ZB_SUCCESS;
}

/*===========================================================================
 *
 * @fn          RSI_ZB_STATUS rsi_zigb_handle_join_state(uint8_t intf, uint8_t cmd)
 * @brief       Handles join state
 * @param[in]   Command type
 * @param[in]   Interface type
 * @param[out]  none
 * @return      Status
 * @section description
 * This API is used to handle Join related pkts in join state
 *
 * ===========================================================================*/

RSI_ZB_STATUS rsi_zigb_handle_join_state(uint8_t intf, uint8_t cmd)
{
  rsi_zigb_app_cb_t *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_uCmdRsp *resp;
  RSI_ZB_STATUS rsi_status;

  resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);
  if (!resp) {
#ifdef ENABLE_DEBUG_PRINTS    
    RSI_DPRINT(RSI_PL1,"resp is NULL Error in processing pkt3 \n");
#endif      
    return RSI_ZB_FAIL;
  }

  if (intf == INTERFACE_CALLBACK)
  {
    if (cmd == APPZIGBEESTACKSTATUSHANDLER) 
    { 
      if (resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
      {
        app_cb_ptr->fsm_state = FSM_ZB_CONNECTED;
#ifdef ZB_DEBUG        
        RSI_DPRINT(RSI_PL1,"Joined Successfully \n");
#endif   
      }
      else
      {
#ifdef ZB_DEBUG        
        //app_cb_ptr->fsm_state = FSM_INIT_SCAN;
        RSI_DPRINT(RSI_PL1,"Joined Failed \n");
        RSI_DPRINT(RSI_PL1,"Network Failed, status: %x\n",resp->uCmdRspPayLoad.statusResp.status);
#endif        
        return RSI_ZB_FAIL;
      }
    }
    rsi_zigb_app_cb_handler(cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
  }
  else if (intf == MANAGEMENT_INTERFACE)
  {
    if (cmd == ZIGBEEJOINNETWORK)
    {
      rsi_status = resp->uCmdRspPayLoad.statusResp.status; 
      if( rsi_status == RSI_ZB_FAIL) 
      {
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Unable to initiate Scan \n");
#endif        
        return RSI_ZB_FAIL;
      }
#ifdef ENABLE_DEBUG_PRINTS        
      RSI_DPRINT(RSI_PL1,"Waiting for Association Confirm\n");
#endif      
      //rsi_zigb_end_device_poll_for_data();
    }
  }
  else
  {
#ifdef ENABLE_DEBUG_PRINTS        
    RSI_DPRINT(RSI_PL1,"Unknown Interface type received in join state \n");
#endif    
    return RSI_ZB_FAIL;
  }
  return RSI_ZB_SUCCESS;
}

/*===========================================================================
 *
 * @fn          RSI_ZB_STATUS rsi_zigb_handle_data()
 * @brief       Handles data in connected state
 * @param[in]   Match desc request sent status
 * @param[out]  none
 * @return      Status
 * @section description
 * This API is used to handle all type of data pkts after connection
 *
 * ===========================================================================*/
RSI_ZB_STATUS rsi_zigb_handle_data()
{
  rsi_zigb_app_cb_t *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;
  rsi_zigb_uCmdRsp *resp;
  RSI_ZB_STATUS rsi_status;
  uint8_t matchDesctimeout = 0;
  uint8_t matchDescReqSent = 0;
  uint8_t cmd, intf;
  uint16_t* lightAddress;
  lightDeviceInfo_t lightDevInfo;

  rsi_zigb_memset((uint8_t *)&lightDevInfo, 0, sizeof(lightDeviceInfo_t));
  while (1)
  {
    if(app_cb_ptr->pkt_pending == RSI_TRUE) 
    {
#if 0
      if ((app_cb_ptr->fsm_state != FSM_ZB_CONNECTED) || (app_cb_ptr->fsm_state != FSM_ZB_REJOIN_SEND) ||
          (app_cb_ptr->fsm_state != FSM_ZB_REJOINED)) 
      {
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Network Down, Not in connected state \n fsm_state:%d",app_cb_ptr->fsm_state);
#endif        
        return RSI_ZB_FAIL;
      }
#endif
      rsi_status = rsi_frame_read(app_cb_ptr->read_packet_buffer);
      if (rsi_status != RSI_ZB_SUCCESS)
      {
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Error in dequeuing pkt \n");
#endif        
        return RSI_ZB_FAIL;
      }
      resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);
      if (!resp) {
#ifdef ENABLE_DEBUG_PRINTS    
        RSI_DPRINT(RSI_PL1,"resp is NULL Error in processing pkt4 \n");
#endif      
        return RSI_ZB_FAIL;
      }

      cmd = resp->cmd_id;
      intf = resp->intf_id;
      if (intf == INTERFACE_CALLBACK)
      {
        if (cmd == APPZIGBEESTACKSTATUSHANDLER)  
        {
          
          if (resp->uCmdRspPayLoad.statusResp.status == ZigBeeChangedNodeID)
          {
            app_cb_ptr->fsm_state = FSM_ZB_REJOINED;
            rsi_zigb_send_match_descriptors_request(0x0, 
                app_info->DeviceSimpleDesc->app_profile_id, 
                (uint8_t*)app_info->DeviceSimpleDesc->p_incluster_list,
                app_info->DeviceSimpleDesc->incluster_count, 
                (uint8_t*) app_info->DeviceSimpleDesc->p_outcluster_list,
                app_info->DeviceSimpleDesc->outcluster_count, 1, 0xFFFF);


          }
          else if(resp->uCmdRspPayLoad.statusResp.status != RSI_ZB_SUCCESS)
          {
            if(app_info->network_down < 10) 
            {
            RSI_DPRINT(RSI_PL1,"**********Network Down, so Initiating Rejoin %d time\n",(app_info->network_down+ 1));
            app_cb_ptr->fsm_state = FSM_ZB_REJOIN_SEND;
            sleep(1);
            rsi_zigb_rejoin_network(TRUE); 
            app_info->network_down++;
            return RSI_ZB_SUCCESS;
            }
            else
            {
            app_cb_ptr->fsm_state = FSM_INIT_SCAN;
            app_info->status_var.matchDescRspStatus = 0xFF;
#ifdef ZB_DEBUG        
            RSI_DPRINT(RSI_PL1,"Joined Failed \n");
            RSI_DPRINT(RSI_PL1,"Network Down, so Initiating scan \n");
#endif        
            return RSI_ZB_FAIL;
            }
          }
          else
          {
            RSI_DPRINT(RSI_PL1,"Stack status Success \n");

            return RSI_ZB_SUCCESS;
          }
        }
        else
        {
          rsi_zigb_app_cb_handler(cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
      }
      else if (intf == MANAGEMENT_INTERFACE)
      {
        if (cmd == ZDPSENDMATCHDESCRIPTORSREQUEST)
        {
          rsi_status = resp->uCmdRspPayLoad.statusResp.status; 
          if( rsi_status == RSI_ZB_FAIL) 
          {
#ifdef ENABLE_DEBUG_PRINTS        
            RSI_DPRINT(RSI_PL1,"Unable to initiate Scan \n");
#endif        
            return RSI_ZB_FAIL;
          }
#ifdef ENABLE_DEBUG_PRINTS        
          RSI_DPRINT(RSI_PL1,"Received Match desc response \n");
#endif        
          matchDescReqSent = 1;
          matchDesctimeout = 0;
          app_info->status_var.matchDescRspStatus = 0xFF;
        }
        else if (cmd == ZIGBEEENDDEVICEPOLLFORDATA)
        {
          rsi_status = resp->uCmdRspPayLoad.statusResp.status; 
          if( rsi_status == RSI_ZB_FAIL) 
          {
#ifdef ENABLE_DEBUG_PRINTS        
            RSI_DPRINT(RSI_PL1,"Unable to initiate Scan \n");
#endif        
            return RSI_ZB_FAIL;
          }
#ifdef ENABLE_DEBUG_PRINTS        
          RSI_DPRINT(RSI_PL1,"Received Poll response \n");
#endif        
          app_info->wait_for_rsp = 0; 
        }
        
      }
      else
      {
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Unknown Interface type received in join state \n");
#endif        
        return RSI_ZB_FAIL;
      }
    }
    else if(matchDescReqSent)
    {
      if (app_info->status_var.matchDescRspStatus)
      {
        rsi_zigb_delay(1);
        if (matchDesctimeout >= 10)
        {
          /*FIX: Add user defined timeout in secs based on processor speed
           * or you can use sleep() api*/
#ifdef ENABLE_DEBUG_PRINTS        
          RSI_DPRINT(RSI_PL1,"matchDescReqSent but didn't get response within time\n");
#endif        
          return RSI_ZB_FAIL;
        }
        else
        {
#ifdef ENABLE_DEBUG_PRINTS        
          RSI_DPRINT(RSI_PL1,"Sending data req for match desc req\n");
#endif        

          if(!app_info->wait_for_rsp){ 
            rsi_zigb_end_device_poll_for_data();
            app_info->wait_for_rsp = 1;
          }
        }
        matchDesctimeout++;
      }
      else
      {
        matchDescReqSent = 0;
        /* received Match desc response with success status */
        lightAddress =  (uint16_t *)&app_info->zb_resp_info.matchDescResp[2];
        lightDevInfo.shortaddress = *lightAddress;    
        lightDevInfo.endpoint = app_info->zb_resp_info.matchDescResp[5];

        /* Send the Toggle command to the light continuously */
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"\n Now Sending Toggle command to Corrdinator Home Automation supported Light\n"); 
#endif        
        rsi_zigb_app_send_data( g_Client_To_Server_c, g_Cluster_Specific_Cmd_c, lightDevInfo.endpoint, lightDevInfo.shortaddress , g_TOGGLE_c, 0x0006, 0x00, 0x0);
        app_info->status_var.dataConfWait = 1;
      }
    }
    else if (app_info->status_var.dataConfWait)
    {
      rsi_zigb_delay(1);
    }
    else if (app_cb_ptr->send_data)
    {
      rsi_zigb_delay(g_POLL_RATE_c/1000);
      rsi_zigb_app_send_data( g_Client_To_Server_c, g_Cluster_Specific_Cmd_c, lightDevInfo.endpoint, lightDevInfo.shortaddress , g_TOGGLE_c, 0x0006, 0x00, 0x0);
      app_info->status_var.dataConfWait = 1;
      app_cb_ptr->send_data = 0;
    }
  }
  return RSI_ZB_FAIL;
}

/*===========================================================================
 *
 * @fn          uint8_t rsi_zigb_zcl_create_command (uint8_t direction, uint8_t *p_asdu,
                                      void* p_ZCL_Cmd_Data, uint8_t ZCL_Cmd_Data_Length,
                                      uint8_t trans_seq_num)
 * @brief       Prepares the ZigBee Cluster command 
 * @param[in]   Direction
 * @param[in]   p_asdu - buffer pointer of data
 * @param[in]   p_ZCL_Cmd_Data - Cluster data
 * @param[in]   length of ZCL data
 * @param[in]   Seq num
 * @param[out]  none
 * @return      Final data length
 * @section description
 * This API is used to prepare the ZigBee Cluster command pkt
 *
 * ===========================================================================*/
  
uint8_t rsi_zigb_zcl_create_command (uint8_t direction, uint8_t *p_asdu,
                void* p_ZCL_Cmd_Data, uint8_t ZCL_Cmd_Data_Length,
                uint8_t trans_seq_num)
{
  App_ZCL_Request_t *p_received_data = ( App_ZCL_Request_t *)p_ZCL_Cmd_Data;
  uint8_t data_length = 0;
  BOOL manufacture_specific = p_received_data->manufacture_specific;
  BOOL disable_default_response = p_received_data->disable_default_response;
  uint8_t *p_ZCL_Header_Payload = p_asdu;

  if ( !( p_received_data->command_type & 0x01 )) {
          ((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control = g_Generic_Cmd_c ;
  } else {
          ((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control = g_Cluster_Specific_Cmd_c;
  }
  ((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control |= direction;

  data_length++;

  if ( disable_default_response ) {
          ((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control |= g_Disable_Default_Response_c;
  }
  if ( manufacture_specific ) {
          ((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control |= g_Manufacture_Specific_Bit_c ;
          rsi_zigb_mcpy((uint8_t*)p_received_data->a_manufacturer_code, (uint8_t*)((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->a_manufacture_code ,2 );
          data_length += sizeof(uint16_t);
          ZCL_Cmd_Data_Length -= 0x03;
  } else {
          ZCL_Cmd_Data_Length -= 0x05;
  }
  *( p_ZCL_Header_Payload + data_length ) = trans_seq_num;

  data_length++;

  *( p_ZCL_Header_Payload +  data_length ) = p_received_data->ZCL_command_received.command_identifier;

  ZCL_Cmd_Data_Length--;
  data_length++;

  rsi_zigb_mcpy((uint8_t*)&( p_received_data->ZCL_command_received.Foundation_Commands ) ,
                  p_ZCL_Header_Payload + data_length, ZCL_Cmd_Data_Length);

  data_length += ZCL_Cmd_Data_Length;

  return data_length;
}

/*===========================================================================
 *
 * @fn          uint8_t rsi_zigb_app_send_data( uint8_t direction, uint8_t commandType, 
 *                                              uint8_t destEP, uint16_t dest_short_address,
                                                uint8_t commandId, uint16_t cluster, 
                                                uint8_t dataLength,uint8_t* payloadPointer )
 * @brief       Prepares ZigBee data pkt
 * @param[in]   Direction
 * @param[in]   Command type
 * @param[in]   Destination End Point
 * @param[in]   Destination Short address
 * @param[in]   ZCL Command ID received
 * @param[in]   Cluster type
 * @param[in]   Data length
 * @param[in]   Payload pointer
 * @param[out]  none
 * @return      Status
 * @section description
 * This API is used to prepare the ZigBee Data pkt with cluster information
 *
 * ===========================================================================*/

uint8_t rsi_zigb_app_send_data( uint8_t direction, uint8_t commandType, uint8_t destEP, uint16_t dest_short_address,
                uint8_t commandId, uint16_t cluster, uint8_t dataLength,uint8_t* payloadPointer )
{
  uint8_t status;
  Address DestAddress;
  ZigBeeAPSDEDataRequest_t APSDERequest;
  App_ZCL_Request_t *pZcl_hdr;
  uint8_t *pAsdu;

  /*+1 is added for Command id*/
  uint8_t ZCLHeaderLength = ((sizeof(App_ZCL_Request_t) - sizeof(ZCL_Command_t)) + 1);

  DestAddress.short_address = 0x00;
  //DestAddress.short_address = dest_short_address;

  APSDERequest.ProfileId = HA_PROFILE_ID;
  APSDERequest.DestEndpoint = destEP;
  APSDERequest.ClusterId = cluster;
  APSDERequest.AsduLength = dataLength;
  APSDERequest.SrcEndpoint = ONOFF_SWITCH_END_POINT;
  // APSDERequest.TxOptions = g_APS_Tx_Opt_Use_NWK_Key_c | g_APS_Tx_Opt_Ack_Req_c;
  APSDERequest.TxOptions = g_APS_Tx_Opt_Use_NWK_Key_c ;
  APSDERequest.Radius = DEFAULT_RADIUS;

  pZcl_hdr = (App_ZCL_Request_t*)APSDERequest.aPayload;
  pZcl_hdr->command_type = commandType;
  pZcl_hdr->disable_default_response = g_Disable_Default_Response_c;
  pZcl_hdr->manufacture_specific = 0;
  pZcl_hdr->ZCL_command_received.command_identifier = commandId;
  pAsdu = APSDERequest.aPayload + ZCLHeaderLength;

  rsi_zigb_mcpy(payloadPointer,pAsdu, dataLength );

  APSDERequest.AsduLength =  rsi_zigb_zcl_create_command ( direction,
                  APSDERequest.aPayload,
                  (App_ZCL_Request_t*)&APSDERequest.aPayload,
                  dataLength + ZCLHeaderLength ,
                  0);

  status = rsi_zigb_send_unicast_data(ZigBee_Outgoing_Direct,
                  DestAddress  , &APSDERequest);

  return status;
}


/*===========================================================================
 *
 * @fn          RSI_ZB_STATUS rsi_zigb_handle_form_state(uint8_t intf, uint8_t cmd)
 * @brief       Handles form state
 * @param[in]   Command type
 * @param[in]   Interface type
 * @param[out]  none
 * @return      Status
 * @section description
 * This API is used to handle form network state
 * Form state is obtained only after coordinator forms a network 
 * successfully
 *
 * ===========================================================================*/

RSI_ZB_STATUS rsi_zigb_handle_form_state(uint8_t intf, uint8_t cmd)
{
  rsi_zigb_app_cb_t *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_uCmdRsp *resp;
  RSI_ZB_STATUS rsi_status;

  resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);
  if (!resp) {
#ifdef ENABLE_DEBUG_PRINTS    
    RSI_DPRINT(RSI_PL1,"resp is NULL Error in processing pkt \n");
#endif      
    return RSI_ZB_FAIL;
  }

  if (intf == INTERFACE_CALLBACK)
  {
    if (cmd == APPZIGBEESTACKSTATUSHANDLER) 
    { 
      if (resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
      {
        app_cb_ptr->fsm_state = FSM_ZB_FORMED;
        rsi_zigb_permit_join(0xFF);
      
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Joined Successfully \n");
#endif        
      }
      else
      {
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Form Failed \n");
        RSI_DPRINT(RSI_PL1,"Network Failed, status: %x\n",resp->uCmdRspPayLoad.statusResp.status);
#endif        
        return RSI_ZB_FAIL;
      }
    }
    rsi_zigb_app_cb_handler(cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
  }
  else if (intf == MANAGEMENT_INTERFACE)
  {
    if (cmd == ZIGBEEFORMNETWORK)
    {
      rsi_status = resp->uCmdRspPayLoad.statusResp.status; 
      if( rsi_status == RSI_ZB_FAIL) 
      {
#ifdef ENABLE_DEBUG_PRINTS        
        RSI_DPRINT(RSI_PL1,"Unable to initiate Scan \n");
#endif        
        return RSI_ZB_FAIL;
      }
    }
  }
  else
  {
#ifdef ENABLE_DEBUG_PRINTS        
    RSI_DPRINT(RSI_PL1,"Unknown Interface type received in join state \n");
#endif    
    return RSI_ZB_FAIL;
  }
  return RSI_ZB_SUCCESS;
}
uint8_t rsi_zigb_channel_ext(uint32_t channel_Mask)
{
  uint8_t i;
  for(i=0 ; i<16 ; i++) {
    if((channel_Mask >> i) & (0x00000800))
      return (i+11);
  }
  return (11);
}
