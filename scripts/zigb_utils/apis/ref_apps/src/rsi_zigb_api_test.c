/**
 *  @file     rsi_zigb_api_test.c
 *  @version  1.0
 *  @date     2014-Sep-22
 *
 *  Copyright(C) Redpine Signals 2014
 *  All rights reserved by Redpine Signals.
 *
 *  @section License
 *  This program should be used on your own responsibility.
 *  Redpine Signals assumes no responsibility for any losses
 *  incurred by customers or third parties arising from the use of this file.
 *
 *  @brief API: ZigBee API test case function
 *
 *  @section Description
 *  This file describes how ZigBee APIS are initiated.
 *
 *  @section Improvements
 *
 */


/**
 * Includes
 * */
#ifdef ZB_API_TEST
#include "rsi_zigb_app.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_interfaces.h"
#include "rsi_zigb_callbacks.h"
#include "rsi_zigb_app_sm.h"
#include "rsi_zigb_types.h"
#include "rsi_zigb_onoff.h"
#include "rsi_zigb_global.h"
#ifdef LINUX_PLATFORM
#include "stdio.h"
#include "platform_specific.h"
#endif
#include "rsi_zigb_api_test.h"

rsi_zigb_apitest_var_t rsi_zigb_apitest_var;

extern rsi_zigb_apitest_t  rsi_zigb_apitest;
extern rsi_zigb_app_info_t rsi_zigb_app_info;
extern rsi_zigb_app_cb_t   rsi_zigb_app_cb;
/*===========================================================================
 *
 * @fn          void rsi_zigb_api_test(void)
 * @brief       Handle API test cases
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to test the sequence of flow of API test
 *
 * ===========================================================================*/

RSI_ZB_STATUS rsi_zigb_api_test()
{
  rsi_zigb_app_info_t     *app_info = &rsi_zigb_app_info;
  rsi_zigb_apitest_t      *api_test_info = &rsi_zigb_apitest;
  rsi_zigb_apitest_var_t  *api_test_var = &rsi_zigb_apitest_var;
  rsi_zigb_app_cb_t       *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_uCmdRsp        *resp;

  /* Initialize variables used in apitest */
  rsi_zigb_api_var_init();

  resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);

  rsi_zigb_set_simple_descriptor(ONOFF_SWITCH_END_POINT,app_info->DeviceSimpleDesc);

  while (1)
  {
    if(app_cb_ptr->pkt_pending == RSI_TRUE) 
    {
      if (app_cb_ptr->fsm_state != FSM_API_TEST) 
      {
        RSI_DPRINT(RSI_PL1,"Network Down, Not in connected state \n");
        return RSI_ZB_FAIL;
      }
      api_test_var->rsi_status = rsi_frame_read(app_cb_ptr->read_packet_buffer);
      if (api_test_var->rsi_status != RSI_ZB_SUCCESS)
      {
        RSI_DPRINT(RSI_PL1,"Error in dequeuing pkt \n");
        return RSI_ZB_FAIL;
      }

      api_test_var->cmd = resp->cmd_id;
      switch(api_test_var->cmd)
      {

        case  ZIGBEESETSIMPLEDESCRIPTOR:
        {
          /*All the below apis are passed by this time */
          RSI_DPRINT(RSI_PL1, "rsi_zigb_init_stack:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_reset_stack:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_device_type:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_app_scan_complete_handler:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_app_network_found_handler:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_app_stack_status_handler:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_join_network:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_simple_descriptor:");
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
          rsi_zigb_stack_is_up();
        }
        break;

        case ZIGBEESTACKISUP :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_stack_is_up:");
          if(resp->uCmdRspPayLoad.statusResp.status == 0x1)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_self_ieee_address();
        }
        break;

        case ZIGBEEGETSELFIEEEADDRESS :
        {
          rsi_zigb_mcpy( resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee , api_test_info->SelfAddress.IEEE_address , 8);

          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_self_ieee_address:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_is_it_self_ieee_address(resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee );
        }
        break;

        case ZIGBEEISITSELFIEEEADDRESS :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_is_it_self_ieee_address:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_get_self_short_address();
        }
        break;

        case ZIGBEEGETSELFSHORTADDRESS :
        {
          api_test_var->short_addr_t = resp->uCmdRspPayLoad.DevShortAddr.short_addr;
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_self_short_address:");
          if(api_test_var->short_addr_t != 0x0000)
          { 
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_set_manufacturer_code_for_node_desc(api_test_var->ManufacturerCode);
        }
        break;

        case ZIGBEESETMANUFACTURERCODEFORNODEDESC :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_manufacturer_code_for_node_desc:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_set_power_descriptor(&api_test_var->nodePowerDesc_t);
        }
        break;

        case ZIGBEESETPOWERDESCRIPTOR :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_power_descriptor:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_set_maxm_incoming_txfr_size(0x1234);
        }
        break;

        case ZIGBEESETMAXMINCOMINGTXFRSIZE :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_maxm_incoming_txfr_size:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_set_maxm_out_going_txfr_size(0x1234);

        }
        break;

        case ZIGBEESETMAXMOUTGOINGTXFRSIZE :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_maxm_out_going_txfr_size:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_get_operating_channel();
        }
        break;

        case ZIGBEEGETOPERATINGCHANNEL :
        {
          api_test_var->channel = resp->uCmdRspPayLoad.DevChannel.channel;

          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_operating_channel:");
          if(api_test_var->channel != 0x00 ) 
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_short_panid();
        }
        break;

        case ZIGBEEGETSHORTPANID :
        {
          api_test_var->pan_Id = resp->uCmdRspPayLoad.DevPanId.pan_id; 

          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_short_panid:");
          if(api_test_var->pan_Id != 0x0000)
          { 
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_extended_panid();
        }
        break;

        case ZIGBEEGETEXTENDEDPANID :
        {
          rsi_zigb_mcpy(resp->uCmdRspPayLoad.Extnd_PanId.Ext_Pan_Id ,api_test_var->Ext_PanId , 8);
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_extended_panid:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS
          rsi_zigb_get_endpoint_id(0x1);
        }
        break;

        case ZIGBEEGETENDPOINTID :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_endpoint_id:");
          if(resp->uCmdRspPayLoad.statusResp.status == 0xF1)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_simple_descriptor(ONOFF_SWITCH_END_POINT);
        }
        break;
 
        case ZIGBEEGETSIMPLEDESCRIPTOR :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_simple_descriptor:");
          if(resp->uCmdRspPayLoad.statusResp.status == TRUE)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_endpoint_cluster(0x00,0x00,0x00);
        }
        break;

        case ZIGBEEGETENDPOINTCLUSTER :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_endpoint_cluster:");

          api_test_var->endpoint_cluster = resp->uCmdRspPayLoad.EpClusterResp.cluster ;
          if(api_test_var->endpoint_cluster == 0xffff)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
              rsi_zigb_stack_profile();
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
        }
        break;

        case ZIGBEESTACKPROFILE :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_stack_profile:");
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
          rsi_zigb_get_parent_ieee_address();
        }
        break;

        case ZIGBEEGETPARENTIEEEADDRESS :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_parent_ieee_address:");             
          rsi_zigb_mcpy( resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee ,api_test_info->ParentAddress.IEEE_address , 8);
          rsi_zigb_mcpy( api_test_info->ParentAddress.IEEE_address , api_test_var->parent_ieee,8);

          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

            rsi_zigb_get_parent_short_address();
        }
        break;

        case ZIGBEEGETPARENTSHORTADDRESS :
        {
          api_test_var->parent_short_addr  =  resp->uCmdRspPayLoad.DevShortAddr.short_addr;
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_parent_short_address:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_get_ieee_addr_for_specified_short_addr(api_test_var->parent_short_addr );
        }
        break;

        case ZIGBEEGETIEEEADDRFORSPECIFIEDSHORTADDR :
        {
          rsi_zigb_mcpy( resp->uCmdRspPayLoad.GetIEEEResp.Ieee_Add ,api_test_var->get_ieee , 8);
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_ieee_addr_for_specified_short_addr:");

          if((rsi_zigb_mcmp(api_test_var->get_ieee,api_test_var->parent_ieee,8)) == RSI_ZB_SUCCESS)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_short_addr_for_specified_ieee_addr( api_test_var->parent_ieee);
        }
        break;


        case ZIGBEEGETSHORTADDRFORSPECIFIEDIEEEADDR :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_short_addr_for_specified_ieee_addr:");
          
          if(resp->uCmdRspPayLoad.DevShortAddr.short_addr == api_test_var->parent_short_addr)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_read_neighbor_table_entry( 0x0);
        }
        break;

        case ZIGBEEREADNEIGHBORTABLEENTRY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_read_neighbor_table_entry:");
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
          rsi_zigb_tree_depth();
        }
        break;

        case ZIGBEEGETROUTETABLEENTRY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_route_table_entry:");
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
        }
        break;

        case ZIGBEETREEDEPTH :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_tree_depth:");
          if(resp->uCmdRspPayLoad.statusResp.status == 1)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_neighbor_table_entry_count();
        }
        break;

        case ZIGBEEGETNEIGHBORTABLEENTRYCOUNT :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_neighbor_table_entry_count:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_zdp_send_device_announcement();
        }
        break;

        case ZIGBEEENDDEVICEPOLLFORDATA :
        {
          if(!api_test_var->end_device_poll_count)
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_end_device_poll_for_data:");
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
              resp->uCmdRspPayLoad.statusResp.status = RSI_ZB_FAIL;
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_zdp_send_nwk_addr_request(api_test_var->parent_ieee,(BOOL)0,0 );
            app_info->wait_for_rsp =1;
          }
          api_test_var->end_device_poll_count++;

          rsi_zigb_delay(1);//Dont remove this sleep ,it is kept to synchronize the responses received from Parent
          
          if(!app_info->wait_for_rsp){

            if( app_info->status_var.networkaddrRspStatus == 0x0f  ){
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.networkaddrRspStatus == 0x00  ){
                 app_info->status_var.networkaddrRspStatus = 0x01; 
                 rsi_zigb_zdp_send_ieee_addr_request(api_test_var->parent_short_addr,api_test_var->request_type,api_test_var->start_index,api_test_var->aps_ack_required);
                 app_info->wait_for_rsp = 1;
           }
            else if( app_info->status_var.ieeeaddrRspStatus == 0x0f  ){
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.ieeeaddrRspStatus == 0x00  ){
              app_info->status_var.ieeeaddrRspStatus = 0x01;
                  rsi_zigb_send_match_descriptors_request(api_test_var->parent_short_addr, 
                  app_info->DeviceSimpleDesc->app_profile_id, 
                  (uint8_t*)app_info->DeviceSimpleDesc->p_incluster_list,
                  app_info->DeviceSimpleDesc->incluster_count, 
                  (uint8_t*) app_info->DeviceSimpleDesc->p_outcluster_list,
                  app_info->DeviceSimpleDesc->outcluster_count, 1, 0xFFFF);
              app_info->wait_for_rsp = 1;
            }
            else if( app_info->status_var.matchDescRspStatus == 0x0f  ){
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.matchDescRspStatus == 0x00  ){
              app_info->status_var.matchDescRspStatus = 0x01;
              rsi_zigb_active_endpoints_request( api_test_var->parent_short_addr, api_test_var->aps_ack_required );
              app_info->wait_for_rsp = 1;
            }
            else if( app_info->status_var.actepRspStatus == 0x0f  ) {
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.actepRspStatus == 0x00  ) {
              app_info->status_var.actepRspStatus = 1;
              rsi_zigb_zdp_send_power_descriptor_request(api_test_var->parent_short_addr,api_test_var->aps_ack_required);
              app_info->wait_for_rsp = 1;
            }
            else if( app_info->status_var.powerDescRspStatus == 0x0f  ) {
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.powerDescRspStatus == 0x00  ) {
              rsi_zigb_zdp_send_node_descriptor_request(api_test_var->parent_short_addr,api_test_var->aps_ack_required);
              app_info->status_var.powerDescRspStatus = 0x01;
              app_info->wait_for_rsp = 1;
            }
            else if( app_info->status_var.nodeDescRspStatus == 0x0f  ) {
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.nodeDescRspStatus == 0x00  ) {
              app_info->status_var.nodeDescRspStatus = 0x1;
              rsi_zigb_simple_descriptor_request(api_test_var->parent_short_addr,api_test_var->end_pont_id);
              app_info->wait_for_rsp = 1;
            }
            else if( app_info->status_var.simpledescRspStatus == 0x0f  ){
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.simpledescRspStatus == 0x00  ){
              app_info->status_var.simpledescRspStatus = 0x1;
              rsi_zigb_bind_request(api_test_var->parent_short_addr, api_test_info->SelfAddress.IEEE_address, api_test_info->Source_EP, api_test_info->ClusterId, api_test_info->DestAddrMode, api_test_info->ParentAddress,api_test_info->Dest_EP, api_test_info->APSAckRequired);
              app_info->wait_for_rsp = 1;
            }
            else if( app_info->status_var.bindRspStatus == 0x0f  ) {
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.bindRspStatus == 0x00  ) {
              app_info->status_var.bindRspStatus = 0x1;
              rsi_zigb_unbind_request(api_test_var->parent_short_addr, api_test_info->SelfAddress.IEEE_address, api_test_info->Source_EP, api_test_info->ClusterId, api_test_info->DestAddrMode, api_test_info->ParentAddress,api_test_info->Dest_EP, api_test_info->APSAckRequired);
              app_info->wait_for_rsp = 1;
            }
            else if( app_info->status_var.unbindRspStatus == 0x0f  ) {
              rsi_zigb_end_device_poll_for_data();
            }
            else if( app_info->status_var.unbindRspStatus == 0x00  ) {
              app_info->status_var.unbindRspStatus = 0x1;
              rsi_zigb_network_restore();
              app_info->wait_for_rsp = 1;
            }
            else
            {
              /* Do nothing */
            }
          }
          else
          {
            rsi_zigb_delay(1);
          }
        }
        break;

        case ZDPSENDNWKADDRREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
        }
        break;

        case ZDPSENDIEEEADDRREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
        }
        break;

        case ZDPSENDDEVICEANNOUNCEMENT :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_zdp_send_device_announcement:");
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
          rsi_zigb_send_unicast_data(ZigBee_Outgoing_Direct, api_test_var->DestAddress,&api_test_var->APSDEDataReq);
        }
        break;

        case ZDPSENDMATCHDESCRIPTORSREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
          /* Response is checked at Data indication Callback */
        }
        break;

        case ZIGBEEACTIVEENDPOINTSREQUEST :
        {
            /* Response is checked at Data indication Callback */
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
        }
        break;

        case ZDPSENDPOWERDESCRIPTORREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
          /* Response is checked at Data indication Callback */
        }
        break;

        case ZDPSENDNODEDESCRIPTORREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
          /* Response is checked at Data indication Callback */
        }
        break;

        case ZIGBEESIMPLEDESCRIPTORREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
        }
        break;

        case ZIGBEESENDUNICASTDATA :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_send_unicast_data:");
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
          rsi_zigb_get_max_aps_payload_length();
        }
        break;

        case ZIGBEEGETMAXAPSPAYLOADLENGTH :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_max_aps_payload_length:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_get_key_table_entry(api_test_var->key_index, &api_test_var->KeyStruct);
        }
        break;

        case ZIGBEEGETKEYTABLEENTRY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_key_table_entry:");
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
          rsi_zigb_get_key(g_Network_Key_c);
        }
        break;

        case ZIGBEEGETKEY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_key:");
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
          rsi_zigb_have_link_key(api_test_var->parent_ieee);
        }
        break;

        case ZIGBEEHAVELINKKEY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_have_link_key:");
          if(resp->uCmdRspPayLoad.statusResp.status == TRUE)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_request_link_key(api_test_var->parent_ieee , api_test_var->parent_ieee);
        }
        break;

        case ZIGBEEREQUESTLINKKEY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_request_link_key:");
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
          api_test_var->key_index = 0;
          rsi_zigb_set_key_table_entry(0, api_test_info->SelfAddress.IEEE_address, g_Network_Key_c, api_test_var->link_key_ptr);
        }
        break;

        case ZIGBEESETKEYTABLEENTRY:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_key_table_entry:");
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

          rsi_zigb_add_or_update_key_table_entry(api_test_info->SelfAddress.IEEE_address, 1, api_test_var->link_key_ptr, 0);
        }
        break;

        case ZIGBEEADDORUPDATEKEYTABLEENTRY:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_add_or_update_key_table_entry:");
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
          rsi_zigb_find_key_table_entry(api_test_var->parent_ieee, g_Network_Key_c);
        }
        break;

        case ZIGBEEFINDKEYTABLEENTRY:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_find_key_table_entry:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_erase_key_table_entry(0x1);
        }
        break;

        case ZIGBEEERASEKEYTABLEENTRY:
        {
          rsi_zigb_mcpy( api_test_info->SelfAddress.IEEE_address, api_test_var->SetBindingEntry.a_src_addr,8);
          api_test_var->SetBindingEntry.src_endpoint = api_test_info->Source_EP;
          *(uint16_t*)api_test_var->SetBindingEntry.a_cluster_id = CUSTOM_CLUSTER;
          api_test_var->SetBindingEntry.dest_addr_mode = api_test_info->DestAddrMode;
          rsi_zigb_mcpy( api_test_info->ParentAddress.IEEE_address,api_test_var->SetBindingEntry.a_dest_addr, 8);
          api_test_var->SetBindingEntry.dest_endpoint = api_test_info->Dest_EP;

          RSI_DPRINT(RSI_PL1, "rsi_zigb_erase_key_table_entry:");
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
          rsi_zigb_set_binding_entry(&api_test_var->SetBindingEntry);
        }
        break;

        case ZIGBEESETBINDINGENTRY:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_binding_entry:");
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
          rsi_zigb_delete_binding(0);
        }
        break;

        case ZIGBEEDELETEBINDING:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_delete_binding:");
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
          rsi_zigb_is_binding_entry_active(api_test_info->Dest_EP);
        }
        break;

        case ZIGBEEISBINDINGENTRYACTIVE:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_is_binding_entry_active:");
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
          rsi_zigb_clear_binding_table();
        }
        break;

        case ZIGBEECLEARBINDINGTABLE:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_clear_binding_table:");
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
          rsi_zigb_end_device_poll_for_data();
          rsi_zigb_delay(1);//Dont remove this sleep ,it is kept to synchronize the responses received from Parent
        }
        break;

        case ZIGBEEBINDREQUEST:
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
          /* Response is checked at Data indication Callback */
        }
        break;

        case ZIGBEEUNBINDREQUEST:
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
            rsi_zigb_end_device_poll_for_data();
          }
          /* Response is checked at Data indication Callback */
        }
        break;
        case ZIGBEELEAVENETWORK:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_leave_network:");
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

          rsi_zigb_delay(1);//if sleep is removed the leave network is not seen over the air
          rsi_zigb_set_operating_channel(12);
        }
        break;

        case ZIGBEESETOPERATINGCHANNEL :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_operating_channel:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c, 
                                 app_info->channel_mask ,
                                 app_info->scan_duration );
        }
        break;

        case ZIGBEEINITIATESCAN :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_initiate_scan:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_stop_scan();
        }
        break;

        case ZIGBEESTOPSCAN :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_stop_scan:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          RSI_DPRINT(RSI_PL1,"\n ****** End of API_TEST ******* \n");
          return 0;
        }
        break;

        case ZIGBEENETWORKRESTORE:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_network_restore:");
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
          rsi_zigb_leave_network();
        }
        break;

        case APPHANDLEDATACONFIRMATION:
        {
          if(!api_test_var->data_cnrfm_count)
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_data_confirmation:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          api_test_var->data_cnrfm_count++;
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPHANDLEDATAINDICATION:
        {
          if(!api_test_var->data_indication_count)
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_data_indication:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
         api_test_var->data_indication_count++;
         rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPSCANCOMPLETEHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPENERGYSCANRESULTHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPNETWORKFOUNDHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPZIGBEESTACKSTATUSHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case ZIGBEECHILDJOINHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPINCOMINGMANYTOONEROUTEREQUESTHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPADDRESSCONFLICTHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        default:
        {
          RSI_DPRINT(RSI_PL1,"Unknown state for the received resp. cmd_id: %x   \n",api_test_var->cmd);
        }
        break;
      }
    }
  }
  return RSI_ZB_FAIL;
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

void rsi_zigb_api_test_data_indication_handler(APSDE_Data_Indication_t *pDataIndication)
{
  rsi_zigb_app_cb_t *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_apitest_t  *api_test_info = &rsi_zigb_apitest;
  rsi_zigb_apitest_var_t  *api_test_var = &rsi_zigb_apitest_var;
  ZigBeeDeviceType device_type = app_cb_ptr->device_type;
  
  static int power_desc_count = 0,node_desc_count = 0,bind_count = 0,unbind_count = 0,active_ep_count = 0,ieee_addr_count = 0;
  static int simple_desc_count = 0,network_addr_count = 0,match_desc_count = 0; 
  rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;

#if 1
  int i;
  printf("\n Called AppHandleDataIndication \n ");
  printf("\n Rcvd Data from : 0x%x", pDataIndication->dest_address.short_address);
  printf("\n Rcvd Data dest ep : 0x%x", pDataIndication->dest_endpoint);
  printf("\n Rcvd Data src ep : 0x%x", pDataIndication->src_endpoint);
  printf("\n Data of size 0x%x = ", pDataIndication->asdulength);
  printf("\n Rcvd profile id : 0x%x", pDataIndication->profile_id);
  printf("\n Rcvd cluster id : 0x%x", pDataIndication->cluster_id);
  printf("\n Rcvd security_status : 0x%x\n", pDataIndication->security_status);

  for(i = 0; i < pDataIndication->asdulength; i++)
    printf("  0x%x", pDataIndication->a_asdu[i]);

  printf("\n");
#endif



  if( pDataIndication->cluster_id == 0x8003)//0x8003: power descriptor response
  { 
    if(!power_desc_count){
     RSI_DPRINT(RSI_PL1,"rsi_zigb_zdp_send_power_descriptor_request:");
    }
    if(pDataIndication->a_asdu[1] == g_SUCCESS_c)
    {
      if(!power_desc_count){
        RSI_DPRINT(RSI_PL1," SUCCESS\n");
        TCSUCCESS
      }
      rsi_zigb_mcpy( pDataIndication->a_asdu,
          app_info->zb_resp_info.powerDescResp, 
          pDataIndication->asdulength);
      app_info->status_var.powerDescRspStatus = 0x00;
    }
    else
    {
      if(!power_desc_count){
        RSI_DPRINT(RSI_PL1," FAIL\n");
        TCFAIL
      }
    }
    power_desc_count++;

    if(device_type == ZigBeeRouter) {
      rsi_zigb_zdp_send_node_descriptor_request(api_test_var->parent_short_addr,api_test_var->aps_ack_required);
    } else if(device_type == ZigBeeCoordinator) {
      rsi_zigb_zdp_send_node_descriptor_request(app_cb_ptr->short_addr,api_test_var->aps_ack_required);
    } else {
      /* Do nothing */
    }
  }

  if( pDataIndication->cluster_id == 0x8002)//0x8002: node descriptor response
  {
    if(!node_desc_count){
      RSI_DPRINT(RSI_PL1, "rsi_zigb_zdp_send_node_descriptor_request:");
    }
    if(pDataIndication->a_asdu[1] == g_SUCCESS_c)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
          app_info->zb_resp_info.nodeDescResp, 
          pDataIndication->asdulength);
      app_info->status_var.nodeDescRspStatus = 0x00;
      if(!node_desc_count){
        RSI_DPRINT(RSI_PL1," SUCCESS\n");
        TCSUCCESS
      }
    }
    else
    {
      if(!node_desc_count){
        RSI_DPRINT(RSI_PL1," FAIL\n");
        TCFAIL
      }
    }
    node_desc_count++;
    
    if(device_type == ZigBeeRouter) {
      rsi_zigb_simple_descriptor_request(api_test_var->parent_short_addr,api_test_var->end_pont_id);
    } else if(device_type == ZigBeeCoordinator) {
      rsi_zigb_simple_descriptor_request(app_cb_ptr->short_addr,api_test_var->end_pont_id);
    } else {
      /* Do nothing */
    }
  }

  if( pDataIndication->cluster_id == 0x8021)//0x8021: bind response
  {
    if(!bind_count){
      RSI_DPRINT(RSI_PL1,"rsi_zigb_bind_request:");
      RSI_DPRINT(RSI_PL1," SUCCESS\n");
      TCSUCCESS
    }
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
          app_info->zb_resp_info.bindResp, 
          pDataIndication->asdulength);
      app_info->status_var.bindRspStatus = 0x00;

      if((device_type == ZigBeeRouter) ) {
        rsi_zigb_unbind_request(api_test_var->parent_short_addr, api_test_info->SelfAddress.IEEE_address, api_test_info->Source_EP, api_test_info->ClusterId, api_test_info->DestAddrMode, api_test_info->ParentAddress,api_test_info->Dest_EP, api_test_info->APSAckRequired);
      }
      else if (device_type == ZigBeeCoordinator) {
        rsi_zigb_unbind_request(app_cb_ptr->short_addr, api_test_info->SelfAddress.IEEE_address, api_test_info->Source_EP, api_test_info->ClusterId, api_test_info->DestAddrMode, api_test_info->ParentAddress,api_test_info->Dest_EP, api_test_info->APSAckRequired);
      }
      else {
        /* Do nothing */
      }
    }
    bind_count++;
  }

  if( pDataIndication->cluster_id == 0x8022)//0x8022: unbind response
  {
    if(!unbind_count){
      RSI_DPRINT(RSI_PL1,"rsi_zigb_unbind_request:");
      TCSUCCESS
      RSI_DPRINT(RSI_PL1, " SUCCESS\n");
    }
    if(pDataIndication->a_asdu[1] == 0x84)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
          app_info->zb_resp_info.unbindResp, 
          pDataIndication->asdulength);
      app_info->status_var.unbindRspStatus = 0x00;
    }
    unbind_count++;

      if((device_type == ZigBeeRouter) || (device_type == ZigBeeCoordinator)) {
      rsi_zigb_network_restore();
    }
  }

  if( pDataIndication->cluster_id == 0x8005)//0x8005: active endpoint response
  {
    if(!active_ep_count){
      RSI_DPRINT(RSI_PL1,"rsi_zigb_active_endpoints_request:");
    }
    if(pDataIndication->a_asdu[1] == g_SUCCESS_c)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
          app_info->zb_resp_info.actepResp, 
          pDataIndication->asdulength);
      app_info->status_var.actepRspStatus = 0x00;
      if(!active_ep_count){
        RSI_DPRINT(RSI_PL1," SUCCESS\n");
        TCSUCCESS
      }
    }
    else
    {
      if(!active_ep_count){
        RSI_DPRINT(RSI_PL1," FAIL\n");
        TCFAIL
      }
    }
    active_ep_count++;
    
    if(device_type == ZigBeeRouter) {
      rsi_zigb_zdp_send_power_descriptor_request(api_test_var->parent_short_addr,api_test_var->aps_ack_required);
    } else if(device_type == ZigBeeCoordinator) {
      rsi_zigb_zdp_send_power_descriptor_request(app_cb_ptr->short_addr,api_test_var->aps_ack_required);
    } else {
      /* Do nothing */
    }
  }

  if( pDataIndication->cluster_id == 0x8001)//0x8001: ieee addr response
  {
    if(!ieee_addr_count){
      RSI_DPRINT(RSI_PL1, "rsi_zigb_zdp_send_ieee_addr_request:");
    }
    if(pDataIndication->a_asdu[1] == g_SUCCESS_c)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
          app_info->zb_resp_info.ieeeaddrResp, 
          pDataIndication->asdulength);
      app_info->status_var.ieeeaddrRspStatus = 0x00;
      if(!ieee_addr_count){
        RSI_DPRINT(RSI_PL1, " SUCCESS\n");
        TCSUCCESS
      }
    }
    else
    {
      if(!ieee_addr_count){
        RSI_DPRINT(RSI_PL1," FAIL\n");
        TCFAIL
      }
    }
    ieee_addr_count++;
    if(device_type == ZigBeeRouter) {
      rsi_zigb_send_match_descriptors_request(api_test_var->parent_short_addr, 
        app_info->DeviceSimpleDesc->app_profile_id, 
        (uint8_t*)app_info->DeviceSimpleDesc->p_incluster_list,
        app_info->DeviceSimpleDesc->incluster_count, 
        (uint8_t*) app_info->DeviceSimpleDesc->p_outcluster_list,
        app_info->DeviceSimpleDesc->outcluster_count, 1, api_test_var->broad_cast_addr);
    } else if(device_type == ZigBeeCoordinator) {
      rsi_zigb_send_match_descriptors_request(app_cb_ptr->short_addr, 
        app_info->DeviceSimpleDesc->app_profile_id, 
        (uint8_t*)app_info->DeviceSimpleDesc->p_incluster_list,
        app_info->DeviceSimpleDesc->incluster_count, 
        (uint8_t*) app_info->DeviceSimpleDesc->p_outcluster_list,
        app_info->DeviceSimpleDesc->outcluster_count, 1, api_test_var->broad_cast_addr);
    } else {
      /* Do nothing */
    }
        
  }

  if( pDataIndication->cluster_id == 0x8004)//0x8004: simple desc response
  {
    if(!simple_desc_count){
      RSI_DPRINT(RSI_PL1, "rsi_zigb_simple_descriptor_request:");
    }
    if(pDataIndication->a_asdu[1] == g_SUCCESS_c)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
          app_info->zb_resp_info.simpledespResp, 
          pDataIndication->asdulength);
      app_info->status_var.simpledescRspStatus = 0x00;
      if(!simple_desc_count){
        RSI_DPRINT(RSI_PL1, " SUCCESS\n");
        TCSUCCESS
      }
    }
    else
    {
      if(!simple_desc_count){
        RSI_DPRINT(RSI_PL1," FAIL\n");
        TCFAIL
      }
    }
    simple_desc_count++;
    
    if((device_type == ZigBeeRouter)) { 
      rsi_zigb_bind_request(api_test_var->parent_short_addr , api_test_info->SelfAddress.IEEE_address, api_test_info->Source_EP, api_test_info->ClusterId, api_test_info->DestAddrMode, api_test_info->ParentAddress,api_test_info->Dest_EP, api_test_info->APSAckRequired);
    } else if(device_type == ZigBeeCoordinator) {
      rsi_zigb_bind_request(app_cb_ptr->short_addr, api_test_info->SelfAddress.IEEE_address, 
                          api_test_info->Source_EP, api_test_info->ClusterId,
                          2, api_test_info->ParentAddress ,
                          api_test_info->Dest_EP, 
                          api_test_info->APSAckRequired);
    } else {
      /* Do nothing */
    }
  }

  if( pDataIndication->cluster_id == 0x8000)//0x8000: network addr response
  {
    if(!network_addr_count){
      RSI_DPRINT(RSI_PL1, "rsi_zigb_zdp_send_nwk_addr_request:");
    }
    if(pDataIndication->a_asdu[1] == g_SUCCESS_c)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu,
          app_info->zb_resp_info.networkaddrResp, 
          pDataIndication->asdulength);
      app_info->status_var.networkaddrRspStatus = 0x00;
      if(!network_addr_count){
        RSI_DPRINT(RSI_PL1, " SUCCESS\n");
        TCSUCCESS
      }
    }
    else
    {
      if(!network_addr_count){
        RSI_DPRINT(RSI_PL1," FAIL\n");
        TCFAIL
      }
    }
    network_addr_count++;
    
    if(device_type == ZigBeeRouter) {
      rsi_zigb_zdp_send_ieee_addr_request(api_test_var->parent_short_addr,api_test_var->request_type,api_test_var->start_index,api_test_var->aps_ack_required);
    } else if(device_type == ZigBeeCoordinator) {
      rsi_zigb_zdp_send_ieee_addr_request(app_cb_ptr->short_addr,(BOOL)TRUE,api_test_var->start_index,api_test_var->aps_ack_required);
    } else {
      /* Do nothing */
    }
  }

  if( pDataIndication->cluster_id == 0x8006)//0x8006: Match decs response
  {
    if(!match_desc_count){
      RSI_DPRINT(RSI_PL1, "rsi_zigb_send_match_descriptors_request:");
    }
    if(pDataIndication->a_asdu[1] == g_SUCCESS_c)
    {
      rsi_zigb_mcpy( pDataIndication->a_asdu, 
          app_info->zb_resp_info.matchDescResp,
          pDataIndication->asdulength);
      app_info->status_var.matchDescRspStatus = 0x00;
      if(!match_desc_count){
        RSI_DPRINT(RSI_PL1, " SUCCESS\n");
        TCSUCCESS
      }
    }
    else
    {
      if(!match_desc_count){
        RSI_DPRINT(RSI_PL1," FAIL\n");
        TCFAIL
      }
    }
    match_desc_count++;
    
    if(device_type == ZigBeeRouter) {
      rsi_zigb_active_endpoints_request( api_test_var->parent_short_addr, api_test_var->aps_ack_required );
    } else if(device_type == ZigBeeCoordinator) {
      rsi_zigb_active_endpoints_request( app_cb_ptr->short_addr, api_test_var->aps_ack_required );
    } else {
      /* Do nothing */
    }
  }

}


/*===========================================================================
 *
 * @fn          void rsi_zigb_router_api_test(void)
 * @brief       Handle API test cases
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to test the sequence of flow of API test
 *
 * ===========================================================================*/

RSI_ZB_STATUS rsi_zigb_router_api_test()
{
  rsi_zigb_app_info_t     *app_info = &rsi_zigb_app_info;
  rsi_zigb_apitest_t      *api_test_info = &rsi_zigb_apitest;
  rsi_zigb_apitest_var_t  *api_test_var = &rsi_zigb_apitest_var;
  rsi_zigb_app_cb_t       *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_uCmdRsp        *resp;
  uint8_t                 stack_is_up = 2; 
  /* Initialize variables used in apitest */
  rsi_zigb_api_var_init();
  resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);
 
  /* Enabling Permit join */ 
   rsi_zigb_network_state();

  while (1)
  {
    if(app_cb_ptr->pkt_pending == RSI_TRUE) 
    {
      if (app_cb_ptr->fsm_state != FSM_API_TEST) 
      {
        RSI_DPRINT(RSI_PL1,"Network Down, Not in connected state \n");
        return RSI_ZB_FAIL;
      }
      api_test_var->rsi_status = rsi_frame_read(app_cb_ptr->read_packet_buffer);
      if (api_test_var->rsi_status != RSI_ZB_SUCCESS)
      {
        RSI_DPRINT(RSI_PL1,"Error in dequeuing pkt \n");
        return RSI_ZB_FAIL;
      }

      api_test_var->cmd = resp->cmd_id;
      switch(api_test_var->cmd)
      {
        case ZIGBEENETWORKSTATE:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_network_state:");
          if((resp->uCmdRspPayLoad.statusResp.status == stack_is_up) ||
             (resp->uCmdRspPayLoad.statusResp.status == g_SUCCESS_c))
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          
//          return RSI_ZB_SUCCESS;
#if 0
          app_info->DeviceSimpleDesc = &On_Off_Light_Simple_Desc;
            rsi_zigb_set_simple_descriptor(ONOFF_LIGHT_END_POINT,
                app_info->DeviceSimpleDesc);
#endif
           rsi_zigb_set_simple_descriptor(ONOFF_SWITCH_END_POINT,app_info->DeviceSimpleDesc);

        }
        break;

        case  ZIGBEESETSIMPLEDESCRIPTOR:
        {
          /*All the below apis are passed by this time */
          RSI_DPRINT(RSI_PL1, "rsi_zigb_init_stack:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_reset_stack:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_device_type:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_app_scan_complete_handler:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_app_network_found_handler:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_app_stack_status_handler:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_join_network:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_simple_descriptor:");
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
          rsi_zigb_stack_is_up();
        }
        break;

        case ZIGBEESTACKISUP :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_stack_is_up:");
          if(resp->uCmdRspPayLoad.statusResp.status == 0x1)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_self_ieee_address();
        }
        break;

        case ZIGBEEGETSELFIEEEADDRESS :
        {
          rsi_zigb_mcpy( resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee , api_test_info->SelfAddress.IEEE_address , 8);

          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_self_ieee_address:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_is_it_self_ieee_address(resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee );
        }
        break;

        case ZIGBEEISITSELFIEEEADDRESS :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_is_it_self_ieee_address:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_get_self_short_address();
        }
        break;

        case ZIGBEEGETSELFSHORTADDRESS :
        {
          api_test_var->short_addr_t = resp->uCmdRspPayLoad.DevShortAddr.short_addr;
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_self_short_address:");
          if(api_test_var->short_addr_t != 0x0000)
          { 
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_set_manufacturer_code_for_node_desc(api_test_var->ManufacturerCode);
        }
        break;

        case ZIGBEESETMANUFACTURERCODEFORNODEDESC :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_manufacturer_code_for_node_desc:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_set_power_descriptor(&api_test_var->nodePowerDesc_t);
        }
        break;

        case ZIGBEESETPOWERDESCRIPTOR :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_power_descriptor:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_set_maxm_incoming_txfr_size(0x1234);
        }
        break;

        case ZIGBEESETMAXMINCOMINGTXFRSIZE :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_maxm_incoming_txfr_size:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_set_maxm_out_going_txfr_size(0x1234);

        }
        break;

        case ZIGBEESETMAXMOUTGOINGTXFRSIZE :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_maxm_out_going_txfr_size:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_get_operating_channel();
        }
        break;

        case ZIGBEEGETOPERATINGCHANNEL :
        {
          api_test_var->channel = resp->uCmdRspPayLoad.DevChannel.channel;

          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_operating_channel:");
          if(api_test_var->channel != 0x00 ) 
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_short_panid();
        }
        break;

        case ZIGBEEGETSHORTPANID :
        {
          api_test_var->pan_Id = resp->uCmdRspPayLoad.DevPanId.pan_id; 

          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_short_panid:");
          if(api_test_var->pan_Id != 0x0000)
          { 
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_extended_panid();
        }
        break;

        case ZIGBEEGETEXTENDEDPANID :
        {
          rsi_zigb_mcpy(resp->uCmdRspPayLoad.Extnd_PanId.Ext_Pan_Id ,api_test_var->Ext_PanId , 8);
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_extended_panid:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS
          rsi_zigb_get_endpoint_id(0x1);
        }
        break;

        case ZIGBEEGETENDPOINTID :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_endpoint_id:");
          if(resp->uCmdRspPayLoad.statusResp.status == 0xF1)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_simple_descriptor(ONOFF_SWITCH_END_POINT);
        }
        break;
 
        case ZIGBEEGETSIMPLEDESCRIPTOR :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_simple_descriptor:");
          if(resp->uCmdRspPayLoad.statusResp.status == TRUE)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_endpoint_cluster(0x00,0x00,0x00);
        }
        break;

        case ZIGBEEGETENDPOINTCLUSTER :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_endpoint_cluster:");

          api_test_var->endpoint_cluster = resp->uCmdRspPayLoad.EpClusterResp.cluster ;
          if(api_test_var->endpoint_cluster == 0xffff)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
              rsi_zigb_stack_profile();
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
        }
        break;

        case ZIGBEESTACKPROFILE :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_stack_profile:");
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
          rsi_zigb_get_parent_ieee_address();
        }
        break;

        case ZIGBEEGETPARENTIEEEADDRESS :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_parent_ieee_address:");             
          rsi_zigb_mcpy( resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee ,api_test_info->ParentAddress.IEEE_address , 8);
          rsi_zigb_mcpy( api_test_info->ParentAddress.IEEE_address , api_test_var->parent_ieee,8);

          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

            rsi_zigb_get_parent_short_address();
        }
        break;

        case ZIGBEEGETPARENTSHORTADDRESS :
        {
          api_test_var->parent_short_addr  =resp->uCmdRspPayLoad.DevShortAddr.short_addr;
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_parent_short_address:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_get_ieee_addr_for_specified_short_addr(api_test_var->parent_short_addr );
        }
        break;

        case ZIGBEEGETIEEEADDRFORSPECIFIEDSHORTADDR :
        {
          rsi_zigb_mcpy( resp->uCmdRspPayLoad.GetIEEEResp.Ieee_Add ,api_test_var->get_ieee , 8);
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_ieee_addr_for_specified_short_addr:");

          if((rsi_zigb_mcmp(api_test_var->get_ieee,api_test_var->parent_ieee,8)) == RSI_ZB_SUCCESS)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_short_addr_for_specified_ieee_addr( api_test_var->parent_ieee);
        }
        break;


        case ZIGBEEGETSHORTADDRFORSPECIFIEDIEEEADDR :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_short_addr_for_specified_ieee_addr:");
          
          if(resp->uCmdRspPayLoad.DevShortAddr.short_addr == api_test_var->parent_short_addr)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_read_neighbor_table_entry( 0x0);
        }
        break;

        case ZIGBEEREADNEIGHBORTABLEENTRY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_read_neighbor_table_entry:");
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
          rsi_zigb_tree_depth();
        }
        break;


        case ZIGBEETREEDEPTH :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_tree_depth:");
          if(resp->uCmdRspPayLoad.statusResp.status == 1)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_get_neighbor_table_entry_count();
        }
        break;

        case ZIGBEEGETNEIGHBORTABLEENTRYCOUNT :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_neighbor_table_entry_count:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_zdp_send_device_announcement();
        }
        break;

        case ZDPSENDNWKADDRREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
        }
        break;

        case ZDPSENDIEEEADDRREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
        }
        break;

        case ZDPSENDDEVICEANNOUNCEMENT :
        {

          RSI_DPRINT(RSI_PL1, "rsi_zigb_zdp_send_device_announcement:");
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
          rsi_zigb_send_unicast_data(ZigBee_Outgoing_Direct, api_test_var->DestAddress,&api_test_var->APSDEDataReq);
        }
        break;

        case ZDPSENDMATCHDESCRIPTORSREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
          /* Response is checked at Data indication Callback */
        }
        break;

        case ZIGBEEACTIVEENDPOINTSREQUEST :
        {
            /* Response is checked at Data indication Callback */
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
        }
        break;

        case ZDPSENDPOWERDESCRIPTORREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
          /* Response is checked at Data indication Callback */
        }
        break;

        case ZDPSENDNODEDESCRIPTORREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
          /* Response is checked at Data indication Callback */
        }
        break;

        case ZIGBEESIMPLEDESCRIPTORREQUEST :
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
        }
        break;

        case ZIGBEESENDUNICASTDATA :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_send_unicast_data:");
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
            rsi_zigb_send_broadcast_data(&api_test_var->APSDEDataReq);
          }
          break;

        case ZIGBEESENDBROADCASTDATA :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_send_broadcast_data:");
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
            rsi_zigb_send_group_data(0x0000,&api_test_var->APSDEDataReq);
          }
          break;

        case ZIGBEESENDGROUPDATA :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_send_group_data:");
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
            rsi_zigb_get_max_aps_payload_length();
          }
          break;


        case ZIGBEEGETMAXAPSPAYLOADLENGTH :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_max_aps_payload_length:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_get_key_table_entry(api_test_var->key_index, &api_test_var->KeyStruct);
        }
        break;

        case ZIGBEEGETKEYTABLEENTRY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_key_table_entry:");
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
          rsi_zigb_get_key(g_Network_Key_c);
        }
        break;

        case ZIGBEEGETKEY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_key:");
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
          rsi_zigb_have_link_key(api_test_var->parent_ieee);
        }
        break;

        case ZIGBEEHAVELINKKEY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_have_link_key:");
          if(resp->uCmdRspPayLoad.statusResp.status == TRUE)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_request_link_key(api_test_var->parent_ieee , api_test_var->parent_ieee);
        }
        break;

        case ZIGBEEREQUESTLINKKEY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_request_link_key:");
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
          api_test_var->key_index = 0;
          rsi_zigb_set_key_table_entry(0, api_test_info->SelfAddress.IEEE_address, g_Network_Key_c, api_test_var->link_key_ptr);
        }
        break;

        case ZIGBEESETKEYTABLEENTRY:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_key_table_entry:");
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

          rsi_zigb_add_or_update_key_table_entry(api_test_info->SelfAddress.IEEE_address, 1, api_test_var->link_key_ptr, 0);
        }
        break;

        case ZIGBEEADDORUPDATEKEYTABLEENTRY:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_add_or_update_key_table_entry:");
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
          rsi_zigb_find_key_table_entry(api_test_var->parent_ieee, g_Network_Key_c);
        }
        break;

        case ZIGBEEFINDKEYTABLEENTRY:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_find_key_table_entry:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_erase_key_table_entry(0x1);
        }
        break;

        case ZIGBEEERASEKEYTABLEENTRY:
        {
          rsi_zigb_mcpy( api_test_info->SelfAddress.IEEE_address, api_test_var->SetBindingEntry.a_src_addr,8);
          api_test_var->SetBindingEntry.src_endpoint = api_test_info->Source_EP;
          *(uint16_t*)api_test_var->SetBindingEntry.a_cluster_id = CUSTOM_CLUSTER;
          api_test_var->SetBindingEntry.dest_addr_mode = api_test_info->DestAddrMode;
          rsi_zigb_mcpy( api_test_info->ParentAddress.IEEE_address,api_test_var->SetBindingEntry.a_dest_addr, 8);
          api_test_var->SetBindingEntry.dest_endpoint = api_test_info->Dest_EP;

          RSI_DPRINT(RSI_PL1, "rsi_zigb_erase_key_table_entry:");
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
          rsi_zigb_set_binding_entry(&api_test_var->SetBindingEntry);
        }
        break;

        case ZIGBEESETBINDINGENTRY:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_binding_entry:");
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
          rsi_zigb_delete_binding(0);
        }
        break;

        case ZIGBEEDELETEBINDING:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_delete_binding:");
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
          rsi_zigb_is_binding_entry_active(api_test_info->Dest_EP);
        }
        break;

        case ZIGBEEISBINDINGENTRYACTIVE:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_is_binding_entry_active:");
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
          rsi_zigb_clear_binding_table();
        }
        break;

        case ZIGBEECLEARBINDINGTABLE:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_clear_binding_table:");
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
          rsi_zigb_get_route_table_entry(api_test_var->child_index);
        }
        break;

        case ZIGBEEGETROUTETABLEENTRY :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_route_table_entry:");
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

          rsi_zigb_get_child_short_address_for_the_index(api_test_var->child_index);
        }
        break;


        case ZIGBEEGETCHILDSHORTADDRESSFORTHEINDEX :
        {
          
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_child_short_address_for_the_index:");
          api_test_var->child_short_addr_t = resp->uCmdRspPayLoad.DevShortAddr.short_addr;
          if(api_test_var->child_short_addr_t != 0x0000)
          { 
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }

          rsi_zigb_get_child_index_for_specified_short_addr(api_test_var->child_short_addr_t);
        }
        break;

        case ZIGBEEGETCHILDINDEXFORSPECIFIEDSHORTADDR :
        {
          
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_child_index_for_specified_short_addr:");
          
          if(resp->uCmdRspPayLoad.statusResp.status == 0xFF)
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }


          rsi_zigb_get_child_details(api_test_var->child_index);
        }
        break;


        case ZIGBEEGETCHILDDETAILS :
        {
          uint8_t ZigBee_No_Entry = 0x11; 
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_child_details:");
          
          if((resp->uCmdRspPayLoad.GetChildDetails.status == ZigBee_No_Entry) ||
             (resp->uCmdRspPayLoad.GetChildDetails.status == g_SUCCESS_c))
          {
            rsi_zigb_mcpy( resp->uCmdRspPayLoad.GetChildDetails.Ieee_Addr, api_test_var->child_ieee , 8);
            api_test_var->child_dev_type = resp->uCmdRspPayLoad.GetChildDetails.device_type;
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }

          rsi_zigb_read_count_of_child_devices();
        }
        break;

        case ZIGBEEREADCOUNTOFCHILDDEVICES :
        {
          api_test_var->child_dev_count = resp->uCmdRspPayLoad.statusResp.status;
          RSI_DPRINT(RSI_PL1, "rsi_zigb_read_count_of_child_devices:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

            rsi_zigb_read_count_of_router_child_devices();
        }
        break;

        case ZIGBEEREADCOUNTOFROUTERCHILDDEVICES :
        {
          api_test_var->rout_child_dev_count = resp->uCmdRspPayLoad.statusResp.status;
          RSI_DPRINT(RSI_PL1, "rsi_zigb_read_count_of_router_child_devices:");
          RSI_DPRINT(RSI_PL1, " SUCCESS\n");
          TCSUCCESS

          rsi_zigb_zdp_send_nwk_addr_request(api_test_var->parent_ieee,(BOOL)0,0 );
        }
        break;


        case ZIGBEEBINDREQUEST:
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
          /* Response is checked at Data indication Callback */
        }
        break;

        case ZIGBEEUNBINDREQUEST:
        {
          if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
          {
            app_info->wait_for_rsp = 0;
          }
          /* Response is checked at Data indication Callback */
        }
        break;
        case ZIGBEELEAVENETWORK:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_leave_network:");
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

          rsi_zigb_delay(1);//if sleep is removed the leave network is not seen over the air
          rsi_zigb_set_operating_channel(12);
        }
        break;

        case ZIGBEESETOPERATINGCHANNEL :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_set_operating_channel:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c, 
                                 app_info->channel_mask ,
                                 app_info->scan_duration );
        }
        break;

        case ZIGBEEINITIATESCAN :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_initiate_scan:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          rsi_zigb_stop_scan();
        }
        break;

        case ZIGBEESTOPSCAN :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_stop_scan:");
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
        }
        break;

        case ZIGBEENETWORKRESTORE:
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_network_restore:");
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
          rsi_zigb_leave_network();
        }
        break;



        case APPHANDLEDATACONFIRMATION:
        {
          if(!api_test_var->data_cnrfm_count)
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_data_confirmation:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          api_test_var->data_cnrfm_count++;
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPHANDLEDATAINDICATION:
        {
          if(!api_test_var->data_indication_count)
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_data_indication:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
         api_test_var->data_indication_count++;
         rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPSCANCOMPLETEHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          RSI_DPRINT(RSI_PL1,"\n ****** End of API_TEST ******* \n");
          return 0;
        }
        break;

        case APPENERGYSCANRESULTHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPNETWORKFOUNDHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPZIGBEESTACKSTATUSHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case ZIGBEECHILDJOINHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPINCOMINGMANYTOONEROUTEREQUESTHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        case APPADDRESSCONFLICTHANDLER:
        {
          rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
        }
        break;

        default:
        {
          RSI_DPRINT(RSI_PL1,"Unknown state for the received resp. cmd_id: %x   \n",api_test_var->cmd);
        }
        break;
      }
    }

  }
  return RSI_ZB_FAIL;
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_cord_api_test(void)
 * @brief       Handle API test cases
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to test the sequence of flow of API test
 *
 * ===========================================================================*/

RSI_ZB_STATUS rsi_zigb_cord_api_test()
{
  rsi_zigb_app_info_t     *app_info = &rsi_zigb_app_info;
  rsi_zigb_apitest_t      *api_test_info = &rsi_zigb_apitest;
  rsi_zigb_apitest_var_t  *api_test_var = &rsi_zigb_apitest_var;
  rsi_zigb_app_cb_t       *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_zigb_uCmdRsp        *resp;
  
  /* Initialize variables used in apitest */
  rsi_zigb_api_var_init();
  resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);

  /* Enabling Permit join */ 
  rsi_zigb_stack_is_up();    

  while (1)
  {
    if(app_cb_ptr->pkt_pending == RSI_TRUE) 
    {
      if (app_cb_ptr->fsm_state != FSM_ZB_FORMED) 
      {
        RSI_DPRINT(RSI_PL1,"Network Down, Not in connected state \n");
        return RSI_ZB_FAIL;
      }
      api_test_var->rsi_status = rsi_frame_read(app_cb_ptr->read_packet_buffer);
      if (api_test_var->rsi_status != RSI_ZB_SUCCESS)
      {
        RSI_DPRINT(RSI_PL1,"Error in dequeuing pkt \n");
        return RSI_ZB_FAIL;
      }

      api_test_var->cmd = resp->cmd_id;
      switch(api_test_var->cmd)
      {
        case ZIGBEESTACKISUP:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_stack_is_up:");
            if(resp->uCmdRspPayLoad.statusResp.status == TRUE)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }

           //return RSI_ZB_SUCCESS;
           //app_info->DeviceSimpleDesc = &On_Off_Light_Simple_Desc;
           //rsi_zigb_set_simple_descriptor(ONOFF_LIGHT_END_POINT,app_info->DeviceSimpleDesc);
            rsi_zigb_set_simple_descriptor(ONOFF_SWITCH_END_POINT,app_info->DeviceSimpleDesc);

          }
          break;

        case  ZIGBEESETSIMPLEDESCRIPTOR:
          {
            /*All the below apis are passed by this time */
            RSI_DPRINT(RSI_PL1, "rsi_zigb_init_stack:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            RSI_DPRINT(RSI_PL1, "rsi_zigb_reset_stack:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_device_type:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            RSI_DPRINT(RSI_PL1, "rsi_zigb_app_scan_complete_handler:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            RSI_DPRINT(RSI_PL1, "rsi_zigb_app_network_found_handler:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            RSI_DPRINT(RSI_PL1, "rsi_zigb_app_stack_status_handler:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            RSI_DPRINT(RSI_PL1, "rsi_zigb_form_network:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            RSI_DPRINT(RSI_PL1, "rsi_zigb_permit_join:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            RSI_DPRINT(RSI_PL1, "rsi_zigb_set_simple_descriptor:");
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
            rsi_zigb_get_self_ieee_address();
          }
          break;

        case ZIGBEEGETSELFIEEEADDRESS :
          {
            rsi_zigb_mcpy( resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee , api_test_info->SelfAddress.IEEE_address , 8);

            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_self_ieee_address:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            rsi_zigb_is_it_self_ieee_address(resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee );
          }
          break;

        case ZIGBEEISITSELFIEEEADDRESS :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_is_it_self_ieee_address:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            rsi_zigb_get_self_short_address();
          }
          break;

        case ZIGBEEGETSELFSHORTADDRESS :
          {
            api_test_var->short_addr_t = resp->uCmdRspPayLoad.DevShortAddr.short_addr;
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_self_short_address:");
            if(api_test_var->short_addr_t == 0x0000)
            { 
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_set_manufacturer_code_for_node_desc(api_test_var->ManufacturerCode);
          }
          break;

        case ZIGBEESETMANUFACTURERCODEFORNODEDESC :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_set_manufacturer_code_for_node_desc:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            rsi_zigb_set_power_descriptor(&api_test_var->nodePowerDesc_t);
          }
          break;

        case ZIGBEESETPOWERDESCRIPTOR :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_set_power_descriptor:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            rsi_zigb_set_maxm_incoming_txfr_size(0x1234);
          }
          break;

        case ZIGBEESETMAXMINCOMINGTXFRSIZE :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_set_maxm_incoming_txfr_size:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            rsi_zigb_set_maxm_out_going_txfr_size(0x1234);

          }
          break;

        case ZIGBEESETMAXMOUTGOINGTXFRSIZE :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_set_maxm_out_going_txfr_size:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            rsi_zigb_get_operating_channel();
          }
          break;

        case ZIGBEEGETOPERATINGCHANNEL :
          {
            api_test_var->channel = resp->uCmdRspPayLoad.DevChannel.channel;

            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_operating_channel:");
            if(api_test_var->channel != 0x00 ) 
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_get_short_panid();
          }
          break;

        case ZIGBEEGETSHORTPANID :
          {
            api_test_var->pan_Id = resp->uCmdRspPayLoad.DevPanId.pan_id; 

            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_short_panid:");
            if(api_test_var->pan_Id != 0x0000)
            { 
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_get_extended_panid();
          }
          break;

        case ZIGBEEGETEXTENDEDPANID :
          {
            rsi_zigb_mcpy(resp->uCmdRspPayLoad.Extnd_PanId.Ext_Pan_Id ,api_test_var->Ext_PanId , 8);
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_extended_panid:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
              rsi_zigb_get_endpoint_id(0x1);
          }
          break;

        case ZIGBEEGETENDPOINTID :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_endpoint_id:");
            if(resp->uCmdRspPayLoad.statusResp.status == 0xF1)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_get_simple_descriptor(ONOFF_SWITCH_END_POINT);
          }
          break;

        case ZIGBEEGETSIMPLEDESCRIPTOR :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_simple_descriptor:");
            if(resp->uCmdRspPayLoad.statusResp.status == TRUE)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_get_endpoint_cluster(0x00,0x00,0x00);
          }
          break;

        case ZIGBEEGETENDPOINTCLUSTER :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_endpoint_cluster:");

            api_test_var->endpoint_cluster = resp->uCmdRspPayLoad.EpClusterResp.cluster ;
            if(api_test_var->endpoint_cluster == 0xffff)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
                rsi_zigb_stack_profile();
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
          }
          break;

        case ZIGBEESTACKPROFILE :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_stack_profile:");
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
            rsi_zigb_get_parent_ieee_address();
          }
          break;

        case ZIGBEEGETPARENTIEEEADDRESS :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_parent_ieee_address:");             
            rsi_zigb_mcpy( resp->uCmdRspPayLoad.DevIEEEAddr.Self_ieee ,api_test_info->ParentAddress.IEEE_address , 8);
            rsi_zigb_mcpy( api_test_info->ParentAddress.IEEE_address , api_test_var->parent_ieee,8);

            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

              rsi_zigb_get_parent_short_address();
          }
          break;

        case ZIGBEEGETPARENTSHORTADDRESS :
          {
            api_test_var->parent_short_addr  =resp->uCmdRspPayLoad.DevShortAddr.short_addr;
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_parent_short_address:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            //rsi_zigb_get_ieee_addr_for_specified_short_addr(0x00);
            rsi_zigb_tree_depth();
          }
          break;

        case ZIGBEEGETIEEEADDRFORSPECIFIEDSHORTADDR :
          {
            rsi_zigb_mcpy( resp->uCmdRspPayLoad.GetIEEEResp.Ieee_Add, api_test_var->get_ieee, 8);
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_ieee_addr_for_specified_short_addr:");

            if((!rsi_zigb_mcmp(api_test_var->get_ieee,app_cb_ptr->mac_addr,8)) == RSI_ZB_SUCCESS)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
	          rsi_zigb_get_short_addr_for_specified_ieee_addr( api_test_var->get_ieee);
          }
          break;


        case ZIGBEEGETSHORTADDRFORSPECIFIEDIEEEADDR :
        {
          RSI_DPRINT(RSI_PL1, "rsi_zigb_get_short_addr_for_specified_ieee_addr:");
          
          if(resp->uCmdRspPayLoad.DevShortAddr.short_addr ==  api_test_info->ParentAddress.short_address )
          {
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS
          }
          else
          {
            RSI_DPRINT(RSI_PL1," FAIL\n");
            TCFAIL
          }
          rsi_zigb_read_neighbor_table_entry( 0x0);
        }
        break;

        case ZIGBEEREADNEIGHBORTABLEENTRY :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_read_neighbor_table_entry:");
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
            rsi_zigb_get_child_short_address_for_the_index(api_test_var->child_index);
          }
          break;


        case ZIGBEETREEDEPTH :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_tree_depth:");
            if(resp->uCmdRspPayLoad.statusResp.status == 0)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_get_neighbor_table_entry_count();
          }
          break;

        case ZIGBEEGETNEIGHBORTABLEENTRYCOUNT :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_neighbor_table_entry_count:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

              rsi_zigb_zdp_send_device_announcement();
          }
          break;

        case ZDPSENDNWKADDRREQUEST :
          {
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
          }
          break;

        case ZDPSENDIEEEADDRREQUEST :
          {
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
          }
          break;

        case ZDPSENDDEVICEANNOUNCEMENT :
          { 
            RSI_DPRINT(RSI_PL1, "rsi_zigb_zdp_send_device_announcement:");
            if(resp->uCmdRspPayLoad.statusResp.status == 0xFF)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_send_unicast_data(ZigBee_Outgoing_Direct, api_test_var->DestAddress,&api_test_var->APSDEDataReq);
          }
          break;

        case ZDPSENDMATCHDESCRIPTORSREQUEST :
          {
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
            /* Response is checked at Data indication Callback */
          }
          break;

        case ZIGBEEACTIVEENDPOINTSREQUEST :
          {
            /* Response is checked at Data indication Callback */
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
          }
          break;

        case ZDPSENDPOWERDESCRIPTORREQUEST :
          {
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
            /* Response is checked at Data indication Callback */
          }
          break;

        case ZDPSENDNODEDESCRIPTORREQUEST :
          {
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
            /* Response is checked at Data indication Callback */
          }
          break;

        case ZIGBEESIMPLEDESCRIPTORREQUEST :
          {
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
          }
          break;

        case ZIGBEESENDUNICASTDATA :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_send_unicast_data:");
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
            rsi_zigb_send_broadcast_data(&api_test_var->APSDEDataReq);
          }
          break;

        case ZIGBEESENDBROADCASTDATA :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_send_broadcast_data:");
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
            rsi_zigb_send_group_data(0x0000,&api_test_var->APSDEDataReq);
          }
          break;

        case ZIGBEESENDGROUPDATA :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_send_group_data:");
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
            rsi_zigb_get_max_aps_payload_length();
          }
          break;


        case ZIGBEEGETMAXAPSPAYLOADLENGTH :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_max_aps_payload_length:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            rsi_zigb_get_key_table_entry(api_test_var->key_index, &api_test_var->KeyStruct);
          }
          break;

        case ZIGBEEGETKEYTABLEENTRY :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_key_table_entry:");
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
            rsi_zigb_get_key(g_Network_Key_c);
          }
          break;

        case ZIGBEEGETKEY :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_key:");
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
            rsi_zigb_have_link_key(api_test_var->get_ieee);
          }
          break;

        case ZIGBEEHAVELINKKEY :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_have_link_key:");
            if(resp->uCmdRspPayLoad.statusResp.status == TRUE)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_request_link_key(api_test_var->parent_ieee , api_test_var->parent_ieee);
          }
          break;

        case ZIGBEEREQUESTLINKKEY :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_request_link_key:");
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
            api_test_var->key_index = 0;
            rsi_zigb_set_key_table_entry(0, api_test_info->SelfAddress.IEEE_address, g_Network_Key_c, api_test_var->link_key_ptr);
          }
          break;

        case ZIGBEESETKEYTABLEENTRY:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_set_key_table_entry:");
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

            rsi_zigb_add_or_update_key_table_entry(api_test_info->SelfAddress.IEEE_address, 1, api_test_var->link_key_ptr, 0);
          }
          break;

        case ZIGBEEADDORUPDATEKEYTABLEENTRY:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_add_or_update_key_table_entry:");
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
            rsi_zigb_find_key_table_entry(api_test_var->parent_ieee, g_Network_Key_c);
          }
          break;

        case ZIGBEEFINDKEYTABLEENTRY:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_find_key_table_entry:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

              rsi_zigb_erase_key_table_entry(0x1);
          }
          break;

        case ZIGBEEERASEKEYTABLEENTRY:
          {
            rsi_zigb_mcpy( api_test_info->SelfAddress.IEEE_address, api_test_var->SetBindingEntry.a_src_addr,8);
            api_test_var->SetBindingEntry.src_endpoint = api_test_info->Source_EP;
            *(uint16_t*)api_test_var->SetBindingEntry.a_cluster_id = CUSTOM_CLUSTER;
            api_test_var->SetBindingEntry.dest_addr_mode = api_test_info->DestAddrMode;
            rsi_zigb_mcpy( api_test_info->ParentAddress.IEEE_address,api_test_var->SetBindingEntry.a_dest_addr, 8);
            api_test_var->SetBindingEntry.dest_endpoint = api_test_info->Dest_EP;

            RSI_DPRINT(RSI_PL1, "rsi_zigb_erase_key_table_entry:");
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
            rsi_zigb_set_binding_entry(&api_test_var->SetBindingEntry);
          }
          break;

        case ZIGBEESETBINDINGENTRY:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_set_binding_entry:");
            if(resp->uCmdRspPayLoad.statusResp.status == 0x00)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }
            rsi_zigb_delete_binding(0);
          }
          break;

        case ZIGBEEDELETEBINDING:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_delete_binding:");
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
            rsi_zigb_is_binding_entry_active(api_test_info->Dest_EP);
          }
          break;

        case ZIGBEEISBINDINGENTRYACTIVE:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_is_binding_entry_active:");
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
            rsi_zigb_clear_binding_table();
          }
          break;

        case ZIGBEECLEARBINDINGTABLE:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_clear_binding_table:");
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
          }
          break;

        case ZIGBEEGETROUTETABLEENTRY :
          {
            uint8_t ZigBee_Index_Out_Of_Range = 0x12; 
            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_route_table_entry:");
            if((resp->uCmdRspPayLoad.statusResp.status == ZigBee_Index_Out_Of_Range) ||
               (resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS))
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }

            rsi_zigb_read_neighbor_table_entry( 0x0);
          }
          break;


        case ZIGBEEGETCHILDSHORTADDRESSFORTHEINDEX :
          {

            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_child_short_address_for_the_index:");
            api_test_var->child_short_addr_t = resp->uCmdRspPayLoad.DevShortAddr.short_addr;
            if(api_test_var->child_short_addr_t != 0x0000)
            { 
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }

            rsi_zigb_get_child_index_for_specified_short_addr(api_test_var->child_short_addr_t);
          }
          break;

        case ZIGBEEGETCHILDINDEXFORSPECIFIEDSHORTADDR :
          {

            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_child_index_for_specified_short_addr:");

            if(resp->uCmdRspPayLoad.statusResp.status == api_test_var->child_index)
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }


            rsi_zigb_get_child_details(api_test_var->child_index);
          }
          break;


        case ZIGBEEGETCHILDDETAILS :
          {

            RSI_DPRINT(RSI_PL1, "rsi_zigb_get_child_details:");

            if(resp->uCmdRspPayLoad.GetChildDetails.status == g_SUCCESS_c)
            {
              rsi_zigb_mcpy( resp->uCmdRspPayLoad.GetChildDetails.Ieee_Addr, api_test_var->child_ieee , 8);
              api_test_var->child_dev_type = resp->uCmdRspPayLoad.GetChildDetails.device_type;
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            else
            {
              RSI_DPRINT(RSI_PL1," FAIL\n");
              TCFAIL
            }

            rsi_zigb_read_count_of_child_devices();
          }
          break;

        case ZIGBEEREADCOUNTOFCHILDDEVICES :
          {
            api_test_var->child_dev_count = resp->uCmdRspPayLoad.statusResp.status;
            RSI_DPRINT(RSI_PL1, "rsi_zigb_read_count_of_child_devices:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

              rsi_zigb_read_count_of_router_child_devices();
          }
          break;

        case ZIGBEEREADCOUNTOFROUTERCHILDDEVICES :
          {
            
            api_test_var->rout_child_dev_count = resp->uCmdRspPayLoad.statusResp.status;
            RSI_DPRINT(RSI_PL1, "rsi_zigb_read_count_of_router_child_devices:");
            RSI_DPRINT(RSI_PL1, " SUCCESS\n");
            TCSUCCESS

            rsi_zigb_zdp_send_nwk_addr_request(api_test_var->get_ieee, (BOOL)1, 0 );
          }
          break;


        case ZIGBEEBINDREQUEST:
          {
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
            /* Response is checked at Data indication Callback */
          }
          break;

        case ZIGBEEUNBINDREQUEST:
          {
            if(resp->uCmdRspPayLoad.statusResp.status == RSI_ZB_SUCCESS)
            {
              app_info->wait_for_rsp = 0;
            }
            /* Response is checked at Data indication Callback */
          }
          break;

        case ZIGBEELEAVENETWORK:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_leave_network:");
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

            rsi_zigb_delay(1);//if sleep is removed the leave network is not seen over the air
            rsi_zigb_set_operating_channel(12);
          }
          break;

        case ZIGBEESETOPERATINGCHANNEL :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_set_operating_channel:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c, 
                app_info->channel_mask ,
                app_info->scan_duration );
          }
          break;

        case ZIGBEEINITIATESCAN :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_initiate_scan:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            rsi_zigb_stop_scan();
          }
          break;

        case ZIGBEESTOPSCAN :
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_stop_scan:");
            {
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            return 0;
          }
          break;

        case ZIGBEENETWORKRESTORE:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_network_restore:");
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
            rsi_zigb_broadcast_nwk_manager_request(0x0000, app_info->channel_mask);
          }
          break;

        case ZIGBEEBROADCASTNWKMANAGERREQUEST:
          {
            RSI_DPRINT(RSI_PL1, "rsi_zigb_broadcast_nwk_manager_request:");
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
            rsi_zigb_leave_network();
          }
          break;

        case APPHANDLEDATACONFIRMATION:
          {
            if(1)
            {
              RSI_DPRINT(RSI_PL1, "rsi_zigb_data_confirmation:");
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            api_test_var->data_cnrfm_count++;
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
          break;

        case APPHANDLEDATAINDICATION:
          {
            if(!api_test_var->data_indication_count)
            {
              RSI_DPRINT(RSI_PL1, "rsi_zigb_data_indication:");
              RSI_DPRINT(RSI_PL1, " SUCCESS\n");
              TCSUCCESS
            }
            api_test_var->data_indication_count++;
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
          break;

        case APPSCANCOMPLETEHANDLER:
          {
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
          break;

        case APPENERGYSCANRESULTHANDLER:
          {
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
          break;

        case APPNETWORKFOUNDHANDLER:
          {
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
          break;

        case APPZIGBEESTACKSTATUSHANDLER:
          {
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
          break;

        case ZIGBEECHILDJOINHANDLER:
          {
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
            api_test_info->ParentAddress.short_address = app_cb_ptr->short_addr; 

            rsi_zigb_get_ieee_addr_for_specified_short_addr(app_cb_ptr->short_addr);
            
          }
          break;

        case APPINCOMINGMANYTOONEROUTEREQUESTHANDLER:
          {
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
          break;

        case APPADDRESSCONFLICTHANDLER:
          {
            rsi_zigb_app_cb_handler(api_test_var->cmd, (uint8_t *)&resp->uCmdRspPayLoad.uRspData);
          }
          break;

        default:
          {
            RSI_DPRINT(RSI_PL1,"Unknown state for the received resp. cmd_id: %x   \n",api_test_var->cmd);
          }
          break;
      }
    }

  }
  return RSI_ZB_FAIL;
}

void rsi_zigb_api_var_init()
{
  uint8_t i;
  uint8_t Link_Key[16] = {0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C, 0x6C, 0x69, 0x61,
                           0x6E, 0x63, 0x65, 0x30, 0x39};
  rsi_zigb_apitest_var_t  *api_test_var = &rsi_zigb_apitest_var;
  
  api_test_var->key_index = 0;
  api_test_var->channel = 0;
  api_test_var->data_indication_count = 0;
  api_test_var->data_cnrfm_count = 0;
  api_test_var->end_device_poll_count = 0;
  api_test_var->cmd = 0;
  api_test_var->child_index = 0;
  api_test_var->child_dev_type = 0;
  api_test_var->child_dev_count = 0;
  api_test_var->rout_child_dev_count = 0;
  api_test_var->aps_ack_required = 1;
  api_test_var->end_pont_id = 8;
  api_test_var->request_type = FALSE;
  api_test_var->start_index = 0;

  api_test_var->endpoint_cluster = 0;
  api_test_var->ManufacturerCode = 0x1234;
  api_test_var->short_addr_t = 0x0000;
  api_test_var->parent_short_addr = 0x0000 ;
  api_test_var->pan_Id = 0x0000;
  api_test_var->child_short_addr_t = 0x0000;
  api_test_var->broad_cast_addr = 0xFFFF;

  api_test_var->link_key_ptr = Link_Key;
  
  api_test_var->DestAddress.short_address = 0x0;
  api_test_var->APSDEDataReq.DestEndpoint = 8;
  api_test_var->APSDEDataReq.SrcEndpoint = 1;
  api_test_var->APSDEDataReq.ProfileId = 10;
  api_test_var->APSDEDataReq.ClusterId = 10;
  api_test_var->APSDEDataReq.AsduLength = 50;
  api_test_var->APSDEDataReq.TxOptions = g_APS_Tx_Opt_Ack_Req_c | g_APS_Tx_Opt_Use_NWK_Key_c;
  api_test_var->APSDEDataReq.Radius = DEAFULT_RADIUS;
  
  for(i = 0; i < 0x33 ; i++)
            api_test_var->APSDEDataReq.aPayload[i] = i;
  api_test_var->nodePowerDesc_t.current_powermode_avail_power_sources = 0x1;
  api_test_var->nodePowerDesc_t.current_powersource_currentpowersourcelevel = 0x2;

}

#endif
