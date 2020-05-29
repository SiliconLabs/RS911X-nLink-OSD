/**
 *  @file     rsi_zigb_init_oper_mode.c
 *  @version  1.0
 *  @date     2014-Sep-24
 *
 *  Copyright(C) Redpine Signals 2014
 *  All rights reserved by Redpine Signals.
 *
 *  @section License
 *  This program should be used on your own responsibility.
 *  Redpine Signals assumes no responsibility for any losses
 *  incurred by customers or third parties arising from the use of this file.
 *
 *  @brief API: Opermode init handling in case of ZigBee alone mode
 *
 *  @section Description
 *  This file contain handling of card ready and opermode commnand initialization
 *
 *  @section Improvements
 *  New command frames are added.
 *
 */


/**
 * Includes
 * */
#include "rsi_hal.h"
#include "rsi_zigb_app.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_oper_mode.h"

extern rsi_zigb_app_cb_t   rsi_zigb_app_cb;

/*=================================================*/
/**
 * @fn          rsi_uCmdRsp *rsi_parse_response(uint8_t *rsp)
 * @brief       To parse the resposne received from Kernel
 * @param[in]   uint8_t *rsp, response buffer pointer
 * @param[out]  none
 * @return      rsi_uCmdRsp *ptr, response pointer
 * @section description 
 * This API is used to parse the recieved response. This returns the 
 * pointer which points to rsptype, status, response payload in order.
 */
rsi_init_uCmdRsp *rsi_parse_response(uint8_t *rsp)
{
  rsi_init_uCmdRsp             *temp_uCmdRspPtr = NULL;
  uint8_t                   temp_rspCode;
  uint16_t                  temp_status;
  uint8_t                   *descPtr = rsp ;
  uint8_t                   *payloadPtr = rsp + RSI_FRAME_DESC_LEN;
#ifdef RSI_DEBUG_PRINT
  uint8_t i;
#endif

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL13,"Recieved Packet PRINT3 \n");
#endif
  /* Check whether it is any rxpkt or just a status indication */
#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL13,"Recieved Packet PRINT4 \n");
#endif


  /* In Response received First 24 bytes are header.
   * And then Desc and Payload of the response is present.
   * 2nd byte of the Desc is status and 14th byte of the Desc is RspType.
   */

  /* Retrieve response code from the received packet */
#ifdef RSI_LITTLE_ENDIAN
  temp_status = (*(uint16_t *)(descPtr + RSI_STATUS_OFFSET));
  temp_rspCode = *((uint8_t *)(descPtr + RSI_RSP_TYPE_OFFSET)); 
#else
  temp_status = (uint8_t)rsi_bytes2R_to_uint16(descPtr + RSI_STATUS_OFFSET);
  temp_rspCode = rsi_bytes2R_to_uint16(descPtr + RSI_RSP_TYPE_OFFSET);
#endif

#ifdef RSI_DEBUG_PRINT
  for(i=0;i<16;i++)
  {
    RSI_DPRINT(RSI_PL13,"received rspcode: 0x%x \n",descPtr[i]);

  }
#endif    

  if(temp_rspCode)
  {
#ifdef RSI_DEBUG_PRINT
    RSI_DPRINT(RSI_PL13,"received status : 0x%x \n",temp_status);
    RSI_DPRINT(RSI_PL13,"received rspcode: 0x%x \n",temp_rspCode);
#endif    
  }
  /* Copy the response type and status to payloadPtr-4, payloadPtr-2
   * locations respectively.
   */
#ifdef RSI_LITTLE_ENDIAN
  *((uint16_t *)(payloadPtr - 2)) = temp_status;
  *((uint16_t *)(payloadPtr - 4)) = temp_rspCode;
#else
  rsi_uint16_to_2bytes((payloadPtr - 2), temp_status);
  rsi_uint16_to_2bytes((payloadPtr - 4), temp_rspCode);
#endif

  temp_uCmdRspPtr = (rsi_init_uCmdRsp *)(payloadPtr - 4);

  return temp_uCmdRspPtr;
}

/*===========================================================================
 *
 * @fn          int zigb_init_oper_mode()
 * @brief       Handle Card Reay from module and send opermode command to module
 * @param[in]   none
 * @param[out]  none
 * @return      Status
 * 
 * @section description
 * This is the function which handles card ready response from ZigBee module 
 * and initiates Opermode command to initiate ZigBee
 *
 * ===========================================================================*/

int16_t rsi_zigb_init_oper_mode()
{
  rsi_zigb_app_cb_t *app_cb_ptr = &rsi_zigb_app_cb;
  rsi_init_uCmdRsp *uCmdRspFrame;
  rsi_zigb_uOperMode opermode;
  rsi_zigb_memset((uint8_t *)&opermode, 0, sizeof(rsi_zigb_uOperMode));
  int16_t retval;
  uint16_t response_type = 0, error_code;

  /* operating mode parameters */
#ifdef RSI_LITTLE_ENDIAN
  opermode.operModeFrameSnd.oper_mode = (uint32_t)RSI_ZIGB_OPERMODE;
  opermode.operModeFrameSnd.feature_bit_map = (uint32_t)RSI_ZIGB_FEATURE_BIT_MAP;
  opermode.operModeFrameSnd.tcp_ip_feature_bit_map = (uint32_t)RSI_ZIGB_TCP_IP_FEATURE_BIT_MAP;
  opermode.operModeFrameSnd.custom_feature_bit_map = (uint32_t)RSI_ZIGB_CUSTOM_FEATURE_BIT_MAP;
#else
  rsi_uint32_to_4bytes(opermode.operModeFrameSnd.oper_mode, RSI_ZIGB_OPERMODE);
  rsi_uint32_to_4bytes(opermode.operModeFrameSnd.feature_bit_map, RSI_ZIGB_FEATURE_BIT_MAP);
  rsi_uint32_to_4bytes(opermode.operModeFrameSnd.tcp_ip_feature_bit_map, RSI_ZIGB_TCP_IP_FEATURE_BIT_MAP);
  rsi_uint32_to_4bytes(opermode.operModeFrameSnd.custom_feature_bit_map, RSI_ZIGB_CUSTOM_FEATURE_BIT_MAP);
#endif

  while (1) 
  {
    if (app_cb_ptr->pkt_pending == RSI_TRUE)
    {
      retval = rsi_frame_read(app_cb_ptr->read_packet_buffer);
      uCmdRspFrame = rsi_parse_response(app_cb_ptr->read_packet_buffer);

      if(retval == 0)
      {
#ifdef RSI_LITTLE_ENDIAN
        /* Retrieve response code from the received packet */ 
        response_type = *(uint16_t *)(uCmdRspFrame->rspCode);    
        error_code = *(uint16_t *)uCmdRspFrame->status;
#else
        /* Retrieve response code from the received packet */
        response_type = rsi_bytes2R_to_uint16(&uCmdRspFrame->rspCode);
        error_code = rsi_bytes2R_to_uint16(&uCmdRspFrame->status);
#endif                    

        /* Switch between type of packet received */
        switch (response_type)
        {
          case RSI_RSP_CARD_READY:                                    
          {
            if(!error_code)
            {
              retval = rsi_zigb_oper_mode(&opermode);
              return 0;
            }                         
          }
          break;

        }
      }
    }
  }
}
