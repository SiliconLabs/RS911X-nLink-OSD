/**
 *     @file     rsi_zigb_app_frame_process.c
 *     @version  1.0
 *     @date     2014-Aug-23
 *
 *     Copyright(C) Redpine Signals 2014
 *     All rights reserved by Redpine Signals.
 *
 *     @section License
 *     This program should be used on your own responsibility.
 *     Redpine Signals assumes no responsibility for any losses
 *     incurred by customers or third parties arising from the use of this file.
 *
 *     @brief API: Definitions of various data structures and variables
 *
 *     @section Description
 *     This file is used to process the received frame from device
 *
 *     @section Improvements
 *     New command frames are added.
 *
 */


/**
 * Includes
 * */
 
#include "rsi_zigb_types.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_app.h"
#include "rsi_zigb_global.h"

/*===========================================================================
 *
 * @fn          rsi_zigb_uCmdRsp *rsi_zigb_app_frame_process(uint8_t *buffer)
 * @brief       Processes the received ZigBee pkt
 * @param[in]   buffer- buffer pointer to fill the descriptor
 * @param[out]  none
 * @return      Buffer pointer after parsing descriptor
 * @section description
 * This API is used to parse ZigBee specific descriptor and return paylod with
 * cmd_id and intf_id of the pkt
 *
 * ===========================================================================*/
  
rsi_zigb_uCmdRsp *rsi_zigb_app_frame_process(uint8_t *buffer)
{
  uint16_t payload_length = 0;
  uint16_t seq_no = 0;
  uint8_t *buf_ptr, cmd_id = 0, intf_id = 0;
#ifdef ENABLE_DEBUG_PRINTS 
  uint8_t i;
#endif  
  rsi_zigb_uCmdRsp *temp_ptr;

  /* Point the buffer pointer to interface and cmd id
   * We can just verify seq num, direction */
  temp_ptr = (rsi_zigb_uCmdRsp *)(buffer + RSI_ZIGB_SEQ_NUM_OFFSET);
#ifdef ENABLE_DEBUG_PRINTS 
  RSI_DPRINT(RSI_PL1,"RCVD : pkt dump\n");
  for(i = 0; i < (16+buffer[0]); i++)
  {
    RSI_DPRINT(RSI_PL1,"%x ", buffer[i]);
    if ((i % 15 == 0) && (i != 0)) {
      RSI_DPRINT(RSI_PL1,"\n");
    }
  }
#endif  
  /* Step-1:- Get the payload length */
	payload_length = ((uint16_t )buffer[0] & 0xfff);

  if (payload_length >= RSI_ZIGB_MAX_PAYLOAD_SIZE) {
#ifdef ENABLE_DEBUG_PRINTS 
    RSI_DPRINT(RSI_PL1,"Error: RSI_ZB_PAYLOAD_SIZE_ERROR \n");
#endif    
    return NULL; 
  }
#ifdef ZB_ONLY
  /* Check Queue no and differentiate protocol */
  if ((((buffer[1] & 0xF0)) == 0x40) && (buffer[2] == 0x10)) {
    /* Incrementing this buffer to get the Sequence number
    * starting address */
    temp_ptr->intf_id = 7;
    temp_ptr->cmd_id = ZIGBEE_OPER_MODE_RSP;
    return temp_ptr;
  }
#endif  

  /* Incrementing this buffer to get the Sequence number
   * starting address */
  buf_ptr = (buffer + RSI_ZIGB_SEQ_NUM_OFFSET);

  /* Step-2:- Sequence number Verification 
   * Currently we are just taking it as token number
   * only  for debugging*/
	seq_no = *buf_ptr++; 

  if (seq_no >= RSI_ZIGB_MAX_PAYLOAD_SIZE) {
#ifdef ENABLE_DEBUG_PRINTS 
    RSI_DPRINT(RSI_PL1,"Error: RSI_ZB_ERROR_SEQUENCE_NUMBER \n");
#endif    
    return NULL; 
  }
	/* Step-3:- Direction check */
  if(*buf_ptr++ != DIR_DEVICE_TO_HOST) {
#ifdef ENABLE_DEBUG_PRINTS 
    RSI_DPRINT(RSI_PL1,"Invalid pkt received\n");
    RSI_DPRINT(RSI_PL1,"Error: RSI_ZB_DIRECTION_ERROR \n");
  for(i = 0; i < (16+buffer[0]); i++)
  {
    buffer += RSI_ZIGB_SEQ_NUM_OFFSET;
    RSI_DPRINT(RSI_PL1,"%x ", buffer[i]);
    if ((i % 15 == 0) && (i != 0)) {
      RSI_DPRINT(RSI_PL1,"\n");
    }
  }
#endif    
    return NULL; 
  }

  /* Step-4:- Get the Interface Id */
  intf_id = *buf_ptr++;

  if (intf_id > INTERNAL_MANAGEMENT_INTERFACE) {
#ifdef ENABLE_DEBUG_PRINTS 
    RSI_DPRINT(RSI_PL1,"Error: RSI_ZB_INTERFACE_ERROR \n");
#endif    
    return NULL; 
  }

  /* Step-5:- Get the Command Id */
  cmd_id = *buf_ptr++;

  if (!cmd_id) {
#ifdef ENABLE_DEBUG_PRINTS 
    RSI_DPRINT(RSI_PL1,"Error: RSI_ZB_COMMAND_ERROR \n");
#endif    
    return NULL; 
  }

  /* Return buffer only from Dir present in desc 
   * rest of desc is discarded*/
  return temp_ptr;
}

