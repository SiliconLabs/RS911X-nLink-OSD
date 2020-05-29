/**
 *  @file     rsi_zigb_execute_cmd.c
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
 *
 *  @section Description
 *  This file contains definition to initiate zigbee write command
 *
 *  @section Improvements
 *
 */


/**
 * Includes
 * */
 
#include "rsi_zigb_types.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_global.h"
#include "rsi_zigb_app.h"

/*===========================================================================
 *
 * @fn          int16_t rsi_zigb_execute_cmd(uint8_t *desc, uint8_t *payload, 
 *                                           uint16_t length)
 * @brief       ZigBee Execute command 
 * @param[in]   desc    - descriptor pointer
 * @param[in]   payload - payload pointer
 * @param[in]   length
 * @param[out]  none
 * @return      Status
 * @section description
 * This API is used to execute ZigBee pkt and this should be written to device 
 *
 * ===========================================================================*/
  
int16_t rsi_zigb_execute_cmd(uint8_t *desc, uint8_t *payload, uint16_t length)
{
  int16_t ret_val;

  rsi_zigb_uFrameDsc uFrameDscFrame;

#ifdef ZB_DEBUG
  RSI_DPRINT(RSI_PL1,"Descriptor Write\n");
  RSI_DPRINT(RSI_PL1,"TX Pkt: Len of the packet: %d\n", length);
#endif

  //! 16 bytes, send/receive command descriptor frame
  //! Create the Command Frame Descriptor  
  rsi_zigb_build_frame_descriptor(&uFrameDscFrame, desc, length);

  //! Write descriptor and payload
  ret_val = rsi_frame_write(&uFrameDscFrame, payload, length);

  if (ret_val != 0x00) 
  {
#ifdef ZB_DEBUG
    RSI_DPRINT(RSI_PL1,"Frame write failErr=%02x", (UINT16)ret_val);
#endif
  }
  return ret_val;
}
