/**
 * @file     rsi_zigb_build_frame_descriptor.c
 * @version  1.0
 * @date     2014-Aug-23
 *
 * Copyright(C) Redpine Signals 2014
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief Function which builds the frame descriptor of BT Packet.
 *
 * @section Description
 * This file contains a function to build the frame descriptor of BT Packet.
 *
 *
 */


/**
 * Includes
 */
#include "rsi_zigb_global.h"


/**
 * Global Variables
 */

/*==================================================*/
/**
 * @fn          void rsi_buildFrameDescriptor(rsi_zigb_uFrameDsc *uFrameDscFrame, uint8 *cmd)
 * @brief       Creates a Frame Descriptor
 * @param[in]   rsi_zigb_uFrameDsc *uFrameDscFrame,Frame Descriptor
 * @param[in]   uint8 *cmd,Indicates type of the packet(data or management)
 * @param[out]  none
 * @return      none
 */
void rsi_zigb_build_frame_descriptor(rsi_zigb_uFrameDsc *uFrameDscFrame, uint8_t *cmd, uint8_t size_param)
{
  uint8_t i;
  for (i = 0; i < RSI_ZIGB_FRAME_DESC_LEN; i++)
  {
    uFrameDscFrame->uFrmDscBuf[i] = 0;
  }

  /* Update the ZigBee Descriptor */

  //! Length of the frame 
  uFrameDscFrame->uFrmDscBuf[0]  = size_param;
  //! ZigBee Host Queue Type
  uFrameDscFrame->uFrmDscBuf[1]  = 0x10; 
  //! Direction
  uFrameDscFrame->uFrmDscBuf[13] = 1;
  //! Interface Type
  uFrameDscFrame->uFrmDscBuf[14] = cmd[0];
  //! Command Type
  uFrameDscFrame->uFrmDscBuf[15] = cmd[1];
  return;
}





