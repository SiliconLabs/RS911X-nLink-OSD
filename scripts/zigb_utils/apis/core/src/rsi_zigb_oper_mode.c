/**
 * @file       rsi_oper_mode.c
 * @version    2.7
 * @date       2012-Sep-26
 *
 * Copyright(C) Redpine Signals 2012
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief OPERATING MODE: implements the OPERATING MODE command
 *
 * @section Description
 * This file contains the rsi_opermode function.
 *
 *
 */


/**
 * Includes
 */
#include "rsi_zigb_global.h"
#include "rsi_zigb_oper_mode.h"


/**
 * Global Variables
 */


/*=================================================*/
/**
 *@fn            int16 rsi_zigb_oper_mode(rsi_uOperMode *uOperMode)
 * @brief        Sends the OPERATING MODE command to the Wi-Fi module via SPI
 * @param[in]    uint8 mode value to configure 0 for legacy client mode , 
  *              1 for wifi-direct mode , 2 for enterprise security mode.
 * @param[out]   none
 * @return       errCode
 *               -2 = Command execution failure
 *               -1 = Buffer Full
 *               0  = SUCCESS
 * @section description 
 * This API is used to select the Legacy client mode or P2P mode or Enterprise Security Mode.
 */

int16_t rsi_zigb_oper_mode(rsi_zigb_uOperMode *uOperMode)
{
  int16_t    retval;
  uint8_t    rsi_zigb_frameCmdOperMode[16] = {0x10, 0x40, 0x10, 0x0};

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL3,"\r\n\nOperating Mode");
#endif

  //! Write descriptor and payload
  retval = rsi_frame_write((rsi_zigb_uFrameDsc *)rsi_zigb_frameCmdOperMode,(uint8_t *)uOperMode, sizeof(rsi_zigb_uOperMode));
  return retval;
}

