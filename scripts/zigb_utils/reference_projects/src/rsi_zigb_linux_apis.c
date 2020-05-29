/**
 * @file      rsi_linux_apis.c
 * @version   1.2
 * @date      2014-Sep-12
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief Implements common functionality for all the commands
 *
 * @section Description
 * This file contains common api needed for all the commands
 *
 *
 */

/**
 * Includes
 */
#include "rsi_zigb_global.h"
#include "rsi_nl_app.h"
#include "platform_specific.h"

/*====================================================*/
/**
 * @fn          int16_t rsi_execute_cmd(uint8_t *descparam,uint8_t *payloadparam,uint16_t size_param)
 * @brief       Common function for all the commands.
 * @param[in]   uint8_t *descparam, pointer to the frame descriptor parameter structure
 * @param[in]   uint8_t *payloadparam, pointer to the command payload parameter structure
 * @param[in]   uint16_t size_param, size of the payload for the command
 * @return      errCode
 *              -2 = Command issue failed
 *              0  = SUCCESS
 * @section description
 * This is a common function used to process a command to the Wi-Fi module.
 */

int16_t rsi_zigb_execute_cmd(uint8_t *descparam, uint8_t *payloadparam, uint16_t size_param)
{
  int16_t                                         retval = 0;
  rsi_zigb_uFrameDsc         uFrameDscFrame;
  uint8_t           *cmd_buff;
#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_zigb_execute_cmd:\n");
#endif

  //! Build 16 bytes, send/receive command descriptor frame
  rsi_zigb_build_frame_descriptor(&uFrameDscFrame, descparam, size_param);

  cmd_buff = rsi_alloc_and_init_cmdbuff((uint8_t *)&uFrameDscFrame,payloadparam,size_param);

  if(rsi_send_usr_cmd(cmd_buff, GET_SEND_LENGTH(cmd_buff)) < 0)
  {
    retval = -2;
  }
  if(retval < 0)
  {
    printf("Unable to issue command\n");
  }

  //! Free the command buffer
  rsi_free(cmd_buff);

  return retval;
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_delay(uint32_t time)
 * @brief       Sleep
 * @param[in]   Time to sleep
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to sleep for a specified 
 * amount of duration
 *
 * ===========================================================================*/
  
void rsi_zigb_delay(uint32_t time)
{
  sleep(time);
}
