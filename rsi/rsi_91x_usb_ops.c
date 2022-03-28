/*******************************************************************************
* @file  rsi_91x_usb_ops.c
* @brief 
*******************************************************************************
* # License
* <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* The licensor of this software is Silicon Laboratories Inc. Your use of this
* software is governed by the terms of Silicon Labs Master Software License
* Agreement (MSLA) available at
* www.silabs.com/about-us/legal/master-software-license-agreement. This
* software is distributed to you in Source Code format and is governed by the
* sections of the MSLA applicable to Source Code.
*
******************************************************************************/

#include <linux/firmware.h>
#include "rsi_usb.h"

/**
 * rsi_usb_rx_thread() - This is a kernel thread to receive the packets from
 *			 the USB device.
 * @common: Pointer to the driver private structure.
 *
 * Return: None.
 */
void rsi_usb_rx_thread(struct rsi_common *common)
{
  struct rsi_hw *adapter     = common->priv;
  struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;
  struct sk_buff *skb;
  int status, idx;

  do {
    status = rsi_wait_event(&dev->rx_thread.event, EVENT_WAIT_FOREVER);
    if (status < 0)
      break;
    rsi_reset_event(&dev->rx_thread.event);

    if (atomic_read(&dev->rx_thread.thread_done))
      break;

    for (idx = 0; idx < MAX_RX_URBS; idx++) {
      while (true) {
        skb = skb_dequeue(&dev->rx_q[idx]);
        if (!skb)
          break;

        status = rsi_read_pkt(common, skb->data, 0);
        if (status) {
          rsi_dbg(ERR_ZONE, "%s: Failed To read data", __func__);
        }
        dev_kfree_skb(skb);
      }
    }
  } while (1);

  rsi_dbg(INFO_ZONE, "%s: Terminated USB RX thread\n", __func__);
  atomic_inc(&dev->rx_thread.thread_done);
  complete_and_exit(&dev->rx_thread.completion, 0);
}
