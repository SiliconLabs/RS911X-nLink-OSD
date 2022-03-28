/*******************************************************************************
* @file  rsi_91x_nlsock.c
* @brief This file contains registration of nl sockets and respective callbacks
* to parse the cmd from the application.
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

#include <linux/module.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include "rsi_main.h"

struct rsi_hw *adapter_g;

static void rsi_nl_recv_msg(struct sk_buff *skb)
{
  struct nlmsghdr *nlh;
  struct rsi_hw *adapter = adapter_g;
  int pid, payload_len, pkt_type, cmd;
#if defined(CONFIG_RSI_COEX_MODE) || defined(CONFIG_RSI_BT_ALONE)
  int status;
#endif
  struct rsi_nl_desc *nl_desc = NULL;
  nlh                         = (struct nlmsghdr *)skb->data;
  nl_desc                     = (struct rsi_nl_desc *)nlmsg_data(nlh);

  nlh      = (struct nlmsghdr *)skb->data;
  pid      = nlh->nlmsg_pid; /*pid of sending process */
  pkt_type = nlh->nlmsg_type;
  cmd      = nl_desc->desc_word[0];

  if (pkt_type == WLAN_PACKET) {
    switch (cmd) {
      case PER_RECV_STOP:
        rsi_dbg(INFO_ZONE, "%s: stop PER receive case\n", __func__);
        adapter->recv_stop       = 1;
        adapter->rx_stats_inprog = 0;
        /* GCC ver 9.3.0 expects fall through comment if we want to combine two switch cases */
        /* fall-through */
      case PER_RECEIVE:
        adapter->wlan_nl_pid = pid;
        if (adapter->priv->fsm_state == FSM_MAC_INIT_DONE) {
          adapter->per_params.rate_flags = nl_desc->desc_word[1];
          adapter->per_params.channel    = nl_desc->desc_word[2];
          adapter->per_stop_bit          = nl_desc->desc_word[3];
          rsi_dbg(MGMT_DEBUG_ZONE, "%s: PER receive case\n", __func__);
          rsi_send_rx_stats_cmd(adapter, nlh);
        } else {
          rsi_dbg(INFO_ZONE, "%s: uninitialized fsm state\n", __func__);
          return;
        }
        break;
      case UPDATE_WLAN_GAIN_TABLE:
        if (adapter->priv->fsm_state == FSM_MAC_INIT_DONE) {
          payload_len = nl_desc->desc_word[1];
          rsi_update_wlan_gain_table(adapter, nlh, payload_len);
        } else {
          rsi_dbg(INFO_ZONE, "%s: uninitialized fsm state\n", __func__);
          return;
        }
        break;
      case PER_TRANSMIT:
        if (adapter->priv->fsm_state == FSM_MAC_INIT_DONE) {
          rsi_dbg(MGMT_DEBUG_ZONE, "%s: PER TRANSMIT case\n", __func__);
          payload_len = sizeof(adapter->per_params);
          rsi_transmit_stats_cmd(adapter, nlh, payload_len);
        } else {
          rsi_dbg(ERR_ZONE, "Driver not installed before issuing transmit command\n");
        }
        break;
      case PER_PACKET:
        rsi_dbg(MGMT_DEBUG_ZONE, "%s: PER PACKET case\n", __func__);
        payload_len = sizeof(adapter->per_packet);
        memcpy((&adapter->per_packet), nlmsg_data(nlh) + FRAME_DESC_SZ, payload_len);
        break;
      case RSI_SET_BB_RF:
        adapter->wlan_nl_pid = pid;
        rsi_dbg(MGMT_DEBUG_ZONE, "%s: RSI_SET_BB_RF case\n", __func__);
        payload_len = sizeof(adapter->bb_rf_params);
        memcpy((&adapter->bb_rf_params), nlmsg_data(nlh) + FRAME_DESC_SZ, payload_len);
        if ((rsi_set_bb_rf_values(adapter)) < 0) {
          rsi_dbg(ERR_ZONE, "Invalid Arguments by user\n");
        }
        break;
      case RSI_EFUSE_MAP:
        adapter->wlan_nl_pid = pid;
        rsi_dbg(MGMT_DEBUG_ZONE, "%s: EFUSE MAP case\n", __func__);
        rsi_copy_efuse_content(adapter);
        break;
      default:
        rsi_dbg(ERR_ZONE, "Invalid Message Type\n");
    }
  } else if (pkt_type == BT_PACKET) {
    switch (cmd) {
#if defined(CONFIG_RSI_COEX_MODE) || defined(CONFIG_RSI_BT_ALONE)
      case BT_E2E_STAT:
        rsi_dbg(INFO_ZONE, "BT E2E STAT from App\n");
        adapter->bt_nl_pid = pid;
        payload_len        = nl_desc->desc_word[1];
        rsi_hex_dump(DATA_TX_ZONE, "TX BT pkt", skb->data, skb->len);
        status = rsi_bt_e2e_stats(adapter, nlh, payload_len, cmd);
        if (status < 0)
          rsi_dbg(ERR_ZONE, " Failed in BT E2E STAT\n");
        return;
        break;
      case BT_BLE_GAIN_TABLE:
        rsi_dbg(INFO_ZONE, "BT GAIN TABLE UPDATE from App\n");
        adapter->bt_nl_pid = pid;
        payload_len        = nl_desc->desc_word[1];
        rsi_hex_dump(DATA_TX_ZONE, "TX BT pkt", skb->data, skb->len);
        status = rsi_bt_ble_update_gain_table(adapter, nlh, payload_len, cmd);
        if (status < 0)
          rsi_dbg(ERR_ZONE, " Failed in BT/BLE GAIN TABLE UPDATE\n");
        break;
#endif
      default:
        rsi_dbg(ERR_ZONE, "Invalid Message Type\n");
        return;
    }
  }
}

int nl_sk_init(struct rsi_hw *adapter)
{
  struct sock *nl_sk;
  struct netlink_kernel_cfg cfg = {
    .input = rsi_nl_recv_msg,
  };

  rsi_dbg(INFO_ZONE, "%s\n", __func__);
  adapter_g = adapter;
  nl_sk     = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
  if (!nl_sk) {
    printk(KERN_ALERT "Error creating socket.\n");
    return -10;
  }
  adapter->nl_sk = nl_sk;
  return 0;
}

void nl_sk_exit(struct sock *nl_sk)
{
  rsi_dbg(INFO_ZONE, "%s\n", __func__);
  netlink_kernel_release(nl_sk);
}
