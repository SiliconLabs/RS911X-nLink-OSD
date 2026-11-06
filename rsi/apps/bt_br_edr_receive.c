// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020-2023 Silicon Labs, Inc.
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include "bt_util.h"
#include "per_util.h"

#define NO_OF_ARGS   15
#define STOP_OF_ARGS 2

int main(int argc, char **argv)
{
  int rc = 0, sfd, len, status = 0;
  unsigned int num_1;
  bt_br_edr_per_recv_params_t bt_br_edr_per_recv_params;
  struct bb_rf_param_bt_t bb_rf_params;
  struct nlmsghdr *nlh;
  unsigned char macaddr[6];

  memset(&bt_br_edr_per_recv_params, 0, sizeof(bt_br_edr_per_recv_params_t));
  memset(&bb_rf_params, 0, sizeof(bb_rf_params));
  printf("argc = %d\n", argc);

  sfd = socket_creation();
  if (sfd < 0) {
    ONEBOX_PRINT("Socket Creation Failed\n");
    return ONEBOX_STATUS_FAILURE;
  }

  if (argc == NO_OF_ARGS) {
    bt_br_edr_per_recv_params.enable = 1;
    memset(macaddr, 0, 6);
    byteconversion(argv[1], macaddr);

    for (num_1 = 0; num_1 < 6; num_1++) {
      bt_br_edr_per_recv_params.bt_addr[5 - num_1] = macaddr[num_1];
    }
    bt_br_edr_per_recv_params.link_type         = atoi(argv[2]);
    bt_br_edr_per_recv_params.pkt_type          = atoi(argv[3]);
    bt_br_edr_per_recv_params.pkt_length        = atoi(argv[4]);
    bt_br_edr_per_recv_params.scrambler_seed    = atoi(argv[5]);
    bt_br_edr_per_recv_params.br_edr_indication = atoi(argv[6]);
    bt_br_edr_per_recv_params.bt_rx_rf_chnl     = atoi(argv[7]);
    bt_br_edr_per_recv_params.bt_tx_rf_chnl     = atoi(argv[8]);
    bt_br_edr_per_recv_params.enable_hopping    = atoi(argv[9]);
    bt_br_edr_per_recv_params.ant_sel           = atoi(argv[10]);
    bt_br_edr_per_recv_params.loop_back_mode    = atoi(argv[11]);
    bt_br_edr_per_recv_params.pll_mode          = atoi(argv[12]);
    bt_br_edr_per_recv_params.rf_type           = atoi(argv[13]);
    bt_br_edr_per_recv_params.rf_chain          = atoi(argv[14]);

    printf("the packet type is %d \n", bt_br_edr_per_recv_params.pkt_type);
    printf("the packet length is %d \n", bt_br_edr_per_recv_params.pkt_length);

    len                = sizeof(int);
    bb_rf_params.value = GET_DRV_COEX_MODE;
    if (bt_per_cmd_wrapper(bb_rf_params, BT_PER, sfd, BT_PACKET) < 0) {
      printf("Unable to perform bt_receive\n");
      return ONEBOX_STATUS_FAILURE;
    }
    nlh = common_recv_mesg_wrapper(sfd, len);
    if (nlh == NULL) {
      printf("Error receving from bb\n");
      return ONEBOX_STATUS_FAILURE;
    }
    memcpy(&rc, NLMSG_DATA(nlh), len);
    free(nlh);

    if ((rc == BT_CLASSIC_MODE) || (rc == BT_DUAL_MODE)) {
      if (bt_br_edr_per_recv_params.bt_rx_rf_chnl < BT_CLASSIC_START_CHANNEL
          || bt_br_edr_per_recv_params.bt_rx_rf_chnl > BT_CLASSIC_END_CHANNEL) {
        printf("Invalid rx_channel in CLASSIC oper mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (bt_br_edr_per_recv_params.bt_tx_rf_chnl < BT_CLASSIC_START_CHANNEL
          || bt_br_edr_per_recv_params.bt_tx_rf_chnl > BT_CLASSIC_END_CHANNEL) {
        printf("Invalid tx_channel in CLASSIC oper mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if ((bt_br_edr_per_recv_params.enable_hopping == FREQ_FIXED_HOP)
          && (bt_br_edr_per_recv_params.bt_tx_rf_chnl == bt_br_edr_per_recv_params.bt_rx_rf_chnl)) {
        printf("Tx and Rx channels should be different in the fixed hopping\n");
        return ONEBOX_STATUS_FAILURE;
      }

      if (bt_br_edr_per_recv_params.link_type == SCO_LINK && bt_br_edr_per_recv_params.br_edr_indication == ONE_MBPS) {
        if (bt_br_edr_per_recv_params.pkt_type == BT_HV1_PKT_TYPE
            && bt_br_edr_per_recv_params.pkt_length != BT_HV1_VOICE_PAYLOAD_LEN) {
          printf("Invalid packet length\n");
          return ONEBOX_STATUS_FAILURE;
        }
        if (bt_br_edr_per_recv_params.pkt_type == BT_HV2_PKT_TYPE
            && bt_br_edr_per_recv_params.pkt_length != BT_HV2_VOICE_PAYLOAD_LEN) {
          printf("Invalid packet length\n");
          return ONEBOX_STATUS_FAILURE;
        }
        if (bt_br_edr_per_recv_params.pkt_type == BT_HV3_PKT_TYPE
            && bt_br_edr_per_recv_params.pkt_length != BT_HV3_VOICE_PAYLOAD_LEN) {
          printf("Invalid packet length\n");
          return ONEBOX_STATUS_FAILURE;
        }
      }

      if (bt_br_edr_per_recv_params.link_type == ESCO_LINK) {
        if (bt_br_edr_per_recv_params.br_edr_indication == ONE_MBPS) {
          if (bt_br_edr_per_recv_params.pkt_type == BT_EV3_PKT_TYPE
              && !(bt_br_edr_per_recv_params.pkt_length > 0
                   && bt_br_edr_per_recv_params.pkt_length <= BT_EV3_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_recv_params.pkt_type == BT_EV4_PKT_TYPE
              && !(bt_br_edr_per_recv_params.pkt_length > 0
                   && bt_br_edr_per_recv_params.pkt_length <= BT_EV4_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_recv_params.pkt_type == BT_EV5_PKT_TYPE
              && !(bt_br_edr_per_recv_params.pkt_length > 0
                   && bt_br_edr_per_recv_params.pkt_length <= BT_EV5_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
        if (bt_br_edr_per_recv_params.br_edr_indication == TWO_MBPS) {
          if (bt_br_edr_per_recv_params.pkt_type == BT_2EV3_PKT_TYPE
              && !(bt_br_edr_per_recv_params.pkt_length > 0
                   && bt_br_edr_per_recv_params.pkt_length <= BT_2EV3_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_recv_params.pkt_type == BT_2EV5_PKT_TYPE
              && !(bt_br_edr_per_recv_params.pkt_length > 0
                   && bt_br_edr_per_recv_params.pkt_length <= BT_2EV5_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
        if (bt_br_edr_per_recv_params.br_edr_indication == THREE_MBPS) {
          if (bt_br_edr_per_recv_params.pkt_type == BT_3EV3_PKT_TYPE
              && !(bt_br_edr_per_recv_params.pkt_length > 0
                   && bt_br_edr_per_recv_params.pkt_length <= BT_3EV3_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_recv_params.pkt_type == BT_3EV5_PKT_TYPE
              && !(bt_br_edr_per_recv_params.pkt_length > 0
                   && bt_br_edr_per_recv_params.pkt_length <= BT_3EV5_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
      }

      if (!((bt_br_edr_per_recv_params.br_edr_indication >= ONE_MBPS)
            && (bt_br_edr_per_recv_params.br_edr_indication <= THREE_MBPS))) {
        printf("Invalid BT_EDR Mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_recv_params.enable_hopping >= FREQ_NO_HOP)
            && (bt_br_edr_per_recv_params.enable_hopping <= FREQ_RAND_HOP))) {
        printf("Invalid Hopping Mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_recv_params.link_type >= SCO_LINK) && (bt_br_edr_per_recv_params.link_type <= ESCO_LINK))) {
        printf("Invalid link type Mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_recv_params.loop_back_mode >= DISABLE_LOOP_BACK)
            && (bt_br_edr_per_recv_params.loop_back_mode <= ENABLE_LOOP_BACK))) {
        printf("Invalid loop_back_mode value\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_recv_params.pll_mode >= PLL_MODE0) && (bt_br_edr_per_recv_params.pll_mode <= PLL_MODE2))) {
        printf("Invalid pll mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_recv_params.rf_type >= EXTERNAL_RF) && (bt_br_edr_per_recv_params.rf_type <= INTERNAL_RF))) {
        printf("Invalid rf_type\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if ((bt_br_edr_per_recv_params.rf_chain > BT_LP_CHAIN) || (bt_br_edr_per_recv_params.rf_chain == WLAN_LP_CHAIN)) {
        printf("Invalid rf chain \n");
        return ONEBOX_STATUS_FAILURE;
      }
    } else {
      printf("Invalid Driver mode\n");
      return ONEBOX_STATUS_FAILURE;
    }

    bb_rf_params.value        = PER_BR_EDR_RECEIVE;
    bb_rf_params.no_of_values = sizeof(bt_br_edr_per_recv_params_t);
    memcpy(&bb_rf_params.Data[0], &bt_br_edr_per_recv_params, sizeof(bt_br_edr_per_recv_params_t));
    usleep(500000);
    if (bt_per_cmd_wrapper(bb_rf_params, BT_PER, sfd, BT_PACKET) < 0) {
      printf("Unable to perform bt_receive\n");
      return ONEBOX_STATUS_FAILURE;
    }
    /* Driver does not send a netlink status for PER RX (matches Proprietary / ble_receive) */
    printf("======== SUCCESS ============\n");
  } else if (argc == STOP_OF_ARGS) {
    if (strcmp(argv[1], "0")) {
      printf("Please enter either 0 as an argument, instead of %s to stop..\n", argv[1]);
      return ONEBOX_STATUS_FAILURE;
    }
    bt_br_edr_per_recv_params.enable = 0;
    bb_rf_params.value               = PER_BR_EDR_RECEIVE;
    bb_rf_params.no_of_values        = sizeof(bt_br_edr_per_recv_params_t);
    /* Use Data[0] (not Proprietary Data[1] bug) so enable=0 is seen by FW */
    memcpy(&bb_rf_params.Data[0], &bt_br_edr_per_recv_params, sizeof(bt_br_edr_per_recv_params_t));
    usleep(500000);
    if (bt_per_cmd_wrapper(bb_rf_params, BT_PER, sfd, BT_PACKET) < 0) {
      printf("Unable to stop bt_receive\n");
      return ONEBOX_STATUS_FAILURE;
    }
    printf("============= BT_RECEIVE_STOP ==================\n");
  } else {
    printf("USAGE: %s <dev_addr> <link_type> <pkt_type> <pkt_length> <scrambler_seed> <br_edr_mode> "
           "<rx_channel_index> <tx_channel_index> <hopping_type> <ant_sel> <loop_back_mode> <pll_mode> "
           "<rf_type> <rf_chain>\n",
           argv[0]);
    return ONEBOX_STATUS_SUCCESS;
  }
  return ONEBOX_STATUS_SUCCESS;
}
