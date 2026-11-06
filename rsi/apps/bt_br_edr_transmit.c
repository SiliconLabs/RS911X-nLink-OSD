// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020-2023 Silicon Labs, Inc.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include "bt_util.h"
#include "per_util.h"

#define NO_OF_ARGS   19
#define STOP_OF_ARGS 2

int main(int argc, char *argv[])
{
  int rc = 0, sfd, len, status = 0;
  bt_br_edr_per_params_t bt_br_edr_per_params;
  struct bb_rf_param_bt_t bb_rf_params;
  struct nlmsghdr *nlh;
  unsigned int num_1;
  unsigned char macaddr[6];

  memset(&bt_br_edr_per_params, 0, sizeof(bt_br_edr_per_params_t));
  memset(&bb_rf_params, 0, sizeof(bb_rf_params));

  sfd = socket_creation();
  if (sfd < 0) {
    ONEBOX_PRINT("Socket Creation Failed\n");
    return ONEBOX_STATUS_FAILURE;
  }

  if (argc == NO_OF_ARGS) {
    memset(macaddr, 0, 6);
    byteconversion(argv[1], macaddr);

    bt_br_edr_per_params.enable = 1;
    for (num_1 = 0; num_1 < 6; num_1++) {
      bt_br_edr_per_params.bt_addr[5 - num_1] = macaddr[num_1];
    }
    bt_br_edr_per_params.pkt_type          = atoi(argv[2]);
    bt_br_edr_per_params.pkt_length        = atoi(argv[3]);
    bt_br_edr_per_params.br_edr_indication = atoi(argv[4]);
    bt_br_edr_per_params.bt_rx_rf_chnl     = atoi(argv[5]);
    bt_br_edr_per_params.bt_tx_rf_chnl     = atoi(argv[6]);
    bt_br_edr_per_params.link_type         = atoi(argv[7]);
    bt_br_edr_per_params.scrambler_seed    = atoi(argv[8]);
    bt_br_edr_per_params.num_pkts          = atoi(argv[9]);
    bt_br_edr_per_params.payload_data_type = atoi(argv[10]);
    bt_br_edr_per_params.tx_pwr_indx       = atoi(argv[11]);
    bt_br_edr_per_params.transmit_mode     = atoi(argv[12]);
    bt_br_edr_per_params.enable_hopping    = atoi(argv[13]);
    bt_br_edr_per_params.ant_select        = atoi(argv[14]);
    bt_br_edr_per_params.inter_pkt_gap     = atoi(argv[15]);
    bt_br_edr_per_params.pll_mode          = atoi(argv[16]);
    bt_br_edr_per_params.rf_type           = atoi(argv[17]);
    bt_br_edr_per_params.rf_chain          = atoi(argv[18]);

    bb_rf_params.value = TX_STATUS;
    if (bt_per_cmd_wrapper(bb_rf_params, BT_PER, sfd, BT_PACKET) < 0) {
      printf("Unable to perform bt_transmit\n");
      return ONEBOX_STATUS_FAILURE;
    }
    len = sizeof(int);
    nlh = common_recv_mesg_wrapper(sfd, len);
    if (nlh == NULL) {
      printf("Error receving from bb\n");
      return ONEBOX_STATUS_FAILURE;
    }
    memcpy(&status, NLMSG_DATA(nlh), len);
    free(nlh);
    if (status < 0) {
      printf("======== ALREADY IN TRANSMIT SO STOP TRANSMIT FIRST ============\n");
      return ONEBOX_STATUS_FAILURE;
    }

    bb_rf_params.value = GET_DRV_COEX_MODE;
    if (bt_per_cmd_wrapper(bb_rf_params, BT_PER, sfd, BT_PACKET) < 0) {
      printf("Unable to perform bt_transmit\n");
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
      if (bt_br_edr_per_params.bt_rx_rf_chnl < BT_CLASSIC_START_CHANNEL
          || bt_br_edr_per_params.bt_rx_rf_chnl > BT_CLASSIC_END_CHANNEL) {
        printf("Invalid rx_channel in CLASSIC oper mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (bt_br_edr_per_params.bt_tx_rf_chnl < BT_CLASSIC_START_CHANNEL
          || bt_br_edr_per_params.bt_tx_rf_chnl > BT_CLASSIC_END_CHANNEL) {
        printf("Invalid tx_channel in CLASSIC oper mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if ((bt_br_edr_per_params.enable_hopping == FREQ_FIXED_HOP)
          && (bt_br_edr_per_params.bt_tx_rf_chnl == bt_br_edr_per_params.bt_rx_rf_chnl)) {
        printf("Tx and Rx channels should be different in the fixed hopping\n");
        return ONEBOX_STATUS_FAILURE;
      }

      if (bt_br_edr_per_params.link_type == ACL_LINK) {
        if (bt_br_edr_per_params.br_edr_indication == ONE_MBPS) {
          if (bt_br_edr_per_params.pkt_type == BT_DM1_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_DM1_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_DH1_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_DH1_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_DM3_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_DM3_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_DH3_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_DH3_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_DM5_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_DM5_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_DH5_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_DH5_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
        if (bt_br_edr_per_params.br_edr_indication == TWO_MBPS) {
          if (bt_br_edr_per_params.pkt_type == BT_2DH1_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_2DH1_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_2DH3_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_2DH3_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_2DH5_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_2DH5_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
        if (bt_br_edr_per_params.br_edr_indication == THREE_MBPS) {
          if (bt_br_edr_per_params.pkt_type == BT_3DH1_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_3DH1_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_3DH3_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_3DH3_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_3DH5_PKT_TYPE
              && bt_br_edr_per_params.pkt_length > BT_3DH5_PAYLOAD_MAX_LEN) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
      }

      if (bt_br_edr_per_params.link_type == SCO_LINK && bt_br_edr_per_params.br_edr_indication == ONE_MBPS) {
        if (bt_br_edr_per_params.pkt_type == BT_HV1_PKT_TYPE
            && bt_br_edr_per_params.pkt_length != BT_HV1_VOICE_PAYLOAD_LEN) {
          printf("Invalid packet length\n");
          return ONEBOX_STATUS_FAILURE;
        }
        if (bt_br_edr_per_params.pkt_type == BT_HV2_PKT_TYPE
            && bt_br_edr_per_params.pkt_length != BT_HV2_VOICE_PAYLOAD_LEN) {
          printf("Invalid packet length\n");
          return ONEBOX_STATUS_FAILURE;
        }
        if (bt_br_edr_per_params.pkt_type == BT_HV3_PKT_TYPE
            && bt_br_edr_per_params.pkt_length != BT_HV3_VOICE_PAYLOAD_LEN) {
          printf("Invalid packet length\n");
          return ONEBOX_STATUS_FAILURE;
        }
      }

      if (bt_br_edr_per_params.link_type == ESCO_LINK) {
        if (bt_br_edr_per_params.br_edr_indication == ONE_MBPS) {
          if (bt_br_edr_per_params.pkt_type == BT_EV3_PKT_TYPE
              && !(bt_br_edr_per_params.pkt_length > 0
                   && bt_br_edr_per_params.pkt_length <= BT_EV3_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_EV4_PKT_TYPE
              && !(bt_br_edr_per_params.pkt_length > 0
                   && bt_br_edr_per_params.pkt_length <= BT_EV4_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_EV5_PKT_TYPE
              && !(bt_br_edr_per_params.pkt_length > 0
                   && bt_br_edr_per_params.pkt_length <= BT_EV5_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
        if (bt_br_edr_per_params.br_edr_indication == TWO_MBPS) {
          if (bt_br_edr_per_params.pkt_type == BT_2EV3_PKT_TYPE
              && !(bt_br_edr_per_params.pkt_length > 0
                   && bt_br_edr_per_params.pkt_length <= BT_2EV3_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_2EV5_PKT_TYPE
              && !(bt_br_edr_per_params.pkt_length > 0
                   && bt_br_edr_per_params.pkt_length <= BT_2EV5_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
        if (bt_br_edr_per_params.br_edr_indication == THREE_MBPS) {
          if (bt_br_edr_per_params.pkt_type == BT_3EV3_PKT_TYPE
              && !(bt_br_edr_per_params.pkt_length > 0
                   && bt_br_edr_per_params.pkt_length <= BT_3EV3_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
          if (bt_br_edr_per_params.pkt_type == BT_3EV5_PKT_TYPE
              && !(bt_br_edr_per_params.pkt_length > 0
                   && bt_br_edr_per_params.pkt_length <= BT_3EV5_VOICE_PAYLOAD_LEN)) {
            printf("Invalid packet length\n");
            return ONEBOX_STATUS_FAILURE;
          }
        }
      }

      if (!((bt_br_edr_per_params.enable_hopping >= FREQ_NO_HOP)
            && (bt_br_edr_per_params.enable_hopping <= FREQ_RAND_HOP))) {
        printf("Invalid Hopping Mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (bt_br_edr_per_params.transmit_mode > 1) {
        printf("Invalid transmit Mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_params.br_edr_indication >= ONE_MBPS)
            && (bt_br_edr_per_params.br_edr_indication <= THREE_MBPS))) {
        printf("Invalid BT_EDR Mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_params.link_type >= SCO_LINK) && (bt_br_edr_per_params.link_type <= ESCO_LINK))) {
        printf("Invalid link type\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_params.pll_mode >= PLL_MODE0) && (bt_br_edr_per_params.pll_mode <= PLL_MODE1))) {
        printf("Invalid pll mode\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if (!((bt_br_edr_per_params.rf_type >= EXTERNAL_RF) && (bt_br_edr_per_params.rf_type <= INTERNAL_RF))) {
        printf("Invalid rf type\n");
        return ONEBOX_STATUS_FAILURE;
      }
      if ((bt_br_edr_per_params.rf_chain > BT_LP_CHAIN) || (bt_br_edr_per_params.rf_chain == WLAN_LP_CHAIN)) {
        printf("Invalid rf chain\n");
        return ONEBOX_STATUS_FAILURE;
      }
    } else {
      printf("Invalid Driver mode\n");
      return ONEBOX_STATUS_FAILURE;
    }

    printf("the packet type is %d \n", bt_br_edr_per_params.pkt_type);
    printf("the packet length is %d \n", bt_br_edr_per_params.pkt_length);

    bb_rf_params.value        = PER_BR_EDR_TRANSMIT;
    bb_rf_params.no_of_values = sizeof(bt_br_edr_per_params_t);
    memcpy(&bb_rf_params.Data[0], &bt_br_edr_per_params, sizeof(bt_br_edr_per_params_t));
    usleep(500000);

    if (bt_per_cmd_wrapper(bb_rf_params, BT_PER, sfd, BT_PACKET) < 0) {
      printf("Unable to perform bt_transmit\n");
      return ONEBOX_STATUS_FAILURE;
    }
    /* Driver does not send a netlink status for PER TX (matches Proprietary / ble_transmit) */
    printf("======== SUCCESS ============\n");
  } else if (argc == STOP_OF_ARGS) {
    if (strcmp(argv[1], "0")) {
      printf("Please enter either 0 as an argument, instead of %s to stop..\n", argv[1]);
      return ONEBOX_STATUS_FAILURE;
    }
    bt_br_edr_per_params.enable = 0;
    bb_rf_params.value          = PER_BR_EDR_TRANSMIT;
    bb_rf_params.no_of_values   = sizeof(bt_br_edr_per_params_t);
    memcpy(&bb_rf_params.Data[0], &bt_br_edr_per_params, sizeof(bt_br_edr_per_params_t));
    usleep(500000);
    if (bt_per_cmd_wrapper(bb_rf_params, BT_PER, sfd, BT_PACKET) < 0) {
      printf("Unable to perform bt_transmit\n");
      return ONEBOX_STATUS_FAILURE;
    }
    printf("============= BT_TRANSMIT_STOP ==================\n");
  } else {
    printf("\nUSAGE to start transmit: %s <dev_addr> <pkt_type> <pkt_length> <br_edr_mode> <rx_channel_index> "
           "<tx_channel_index> <link_type> <scrambler_seed> <no_of_packets> <payload_type> <tx_power> <tx_mode> "
           "<hopping_type> <ant_sel> <inter_pkt_gap> <pll_mode> <rf_type> <rf_chain>\n",
           argv[0]);
    return ONEBOX_STATUS_SUCCESS;
  }
  return ONEBOX_STATUS_SUCCESS;
}
