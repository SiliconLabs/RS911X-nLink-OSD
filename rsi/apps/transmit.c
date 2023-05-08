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
#include <linux/types.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include "per_util.h"

int is11brate(unsigned int rate)
{
  switch (rate) {
    case RSI_RATE_1:
    case RSI_RATE_2:
    case RSI_RATE_5_5:
    case RSI_RATE_11:
      return 1;
    default:
      return 0;
  }
}
int islegacyrate(unsigned int rate)
{
  switch (rate) {
    case RSI_RATE_6:
    case RSI_RATE_9:
    case RSI_RATE_12:
    case RSI_RATE_18:
    case RSI_RATE_24:
    case RSI_RATE_36:
    case RSI_RATE_48:
    case RSI_RATE_54:
      return 1;
    default:
      return 0;
  }
}
//main
int main(int argc, char *argv[])
{
  struct iwreq iwr;
  int tx_pwr, tx_pktlen, tx_mode, chan_number;
  int sockfd, i, cmd;
  char *tmp_rate;
  per_params_t per_params;
  unsigned char rate_flags                    = 0;
  unsigned int valid_channels_5_Ghz[]         = { 36,  40,  44,  48,  52,  56,  60,  64,  100, 104, 108, 112,
                                          116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165 };
  unsigned int valid_channels_5_Ghz_40Mhz[]   = { 38,  42,  46,  50,  54,  58,  62,  102, 106, 110, 114,
                                                118, 122, 126, 130, 134, 138, 151, 155, 159, 163 };
  unsigned int valid_channels_4_9_Ghz_20Mhz[] = { 184, 188, 192, 196, 8, 12, 16, 0xff, 0x0 };
  unsigned int valid_channels_4_9_Ghz_10Mhz[] = { 183, 185, 187, 189, 7, 9, 11, 0xff, 0x0 };
  unsigned char enable_40                     = 0;
  unsigned char enable_11j                    = 0;

  /*Creating a Socket*/
  sockfd = socket_creation();
  if (sockfd < 0) {
    printf("Unable to create a socket\n");
    return sockfd;
  }

  memset(&per_params, 0, sizeof(per_params_t));
  if ((argc == 13)
  ) {

    per_params.enable     = 1;
    per_params.enable_11j = 0;

    tx_pwr = atoi(argv[2]);
    printf("TX PWR is %d\n", tx_pwr);
#if 0
		if (tx_pwr > 63)
		{
			tx_pwr = 63; 
		}
		if (tx_pwr > 31 && tx_pwr < 63)
		{
			tx_pwr += 32;
		}
#endif
    if ((tx_pwr >= 0 && tx_pwr <= 30) || tx_pwr == 127) {
      per_params.power = tx_pwr;
    } else {
      printf("Invalid tx_pwr is given by user . Please enter tx_pwr between 0 to 30 (or) tx_pwr == 127 \n");
      exit(0);
    }

    //Rate
    tmp_rate = argv[3];

    if (!strcmp(tmp_rate, "1"))
      per_params.rate = RSI_RATE_1;
    else if (!strcmp(tmp_rate, "2"))
      per_params.rate = RSI_RATE_2;
    else if (!strcmp(tmp_rate, "5.5"))
      per_params.rate = RSI_RATE_5_5;
    else if (!strcmp(tmp_rate, "11"))
      per_params.rate = RSI_RATE_11;
    else if (!strcmp(tmp_rate, "6"))
      per_params.rate = RSI_RATE_6;
    else if (!strcmp(tmp_rate, "9"))
      per_params.rate = RSI_RATE_9;
    else if (!strcmp(tmp_rate, "12"))
      per_params.rate = RSI_RATE_12;
    else if (!strcmp(tmp_rate, "18"))
      per_params.rate = RSI_RATE_18;
    else if (!strcmp(tmp_rate, "24"))
      per_params.rate = RSI_RATE_24;
    else if (!strcmp(tmp_rate, "36"))
      per_params.rate = RSI_RATE_36;
    else if (!strcmp(tmp_rate, "48"))
      per_params.rate = RSI_RATE_48;
    else if (!strcmp(tmp_rate, "54"))
      per_params.rate = RSI_RATE_54;
    else if (!strcasecmp(tmp_rate, "mcs0"))
      per_params.rate = RSI_RATE_MCS0;
    else if (!strcasecmp(tmp_rate, "mcs1"))
      per_params.rate = RSI_RATE_MCS1;
    else if (!strcasecmp(tmp_rate, "mcs2"))
      per_params.rate = RSI_RATE_MCS2;
    else if (!strcasecmp(tmp_rate, "mcs3"))
      per_params.rate = RSI_RATE_MCS3;
    else if (!strcasecmp(tmp_rate, "mcs4"))
      per_params.rate = RSI_RATE_MCS4;
    else if (!strcasecmp(tmp_rate, "mcs5"))
      per_params.rate = RSI_RATE_MCS5;
    else if (!strcasecmp(tmp_rate, "mcs6"))
      per_params.rate = RSI_RATE_MCS6;
    else if (!strcasecmp(tmp_rate, "mcs7"))
      per_params.rate = RSI_RATE_MCS7;
    else {
      printf("Invalid tx_rate is given by user .  Please enter Valid tx_rate\n");
      exit(0);
    }

    //pkt length
    tx_pktlen = atoi(argv[4]);

    per_params.pkt_length = tx_pktlen;

    //mode
    tx_mode = atoi(argv[5]);
    if (tx_mode == 1 || tx_mode == 0) {
      per_params.mode = tx_mode;
    } else {
      per_params.mode = 0;
    }

    chan_number = atoi(argv[6]);

    if ((chan_number >= 36) && (chan_number <= 165)) {
      if (is11brate(per_params.rate)) {
        printf("Invalid Rate in 5G\n");
        return -1;
      }
    }
    rate_flags             = atoi(argv[8]);
    per_params.rate_flags  = rate_flags;
    per_params.per_ch_bw   = (rate_flags >> 2) & (0x07); /* BW Configuration is BIT(2)-BIT(4) 3 bits in rate flags */
    enable_11j             = (rate_flags & BIT(5));
    per_params.enable_11j  = enable_11j;
    per_params.aggr_enable = atoi(argv[9]);
    per_params.aggr_count  = (per_params.pkt_length / PER_AGGR_LIMIT_PER_PKT);
    if ((per_params.pkt_length - (per_params.aggr_count * PER_AGGR_LIMIT_PER_PKT)) > 0) {
      per_params.aggr_count++;
    }
    if (per_params.aggr_count == 1) {
      per_params.aggr_enable = 0;
      per_params.aggr_count  = 0;
    }
    per_params.no_of_pkts  = atoi(argv[10]);
    per_params.delay       = atoi(argv[11]);
    per_params.ctry_region = atoi(argv[12]);
#if 1
    if (tx_pktlen > 1536 && per_params.aggr_enable == 0) {
      printf("Invalid length,Give the length <= 1536 \n");
      exit(0);
    }
    if ((tx_pktlen > 30000) && (per_params.aggr_enable)) {
      printf("Cant aggregate,Give the length <= 30000 \n");
      exit(0);
    }
    if ((per_params.aggr_enable) && !(per_params.rate >= RSI_RATE_MCS0 && per_params.rate <= RSI_RATE_MCS7)) {
      printf("Cant aggregate,Give 11n rate \n");
      exit(0);
    }
#endif

    if (per_params.per_ch_bw == BW_U40 || per_params.per_ch_bw == BW_L40 || per_params.per_ch_bw == BW_F40) {
      enable_40 = 1;
    }
    if (enable_11j == 0) {
      if (chan_number == 0xFF) {
        per_params.channel = chan_number;
        /* Pass 0xFF so as to skip channel programming */
      } else if (chan_number <= 14 && ((per_params.per_ch_bw == BW_20) || enable_40)) {
        per_params.channel = chan_number;
      } else if ((chan_number >= 36 && chan_number <= 165) && (per_params.per_ch_bw == BW_20)) /* For 20Mhz BW */
      {
        for (i = 0; i < 24; i++) {
          if (chan_number == valid_channels_5_Ghz[i]) {
            per_params.channel = chan_number;
            break;
          }
        }
        if (!(per_params.channel == chan_number)) {
          printf("Invalid Channel issued by user for 20Mhz BW\n");
          exit(0);
        }
      } else if ((chan_number >= 36 && chan_number <= 165) && enable_40) /* For 20Mhz BW */
      {
        for (i = 0; i < 21; i++) {
          if (chan_number == valid_channels_5_Ghz_40Mhz[i]) {
            per_params.channel = chan_number;
            break;
          }
        }
        if (!(per_params.channel == chan_number)) {
          printf("Invalid Channel issued by user for 40Mhz BW\n");
          exit(0);
        }
      } else {
        printf("Invalid parameters for transmit. Please check channel and rate flags\n");
        exit(0);
      }
    } else {
      if (per_params.per_ch_bw == BW_20) {
        for (i = 0; i < sizeof(valid_channels_4_9_Ghz_20Mhz) / sizeof(valid_channels_4_9_Ghz_20Mhz[0]); i++) {
          if (chan_number == valid_channels_4_9_Ghz_20Mhz[i]) {
            per_params.channel = chan_number;
            break;
          }
        }
        if (!(per_params.channel == chan_number)) {
          printf("Invalid Channel issued by user for 20Mhz BW\n");
          exit(0);
        }
      } else if (per_params.per_ch_bw == BW_10) {
        for (i = 0; i < sizeof(valid_channels_4_9_Ghz_10Mhz) / sizeof(valid_channels_4_9_Ghz_10Mhz[0]); i++) {
          if (chan_number == valid_channels_4_9_Ghz_10Mhz[i]) {
            per_params.channel = chan_number;
            break;
          }
        }
        if (!(per_params.channel == chan_number)) {
          printf("Invalid Channel issued by user for 20Mhz BW\n");
          exit(0);
        }
      } else if (per_params.per_ch_bw == BW_5) {
        printf("5MHz BW is not supported\n");
        exit(0);
      } else {
        printf("Invalid BW Configuration\n");
        exit(0);
      }
    }

    printf("\n--Tx TEST CONFIGURATION--\n\n");
    printf("Tx POWER      : %d\n", atoi(argv[2]));
    printf("Tx RATE       : %s\n", argv[3]);
    printf("PACKET LENGTH : %d\n", per_params.pkt_length);
    if (tx_mode == 1) {
      printf("Tx MODE       : CONTINUOUS\n");
      per_params.pkt_length = 28;
    } else if (tx_mode == 0) {
      printf("Tx MODE       : BURST\n");
    } else {
      printf("Tx MODE       : CONTINUOUS\n");
    }
    printf("CHANNEL NUM   : %d\n", chan_number);
    printf("RATE_FLAGS    : %d\n", per_params.rate_flags);
    printf("CHAN_WIDTH    : %d\n", per_params.per_ch_bw);
    printf("AGGR_ENABLE   : %d\n", per_params.aggr_enable);
    printf("NO OF PACKETS : %d\n", per_params.no_of_pkts);
    printf("DELAY         : %d\n", per_params.delay);
    printf("CTRY_REGION : %d\n", per_params.ctry_region);

    if (per_params.ctry_region == 255) {
      /*** Remove Me When Updated in Doc and More regions are added*/
      per_params.ctry_region = 3; /* changing ctry_region to 3  from 127 to make sure same value in PER and End-to-End*/
    } else if ((per_params.ctry_region < 0) || (per_params.ctry_region > 2)) {
      printf("Invalid Country region \n");
      printf("Valid country regions are : 0- FCC(US), 1- ETSI(Europe), 2-JP (japan), 255-World\n");
      return -1;
    }

    cmd = (unsigned short)PER_TRANSMIT;
    if (per_transmit_wrapper(per_params, cmd, sockfd) < 0) {
      printf("Sending Failed\n");
      printf("Transmit already running or Driver not installed properly\n");
    } else
      printf("Tx Started\n");
  }

  else if (argc == 3) {
    if (!(strcmp(argv[2], "1") && strcmp(argv[2], "0"))) {
      per_params.enable = 0;
      cmd               = (unsigned short)PER_TRANSMIT;

      if (per_transmit_wrapper(per_params, cmd, sockfd) < 0) {
        perror(argv[0]);
        printf("&&Please ensure Burst or Continuous Mode is running\n");
      } else {
        printf("Tx Stopped\n");
      }
    } else {
      printf("Please enter either 0 or 1 as an argument, instead of %s to stop..\n", argv[2]);
    }
  }

  else {
    printf("\nUSAGE to start transmit: %s rpine_interface tx_power rate length tx_mode channel ExtPA-Enable Rate_flags "
           "Aggr_enable no_of_packets delay ctry_region\n",
           argv[0]);
    printf("\nUSAGE to stop transmit: %s rpine_interface tx_mode\n\t****** FIELDS *******", argv[0]);
    printf("\ntx_mode : 0 - Burst , 1 - Continuous mode\n");
    printf("\nTX-Power 127 to use max Power in Flash\n");
    printf("\nRate_flags Bits: \n");
    printf("Bit 0		: (Short_GI for HT mode)/(Short_preamble in 11b)\n");
    printf("Bit 1		: (GreenField for HT mode)/(preamble enable for 11b)\n");
    printf("Bit 4-2		: CH_BW flags\n");
    printf("Bit 5		: This bit has to be set when the user selects 11J channel\n\n");
    printf("Bit 15-6	: Reserved\n\n");
    printf("CTRY_REGION : 0- FCC(US), 1- ETSI(Europe), 2-JP (japan), 255-World\n");

#ifdef MODE_11AH
    printf("\n\t11AH\nRate Flags = 8 for 2MHz \n Rate Flags = 12 for 4MHz  	 \n\n");

#endif
    return 0;
  }

  return 0;
}
