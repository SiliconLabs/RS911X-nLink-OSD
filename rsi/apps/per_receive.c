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
#include <linux/types.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include "per_util.h"
#include <signal.h>
#include <stdlib.h>

#define NO_OF_STATS         8
#define NUM_PACKETS_ARG_NUM 7
#define PER_MODE
//#define PER_BASIC_STATS
#ifdef RSSI_AVG
int kk;
#endif

int crc_pass_cumulative = 0, crc_fail_cumulative = 0, no_of_packets = 0;
void receive_stats_exit_handler(int sig)
{
  /* This is for calculating  PER %.
   * This is computed as follows:
   * 1) If you are given a non-zero value for the expected number of packets (N), then just look at the packets received correctly (C) and compute the % PER as (N-C)*100/N
   * 2) If you are given '0' as the expected number of packets, then look at the packets received correctly (C) and the packets with CRC error (E) and compute the % as (E*100)/(C+E).
   */
  float rcv_percentage;
  signal(sig, SIG_IGN);
  if (no_of_packets) {
    rcv_percentage = ((no_of_packets - crc_pass_cumulative) * (100.0 / no_of_packets));
    printf("\n\nExpected packet percentage: %f%\n", rcv_percentage);
  } else {
    rcv_percentage = (crc_fail_cumulative * 100.0) / (crc_pass_cumulative + crc_fail_cumulative);
    printf("\n\nReceived CRC Error percentage: %f%\n", rcv_percentage);
  }
  exit(0);
}

int main(int argc, char **argv)
{
  int sock_fd;
  struct nlmsghdr *nlh;
  unsigned int freq, user_mask = 0;
  int first_time = 1;
  int count = 0, stop_bit = 0;
  int i, pid;
  int cmd, len, length;
  unsigned short ch_width = 0;
  FILE *pFile;
  struct iwreq iwr;
#ifdef RSSI_AVG
  int rssi_buff[10];
  int jj         = 0, avg_loop;
  int rssi_total = 0, rssi_avg = 0;
#endif
  unsigned int valid_channels_5_Ghz[]         = { 36,  40,  44,  48,  52,  56,  60,  64,  100, 104, 108, 112,
                                          116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165 };
  unsigned int valid_channels_5_Ghz_40Mhz[]   = { 38,  42,  46,  50,  54,  58,  62,  102, 106, 110, 114,
                                                118, 122, 126, 130, 134, 138, 151, 155, 159, 163 };
  unsigned int valid_channels_4_9_Ghz_20Mhz[] = { 184, 188, 192, 196, 8, 12, 16, 0xff, 0x00 };
  unsigned int valid_channels_4_9_Ghz_10Mhz[] = { 183, 185, 187, 189, 7, 9, 11, 0xff, 0x00 };
  unsigned char enable_40                     = 0;
  unsigned char enable_11j                    = 0;
  unsigned char rate_flags                    = 0;

  if ((argc != NO_OF_STATS) && (argc != (NO_OF_STATS - 1)) && (argc != (NO_OF_STATS - 2))) {
    printf("Onebox dump stats application\n");
    printf("Usage: %s <rpine_interface> <file_name> <channel num> <start-stop> <Channel BW> [stat selection] [no of "
           "packets]\n",
           argv[0]);
    printf("\tFile_name   		- File to dump status\n");
    printf("\tChannel num 		- Channel to operate\n");
    printf("\tStart-Stop value	- 0: Start & 1: Stop \n");
    printf("\tChannel BW  		- 0: 20MHz, 2: Upper 40MHz, 4: Lower 40MHz 6: Full 40MHz & 8: 20MHz for 11J Channel\n");
    printf("\t[Optional]: stat selection  - value to display the required stats\n");
    printf("\t[Optional]: no_of_packets   - 0: gives pass %, non-zero: gives expected packets %\n");

    return 1;
  }

  freq       = atoi(argv[3]);
  stop_bit   = atoi(argv[4]);
  rate_flags = atoi(argv[5]);
  ch_width = rate_flags & (0x07);
  if ((argc == NO_OF_STATS) && !(stop_bit)) {
    no_of_packets = atoi(argv[NUM_PACKETS_ARG_NUM]);
    signal(SIGINT, receive_stats_exit_handler);
  }
  enable_11j = (rate_flags & BIT(3));

  if (ch_width == BW_U40 || ch_width == BW_L40 || ch_width == BW_F40) {
    enable_40 = 1;
  }

  if (!enable_11j) {
    if (freq == 0xFF) {
      /* Pass 0xFF so as to skip channel programming */
    } else if ((freq >= 36 && freq <= 165 && ch_width == BW_20)) {
      for (i = 0; i < 24; i++) {
        if (freq == valid_channels_5_Ghz[i]) {
          break;
        }
      }
      if (i == 24) {
        printf("Invalid Channel issued by user\n");
        exit(0);
      }
    } else if ((freq >= 36 && freq <= 165 && enable_40)) {
      for (i = 0; i < 21; i++) {
        if (freq == valid_channels_5_Ghz_40Mhz[i]) {
          break;
        }
      }
      if (i == 21) {
        printf("Invalid Channel issued by user\n");
        exit(0);
      }
    } else if (!(freq <= 14)) {
      printf("Invalid Channel issued by user\n");
      exit(0);
    }
  } else {
    if (ch_width == BW_20) {
      for (i = 0; i < sizeof(valid_channels_4_9_Ghz_20Mhz) / sizeof(valid_channels_4_9_Ghz_20Mhz[0]); i++) {
        if (freq == valid_channels_4_9_Ghz_20Mhz[i]) {
          break;
        }
      }
      if (i == sizeof(valid_channels_4_9_Ghz_20Mhz) / sizeof(valid_channels_4_9_Ghz_20Mhz[0])) {
        printf("Invalid Channel issued by user\n");
        exit(0);
      }
    } else if (ch_width == BW_10) {
      for (i = 0; i < sizeof(valid_channels_4_9_Ghz_10Mhz) / sizeof(valid_channels_4_9_Ghz_10Mhz[0]); i++) {
        if (freq == valid_channels_4_9_Ghz_10Mhz[i]) {
          break;
        }
      }
      if (i == sizeof(valid_channels_4_9_Ghz_10Mhz) / sizeof(valid_channels_4_9_Ghz_10Mhz[0])) {
        printf("Invalid Channel issued by user\n");
        exit(0);
      }
    } else if (ch_width == BW_5) {
      printf("5MHz BW is not supported\n");
      exit(0);
    } else {
      printf("Invalid BW Configuration\n");
      exit(0);
    }
  }

  pFile = fopen(argv[2], "w");
  if (pFile == NULL) {
    printf("Unable to create a file\n");
    return -1;
  }

  per_stats *sta_info = malloc(sizeof(per_stats));
  length              = sizeof(per_stats);
  sock_fd             = socket_creation();
  if (sock_fd < 0) {
    return -1;
  }
  if (!stop_bit)
    cmd = (unsigned short)PER_RECEIVE;
  else
    cmd = (unsigned short)PER_RCV_STOP;
  len = sizeof(sta_info);
  printf("receive application\n");
  if (per_recv_send_wrapper(sta_info, cmd, len, stop_bit, rate_flags, freq, sock_fd) < 0)
    return -1;

  while (1) {
#if 0
		if(sleep(1)!=0)
		{
			printf("Unable to sleep\n");            
			free(sta_info);        
			break;
		}
#endif

    if (stop_bit) {
      printf("RECEIVE STOPPED\n");
      break;
    }
    nlh = common_recv_mesg_wrapper(sock_fd, length);
    if (nlh == NULL) {
      break;
    } else {

#ifdef RSSI_AVG
      if (jj < 10) {
        rssi_buff[jj] = sta_info->cal_rssi;
        jj            = jj + 1;
      } else {
        rssi_total    = 0;
        rssi_buff[kk] = sta_info->cal_rssi;
        for (avg_loop = 0; avg_loop < 10; avg_loop++) {
          rssi_total = rssi_total + rssi_buff[avg_loop];
        }
        rssi_avg = rssi_total / 10;
        kk++;
        sta_info->cal_rssi = rssi_avg;
        if (kk == 10) {
          kk = 0;
        }
      }
#endif

      freq = 0;
      memcpy(sta_info, NLMSG_DATA(nlh), length);

      if (first_time) {
        first_time = 0;
        continue;
      }
      crc_pass_cumulative += sta_info->crc_pass;
      crc_fail_cumulative += sta_info->crc_fail;


        if ((count % 20) == 0) {
#ifdef PER_MODE
#ifdef PER_BASIC_STATS
          printf(" %8s %8s %8s %8s\n", "crc_pass", "crc_fail", "false_cca", "cal_rssi");
          fprintf(pFile, "%8s %8s %8s %8s\n", "crc_pass", "crc_fail", "false_cca", "cal_rssi");
#else
          printf("%12s %12s\n", "crc_pass", "crc_fail");
          fprintf(pFile, "\n%12s %12s\n", "crc_pass", "crc_fail");
#endif
        }
#ifdef PER_BASIC_STATS
        printf("%7d %7d %7d %7d\n", sta_info->crc_pass, sta_info->crc_fail, sta_info->cca_idle, sta_info->cal_rssi);
        fprintf(pFile,
                "%7d %7d %7d %7d\n",
                sta_info->crc_pass,
                sta_info->crc_fail,
                sta_info->cca_idle,
                sta_info->cal_rssi);
#else
        printf("%12d %12d\n", sta_info->crc_pass, sta_info->crc_fail);
        fprintf(pFile, "%12d %12d\n", sta_info->crc_pass, sta_info->crc_fail);
#endif
#else
          printf(" %20s %8s %10s %10s %10s \n", "tx_pkts", "retries", "pkts_drop", "rssi", "cons_drops");
          fprintf(pFile, "%20s %10s %10s %10s %10s \n", "tx_pkts", "retries", "pkts_drop", "rssi", "cons_drops");
        }
        printf("%20d %9d %9d %9d %9d \n",
               sta_info->tx_pkts,
               sta_info->tx_retries,
               sta_info->xretries,
               sta_info->bea_avg_rssi,
               sta_info->max_cons_pkts_dropped);
        fprintf(pFile,
                "%20d %9d %9d %9d %9d\n",
                sta_info->tx_pkts,
                sta_info->tx_retries,
                sta_info->xretries,
                sta_info->bea_avg_rssi,
                sta_info->max_cons_pkts_dropped);
#endif

      ++count;
      free(nlh);
    }
    fflush(pFile);
  }
  free(sta_info);
  close(sock_fd);
  fclose(pFile);

  return 0;
}
