/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020-2023 Silicon Labs, Inc.
 */

#ifndef _ONEBOX_UTILS_H_
#define _ONEBOX_UTILS_H_

#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <linux/netlink.h>

#define BW_20  0
#define BW_1   1
#define BW_U40 2
#define BW_2   3
#define BW_L40 4
#define BW_5   5
#define BW_F40 6
#define BW_10  7

typedef struct {
  //! no. of tx pkts
  unsigned short tx_pkts;
  //! no. of rx pkts
  unsigned short rx_pkts;
  //! no. of tx retries
  unsigned short tx_retries;
  //! no. of pkts that pass crc
  unsigned short crc_pass;
  //! no. of pkts failing crc chk
  unsigned short crc_fail;
  //! no. of times cca got stuck
  unsigned short cca_stk;
  //! no of times cca didn't get stuck
  unsigned short cca_not_stk;
  //! no. of pkt aborts
  unsigned short pkt_abort;
  //! no. of false rx starts
  unsigned short fls_rx_start;
  //! cca idle time
  unsigned short cca_idle;
  //! no. of greenfield pkts
  unsigned short gf_pkts;
  //! no. of high throughput pkts
  unsigned short ht_pkts;
  //! no. of legacy pkts
  unsigned short leg_pkt;
  //! add description
  unsigned short leg_prty_fails;
  //! no. of ht pkts failing crc chk
  unsigned short ht_crc_fails;
  //! add description
  unsigned short sp_rejected;
  //! add description
  unsigned short lp_rejected;
  //! Channel 1 signal power
  unsigned short ch1_sig_pow;
  //! channel 1 noise power
  unsigned short ch1_noise_pow;
  //! channel 2 signal power
  unsigned short ch2_sig_pow;
  //! channel 2 noise power
  unsigned short ch2_noise_pow;
  //! channel 3 signal power
  unsigned short ch3_sig_pow;
  //! channel 3 noise power
  unsigned short ch3_noise_pow;
  //! no. of rx retries
  unsigned short rx_retries;
  //! rssi value
  unsigned short bea_avg_rssi;
  //! cal_rssi
  unsigned short cal_rssi;
  //! lna_gain bb_gain
  unsigned short lna_bb_gain;
  //! avg_val
  unsigned short avg_val;
  //! xretries pkts dropped
  unsigned short xretries;
  //! consecutive pkts dropped
  unsigned short max_cons_pkts_dropped;
  //!
  unsigned short false_under_sat;
  //!BSS MATCHED BROADCAST PKT STATS
  unsigned short bss_broadcast_pkts;
  //!BSS MATCHED MULTICAST PKT STATS
  unsigned short bss_multicast_pkts;
  //!BSS and DA MATCHED MULTICAST PKT STATS
  unsigned short bss_da_matched_multicast_pkts;
  unsigned int eof_pkt_drop_count;
  unsigned int mask_pkt_drop_count;
  unsigned int ack_sent;
  //!No.of pkts rcvd with 48M
  unsigned short pkt_rcvd_with_48M;
  //!No.of pkts rcvd with 24M
  unsigned short pkt_rcvd_with_24M;
  //!No.of pkts rcvd with 12M
  unsigned short pkt_rcvd_with_12M;
  //!No.of pkts rcvd with 6M
  unsigned short pkt_rcvd_with_6M;
  //!No.of pkts rcvd with 54M
  unsigned short pkt_rcvd_with_54M;
  //!No.of pkts rcvd with 36M
  unsigned short pkt_rcvd_with_36M;
  //!No.of pkts rcvd with 18M
  unsigned short pkt_rcvd_with_18M;
  //!No.of pkts rcvd with 9M
  unsigned short pkt_rcvd_with_9M;
  //!No.of pkts rcvd with 11M
  unsigned short pkt_rcvd_with_11M;
  //!No.of pkts rcvd with 5.5M
  unsigned short pkt_rcvd_with_5M;
  //!No.of pkts rcvd with 2M
  unsigned short pkt_rcvd_with_2M;
  //!No.of pkts rcvd with 1M
  unsigned short pkt_rcvd_with_1M;
  //!No.of pkts rcvd with mcs0
  unsigned short pkt_rcvd_with_mcs0;
  //!No.of pkts rcvd with mcs1
  unsigned short pkt_rcvd_with_mcs1;
  //!No.of pkts rcvd with mcs2
  unsigned short pkt_rcvd_with_mcs2;
  //!No.of pkts rcvd with mcs3
  unsigned short pkt_rcvd_with_mcs3;
  //!No.of pkts rcvd with mcs4
  unsigned short pkt_rcvd_with_mcs4;
  //!No.of pkts rcvd with mcs5
  unsigned short pkt_rcvd_with_mcs5;
  //!No.of pkts rcvd with mcs6
  unsigned short pkt_rcvd_with_mcs6;
  //!No.of pkts rcvd with mcs7
  unsigned short pkt_rcvd_with_mcs7;
  /* Channel Utilization related stats */
  unsigned int utilization;
  unsigned int rssi_utilization;
  unsigned int tot_bytes;
  unsigned int rssi_bytes;
  unsigned int interval_duration;
  unsigned int false_cca_avg_rssi;
  unsigned int max_cca_avg_rssi;
  unsigned int cca_duration;
  unsigned int ed_duration;
  unsigned short int noise_rssi;
  int stop_per;
} per_stats;


typedef struct per_params_s {
  unsigned short enable;
  signed short power;
  unsigned int rate;
  unsigned short pkt_length;
  unsigned short mode;
  unsigned short channel;
  unsigned short rate_flags;
  unsigned short per_ch_bw;
  unsigned short aggr_enable;
  unsigned short aggr_count;
  unsigned short no_of_pkts;
  unsigned int delay;
  unsigned char ctry_region;
  unsigned char enable_11j;
} per_params_t;

typedef struct per_packet_s {
  unsigned char enable;
  unsigned int length;
  unsigned char insert_seq;
  unsigned char packet[1536];
} per_packet_t;

#define PER_RECEIVE            2
#define PER_RCV_STOP           6
#define PER_TRANSMIT           1
#define PER_PACKET             8
#define PER_AGGR_LIMIT_PER_PKT 1792
#define WLAN_PACKET            1

#define NL_DATA_DESC_SZ 16
#define NETLINK_USER    31

#define RSI_RATE_00      0x00
#define RSI_RATE_1       0x0
#define RSI_RATE_2       0x2
#define RSI_RATE_5_5     0x4
#define RSI_RATE_11      0x6
#define RSI_RATE_6       0x8b
#define RSI_RATE_9       0x8f
#define RSI_RATE_12      0x8a
#define RSI_RATE_18      0x8e
#define RSI_RATE_24      0x89
#define RSI_RATE_36      0x8d
#define RSI_RATE_48      0x88
#define RSI_RATE_54      0x8c
#define RSI_RATE_MCS0    0x100
#define RSI_RATE_MCS1    0x101
#define RSI_RATE_MCS2    0x102
#define RSI_RATE_MCS3    0x103
#define RSI_RATE_MCS4    0x104
#define RSI_RATE_MCS5    0x105
#define RSI_RATE_MCS6    0x106
#define RSI_RATE_MCS7    0x107
#define RSI_RATE_MCS7_SG 0x307

#define RSI_RATE_1M    1
#define RSI_RATE_2M    2
#define RSI_RATE_5_5M  5.5
#define RSI_RATE_11M   11
#define RSI_RATE_6M    6
#define RSI_RATE_9M    9
#define RSI_RATE_12M   12
#define RSI_RATE_18M   18
#define RSI_RATE_24M   24
#define RSI_RATE_36M   36
#define RSI_RATE_48M   48
#define RSI_RATE_54M   54
#define RSI_RATE_6_5M  mcs0
#define RSI_RATE_13M   mcs1
#define RSI_RATE_19_5M mcs2
#define RSI_RATE_26M   mcs3
#define RSI_RATE_39M   mcs4
#define RSI_RATE_52M   mcs5
#define RSI_RATE_58_5M mcs6
#define RSI_RATE_65M   mcs7

struct rsi_nl_desc {
  uint16_t desc_word[8];
};

int socket_creation(void);

#define BIT(n) (1 << (n))
typedef unsigned char uint_8;
typedef char int_8;
typedef unsigned short uint_16;
typedef short int_16;
typedef unsigned int uint_32;
typedef int int_32;

/* EFUSE_MAP related define */
#define EFUSE_MAP 80
struct efuse_content_t {
  u_int16_t efuse_map_version;
  u_int16_t module_version;
  u_int16_t module_part_no;
  u_int16_t mfg_sw_version;
  u_int16_t module_type;
  u_int16_t chip_version;
  u_int16_t m4sb_cert_en;
  u_int16_t mfg_sw_subversion;
  u_int32_t chip_id_no;
  u_int16_t sdb_mode;
};

/* BB READ WRITE related define */
#define RSI_SET_BB_READ      0x01
#define RSI_SET_BB_WRITE     0x02
#define RSI_MULTIPLE_BB_READ 0x03
#define RSI_SET_BB_RF        5
struct bb_rf_param_t {
  unsigned short Data[1024];
  unsigned short no_of_fields;
  unsigned short no_of_values;
  unsigned char value;
  unsigned char soft_reset;
  unsigned char protocol_id;
};
/* Wlan Gain Table related define*/
#define UPDATE_WLAN_GAIN_TABLE   78
#define MAX_WLAN_GAIN_TABLE_SIZE (512 - 128 - 16 - 20 - 8)
typedef struct update_wlan_gain_table_s {
#define _2GHZ_BAND 1
#define _5GHZ_BAND 2
  uint8_t band;
#define BAND_WIDTH_40 1
#define BAND_WIDTH_20 0
  uint8_t bandwidth;
  uint16_t struct_size;
  uint32_t reserved;
  uint8_t structure[MAX_WLAN_GAIN_TABLE_SIZE];
} update_wlan_gain_table_t;

#define RSI_GET_RSSI 81
/*Filter broadcast frame */
#define RSI_FILTER_BCAST 79
struct fltr_bcast {
  unsigned char reserved;
  unsigned char filter_bcast_in_tim;
  unsigned short int beacon_drop_threshold;
  unsigned char filter_bcast_tim_till_next_cmd;
};

int gain_table_sckt_creation(update_wlan_gain_table_t table_info, int sfd);
int update_wlan_gain_table(int argc, char *argv[], char *ifName, int sfd);
int update_wlan_region_based_struct(int argc, char *argv[], char *ifName, int sfd);
struct nlmsghdr *common_recv_mesg_wrapper(int sock_fd, int len);
void usage();
int getcmdnumber(char *command, char *ifName);
int rsi_print_efuse_map(struct efuse_content_t *efuse_content, int sock_fd);
int per_recv_send_wrapper(per_stats *sta_info,
                          int cmd,
                          int len,
                          int stop_bit,
                          unsigned char rate_flags,
                          int freq,
                          int sock_fd);
int bb_read_write_wrapper(struct bb_rf_param_t bb_rf_params, int sock_fd);
int per_transmit_packet_wrapper(per_packet_t per_packet, int cmd, int sock_fd);
int per_transmit_wrapper(per_params_t per_params, int cmd, int sock_fd);
int send_get_rssi_frame_to_drv(int sock_fd);
int send_filter_broadcast_frame_to_drv(struct fltr_bcast bcast, int sock_fd);
#define ONEBOX_STATUS_FAILURE      -1
#define ONEBOX_STATUS_SUCCESS      0
#define ONEBOX_STATUS              int_32
#define ONEBOX_PRINT(fmt, args...) fprintf(stdout, fmt, ##args)

#endif
