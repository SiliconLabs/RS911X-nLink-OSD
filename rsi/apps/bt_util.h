/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020-2023 Silicon Labs, Inc.
 */

#ifndef _BT_UTIL_H_
#define _BT_UTIL_H_

#include <arpa/inet.h>
#include <sys/socket.h>
#include <linux/genetlink.h>

typedef unsigned char uint_8, uint8;
typedef char int_8, int8;
typedef unsigned short uint_16, uint16;
typedef short int_16, int16;
typedef unsigned int uint_32, uint32;
typedef int int_32, int32;
#define NL_DATA_DESC_SZ 16
#define NETLINK_USER    31
#define SLEEP_TIME      998752

struct rsi_nl_desc {
  uint16_t desc_word[8];
};
int socket_creation(void);
int common_send_mesg_wrapper(int sock_fd, struct sockaddr_nl dest_addr, struct nlmsghdr *nlh);

/*Link Type*/
#define SCO_LINK  0
#define ACL_LINK  1
#define ESCO_LINK 2

typedef struct bt_stats_s {
  unsigned short crc_pass;
  unsigned short crc_fail;
  unsigned short tx_aborts;
  unsigned short cca_stk;
  unsigned short cca_idle;
  unsigned short fls_rx_start;
  unsigned short rx_phy_abort;
  unsigned short tx_dones;
  unsigned short fifo_occupied;
  signed short int rssi;
  unsigned short id_pkts_rcvd;
} bt_stats_t;

#define ONEBOX_PRINT(fmt, args...) fprintf(stdout, fmt, ##args)
#define ONEBOX_PRINT_INFO(a, fmt) \
  if (a)                          \
    printf(fmt);

/* Function prototypes */
void usage();
int getcmdnumber(char *command);

#define ONEBOX_STATUS_FAILURE -1
#define ONEBOX_STATUS_SUCCESS 0
#define ONEBOX_STATUS         int_32

#define BIT(n) (1 << (n))

/* Common Frame nos for matlab_utils.c and bt_util.c */
#define BT_E2E_STAT  0x13
#define BT_E2E_STATS 0x26
#define BT_PACKET    2

#define RSI_GET_BT_E2E_STATS        0x0B
#define RSI_GET_BT_E2E_PERIOD_STATS 0x0C

struct bb_rf_param_t {
  unsigned char value; //type is present here
  unsigned char no_of_fields;
  unsigned char no_of_values;
  unsigned char soft_reset;
  unsigned short Data[128];
};
int_32 bt_e2e_stats(uint_8 *file_name, int sfd);
int_32 bt_e2e_period_stats(uint_8 *file_name, int period, int sfd);
#endif
