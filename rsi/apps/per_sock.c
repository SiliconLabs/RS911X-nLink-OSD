// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020-2023 Silicon Labs, Inc.
 */

#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "per_util.h"
#include <unistd.h>
#include <linux/wireless.h>

int socket_creation()
{
  int sock_fd;
  struct sockaddr_nl src_addr;
  sock_fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_USER);
  if (sock_fd < 0)
    return -1;
  memset(&src_addr, 0, sizeof(src_addr));
  src_addr.nl_family = AF_NETLINK;
  src_addr.nl_pid    = getpid(); /* self pid */

  bind(sock_fd, (struct sockaddr *)&src_addr, sizeof(src_addr));
  return sock_fd;
}

int common_send_mesg_wrapper(int sock_fd, struct sockaddr_nl dest_addr, struct nlmsghdr *nlh)
{
  struct iovec iov  = { 0 };
  struct msghdr msg = { 0 };
  iov.iov_base      = (void *)nlh;
  iov.iov_len       = nlh->nlmsg_len;
  msg.msg_name      = (void *)&dest_addr;
  msg.msg_namelen   = sizeof(dest_addr);
  msg.msg_iov       = &iov;
  msg.msg_iovlen    = 1;
  if (sendmsg(sock_fd, &msg, 0) < 0) {
    printf("Sending Failed\n");
    return -1;
  }
  return 0;
}

struct nlmsghdr *common_recv_mesg_wrapper(int sock_fd, int len)
{
  /* Read message from kernel */
  struct msghdr msg = { 0 };
  struct iovec iov  = { 0 };
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh;
  nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(len));
  memset(nlh, 0, NLMSG_SPACE(len));
  nlh->nlmsg_len  = NLMSG_SPACE(len);
  iov.iov_base    = (void *)nlh;
  iov.iov_len     = nlh->nlmsg_len;
  msg.msg_name    = (void *)&dest_addr;
  msg.msg_namelen = sizeof(dest_addr);
  msg.msg_iov     = &iov;
  msg.msg_iovlen  = 1;

  if (recvmsg(sock_fd, &msg, 0) < 0) {
    printf("RECEVING FAILED\n");
    close(sock_fd);
    free(nlh);
    return NULL;
  }

  return nlh;
}

int gain_table_sckt_creation(update_wlan_gain_table_t table_info, int sfd)
{
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh        = NULL;
  struct rsi_nl_desc *nl_desc = NULL;
  int ret;
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.nl_family = AF_NETLINK;
  dest_addr.nl_pid    = 0; /* For Linux Kernel */
  dest_addr.nl_groups = 0; /* unicast */
  nlh                 = (struct nlmsghdr *)malloc(NLMSG_SPACE(table_info.struct_size + 8 + NL_DATA_DESC_SZ));
  memset(nlh, 0, NLMSG_SPACE(table_info.struct_size + 8 + NL_DATA_DESC_SZ));
  nlh->nlmsg_len        = NLMSG_SPACE(table_info.struct_size + 8 + NL_DATA_DESC_SZ);
  nl_desc               = (struct rsi_nl_desc *)NLMSG_DATA(nlh);
  nl_desc->desc_word[0] = UPDATE_WLAN_GAIN_TABLE;
  nl_desc->desc_word[1] = table_info.struct_size + 8;
  nlh->nlmsg_type       = (unsigned short)WLAN_PACKET;
  nlh->nlmsg_pid        = getpid();
  nlh->nlmsg_flags      = 0;
  memcpy(NLMSG_DATA(nlh) + NL_DATA_DESC_SZ, &table_info, table_info.struct_size + 8);
  ret = common_send_mesg_wrapper(sfd, dest_addr, nlh);
  if (ret < 0)
    close(sfd);
  free(nlh);
  return ret;
}

int per_recv_send_wrapper(per_stats *sta_info,
                          int cmd,
                          int len,
                          int stop_bit,
                          unsigned char rate_flags,
                          int freq,
                          int sock_fd)
{
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh         = NULL;
  struct rsi_nl_desc *nlh_desc = NULL;
  int ret;
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.nl_family = AF_NETLINK;
  dest_addr.nl_pid    = 0; /* For Linux Kernel */
  dest_addr.nl_groups = 0; /* unicast */
  nlh                 = (struct nlmsghdr *)malloc(NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  memset(nlh, 0, NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  nlh->nlmsg_len         = NLMSG_SPACE(len + NL_DATA_DESC_SZ);
  nlh_desc               = (struct rsi_nl_desc *)NLMSG_DATA(nlh);
  nlh_desc->desc_word[0] = cmd;
  nlh_desc->desc_word[1] = rate_flags;
  nlh_desc->desc_word[2] = freq;
  nlh_desc->desc_word[3] = stop_bit;
  nlh->nlmsg_pid         = getpid();
  nlh->nlmsg_flags       = 0;
  nlh->nlmsg_type        = (unsigned short)WLAN_PACKET;
  ret                    = common_send_mesg_wrapper(sock_fd, dest_addr, nlh);
  if ((ret < 0) || stop_bit)
    close(sock_fd);
  free(nlh);
  return ret;
}

int per_transmit_wrapper(per_params_t per_params, int cmd, int sock_fd)
{
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh         = NULL;
  struct rsi_nl_desc *nlh_desc = NULL;
  int ret, len;
  len = sizeof(per_params_t);
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.nl_family = AF_NETLINK;
  dest_addr.nl_pid    = 0; /* For Linux kernel */
  dest_addr.nl_groups = 0; /* unicast */
  nlh                 = (struct nlmsghdr *)malloc(NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  memset(nlh, 0, NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  nlh->nlmsg_len         = NLMSG_SPACE(len + NL_DATA_DESC_SZ);
  nlh_desc               = (struct rsi_nl_desc *)NLMSG_DATA(nlh);
  nlh_desc->desc_word[0] = cmd;
  nlh->nlmsg_pid         = getpid();
  nlh->nlmsg_flags       = 0;
  nlh->nlmsg_type        = (unsigned short)WLAN_PACKET;
  memcpy(NLMSG_DATA(nlh) + NL_DATA_DESC_SZ, &per_params, len);
  ret = common_send_mesg_wrapper(sock_fd, dest_addr, nlh);
  if (ret < 0)
    close(sock_fd);
  free(nlh);
  return ret;
}

int per_transmit_packet_wrapper(per_packet_t per_packet, int cmd, int sock_fd)
{
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh         = NULL;
  struct rsi_nl_desc *nlh_desc = NULL;
  int ret, len;
  len = sizeof(per_packet_t);
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.nl_family = AF_NETLINK;
  dest_addr.nl_pid    = 0; /* For Linux kernel */
  dest_addr.nl_groups = 0; /* unicast */
  nlh                 = (struct nlmsghdr *)malloc(NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  memset(nlh, 0, NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  nlh->nlmsg_len         = NLMSG_SPACE(len + NL_DATA_DESC_SZ);
  nlh_desc               = (struct rsi_nl_desc *)NLMSG_DATA(nlh);
  nlh_desc->desc_word[0] = cmd;
  nlh->nlmsg_pid         = getpid();
  nlh->nlmsg_flags       = 0;
  nlh->nlmsg_type        = (unsigned short)WLAN_PACKET;
  memcpy(NLMSG_DATA(nlh) + NL_DATA_DESC_SZ, &per_packet, len);
  ret = common_send_mesg_wrapper(sock_fd, dest_addr, nlh);
  if (ret < 0)
    close(sock_fd);
  free(nlh);
  return ret;
}

int bb_read_write_wrapper(struct bb_rf_param_t bb_rf_params, int sock_fd)
{
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh         = NULL;
  struct rsi_nl_desc *nlh_desc = NULL;
  int ret, len;
  len = sizeof(bb_rf_params);
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.nl_family = AF_NETLINK;
  dest_addr.nl_pid    = 0; /* For Linux kernel */
  dest_addr.nl_groups = 0; /* unicast */
  nlh                 = (struct nlmsghdr *)malloc(NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  memset(nlh, 0, NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  nlh->nlmsg_len         = NLMSG_SPACE(len + NL_DATA_DESC_SZ);
  nlh_desc               = (struct rsi_nl_desc *)NLMSG_DATA(nlh);
  nlh_desc->desc_word[0] = RSI_SET_BB_RF;
  nlh->nlmsg_pid         = getpid();
  nlh->nlmsg_flags       = 0;
  nlh->nlmsg_type        = (unsigned short)WLAN_PACKET;
  memcpy(NLMSG_DATA(nlh) + NL_DATA_DESC_SZ, &bb_rf_params, len);
  ret = common_send_mesg_wrapper(sock_fd, dest_addr, nlh);
  if (ret < 0)
    close(sock_fd);
  free(nlh);
  return ret;
}
int rsi_print_efuse_map(struct efuse_content_t *efuse_content, int sock_fd)
{
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh         = NULL;
  struct rsi_nl_desc *nlh_desc = NULL;
  int ret, len;
  len = sizeof(struct efuse_content_t);
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.nl_family = AF_NETLINK;
  dest_addr.nl_pid    = 0; /* For Linux kernel */
  dest_addr.nl_groups = 0; /* unicast */
  nlh                 = (struct nlmsghdr *)malloc(NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  memset(nlh, 0, NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  nlh->nlmsg_len         = NLMSG_SPACE(len + NL_DATA_DESC_SZ);
  nlh_desc               = (struct rsi_nl_desc *)NLMSG_DATA(nlh);
  nlh_desc->desc_word[0] = EFUSE_MAP;
  nlh->nlmsg_pid         = getpid();
  nlh->nlmsg_flags       = 0;
  nlh->nlmsg_type        = (unsigned short)WLAN_PACKET;
  memcpy(NLMSG_DATA(nlh) + NL_DATA_DESC_SZ, efuse_content, len);
  ret = common_send_mesg_wrapper(sock_fd, dest_addr, nlh);
  if (ret < 0)
    close(sock_fd);
  free(nlh);
  return ret;
}

int send_filter_broadcast_frame_to_drv(struct fltr_bcast bcast, int sock_fd)
{
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh         = NULL;
  struct rsi_nl_desc *nlh_desc = NULL;
  int ret, len;
  len = sizeof(struct fltr_bcast);
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.nl_family = AF_NETLINK;
  dest_addr.nl_pid    = 0; /* For Linux kernel */
  dest_addr.nl_groups = 0; /* unicast */
  nlh                 = (struct nlmsghdr *)malloc(NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  memset(nlh, 0, NLMSG_SPACE(len + NL_DATA_DESC_SZ));
  nlh->nlmsg_len         = NLMSG_SPACE(len + NL_DATA_DESC_SZ);
  nlh_desc               = (struct rsi_nl_desc *)NLMSG_DATA(nlh);
  nlh_desc->desc_word[0] = RSI_FILTER_BCAST;
  nlh_desc->desc_word[1] = len;
  nlh->nlmsg_pid         = getpid();
  nlh->nlmsg_flags       = 0;
  nlh->nlmsg_type        = (unsigned short)WLAN_PACKET;
  memcpy(NLMSG_DATA(nlh) + NL_DATA_DESC_SZ, &bcast, len);
  ret = common_send_mesg_wrapper(sock_fd, dest_addr, nlh);
  if (ret < 0)
    close(sock_fd);
  free(nlh);
  return ret;
}

int send_get_rssi_frame_to_drv(int sock_fd)
{
  struct sockaddr_nl dest_addr;
  struct nlmsghdr *nlh         = NULL;
  struct rsi_nl_desc *nlh_desc = NULL;
  int ret, len;
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.nl_family = AF_NETLINK;
  dest_addr.nl_pid    = 0; /* For Linux kernel */
  dest_addr.nl_groups = 0; /* unicast */
  nlh                 = (struct nlmsghdr *)malloc(NLMSG_SPACE(NL_DATA_DESC_SZ));
  memset(nlh, 0, NLMSG_SPACE(NL_DATA_DESC_SZ));
  nlh->nlmsg_len         = NLMSG_SPACE(NL_DATA_DESC_SZ);
  nlh_desc               = (struct rsi_nl_desc *)NLMSG_DATA(nlh);
  nlh_desc->desc_word[0] = RSI_GET_RSSI;
  nlh->nlmsg_pid         = getpid();
  nlh->nlmsg_flags       = 0;
  nlh->nlmsg_type        = (unsigned short)WLAN_PACKET;
  ret                    = common_send_mesg_wrapper(sock_fd, dest_addr, nlh);
  if (ret < 0)
    close(sock_fd);
  free(nlh);
  return ret;
}

