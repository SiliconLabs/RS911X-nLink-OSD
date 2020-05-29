/**
 * @file     rsi_nl_app.h
 * @version  3.6
 * @date     2013-May-16
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief HEADER  
 *
 * @section Description
 * 
 *
 */


#ifndef __RSI_GNUSER_H_
#define __RSI_GNUSER_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>
#include <linux/genetlink.h>

#include "rsi_zigb_types.h"
#include "platform_specific.h"

/*
 * Generic macros for dealing with netlink sockets. Might be duplicated
 * elsewhere. It is recommended that commercial grade applications use
 * libnl or libnetlink and use the interfaces provided by the library
 */
#define GENLMSG_DATA(glh) ((void *)(NLMSG_DATA(glh) + GENL_HDRLEN))
#define GENLMSG_PAYLOAD(glh) (NLMSG_PAYLOAD(glh, 0) - GENL_HDRLEN)
#define NLA_DATA(na) ((void *)((char*)(na) + NLA_HDRLEN))
#define MAX_RCV_SIZE  1600
#define RSI_NL_HEAD_SIZE         (sizeof(struct nlmsghdr) + sizeof(struct genlmsghdr) + sizeof(struct nlattr))
#define RSI_STATUS_OFFSET         12
#define RSI_TWOBYTE_STATUS_OFFSET 12
#define RSI_RSP_TYPE_OFFSET       2
#define RSI_FRAME_DESC_LEN                  16     //@ Length of the frame descriptor, for both read and write
#define GET_SEND_LENGTH(a) ((uint16_t )(*(uint32_t *)(a)))

/* User to Kernel Update Types */
/*enum {
  MODULE_POWER_CYCLE = 0x01,
  UPDATE_JOIN_DONE   = 0x02,
  PS_CONTINUE        = 0x03,
  WKP_FROM_HOST      = 0x04,
};*/


/* type defines */
typedef struct {
  struct nlmsghdr n;
  struct genlmsghdr g;
} rsi_nlPkt_t;

/* Function prototypes */
int32_t rsi_nl_socket_init(void);
void rsi_fill_genl_nl_hdrs_for_cmd(void);
uint8_t *rsi_alloc_and_init_cmdbuff(uint8_t *Desc, uint8_t *payload, uint16_t payload_size);
int16_t rsi_send_usr_cmd(uint8_t *buff, uint16_t bufLen);

int32_t rsi_sendto_fd(int32_t s, const uint8_t *buf, int32_t bufLen);
int32_t rsi_get_family_id(int32_t sd);
void rsi_enqueue_to_rcv_q(pkt_struct_t *Pkt);
pkt_struct_t *rsi_dequeue_from_rcv_q(void);
void * RecvThreadBody(void * arg );
uint8_t *rsi_wrapper_to_rsp(uint8_t *rsp, uint8_t rsp_type);
int16_t rsi_update_info(uint8_t type);
int16_t rsi_execute_cmd_serial_to_driver(uint8_t *payloadparam, uint16_t size_param);
uint8_t *rsi_prepare_nl_cmdbuff(uint8_t *payload, uint16_t payload_size);
int16_t rsi_send_driver_pkt_to_dev(void);

#endif //__RSI_GNUSER_H_
