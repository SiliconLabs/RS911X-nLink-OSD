/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	1. Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *
 *	2. Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *
 *	3. Neither the name of the copyright holder nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ONEBOX_COMMON_H__
#define __ONEBOX_COMMON_H__

#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <net/genetlink.h>
#include <net/sock.h>

#if((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))&& \
    (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)))
#include <linux/sdio/ctsystem.h>
#include <linux/sdio/sdio_busdriver.h>
#include <linux/sdio/_sdio_defs.h>
#include <linux/sdio/sdio_lib.h>
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#endif

#ifndef __ONEBOX_ZB_IOCTL_H__
#define __ONEBOX_ZB_IOCTL_H__

#define RSI_ZIGB_SEND    SIOCIWLASTPRIV - 0x1
#define RSI_ZIGB_RECV    SIOCIWLASTPRIV - 0x2
int zigb_ioctl(struct net_device *dev,struct ifreq *ifr, int cmd);

#endif

static u8 device_mac_addr[6] = {0x00, 0x23, 0xa7, 0x27, 0x03, 0x99};

#if 1
/* Kernel version between and including a & b */
#define KERNEL_VERSION_BTWN_2_6_(a,b) \
  ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a)) && \
  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,b)))

#define KERNEL_VERSION_EQUALS_2_6_(a) \
  (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,a))

/* Kernel version greater than equals */
#define KERNEL_VERSION_GREATER_THAN_2_6_(a)  \
 (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a))

/* Kernel version less than or equal to */
#define KERNEL_VERSION_LESS_THAN_3_6(a)  \
 (LINUX_VERSION_CODE <= KERNEL_VERSION(3,6,a))

#define KERNEL_VERSION_LESS_THAN_3_12_(a) \
 (LINUX_VERSION_CODE <= KERNEL_VERSION(3,12,a))
#endif

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#define FRAME_DESC_SZ		16
#define DWORD_ALIGN_SZ		64
#define ZIGB_DEREGISTER		0xff

#define GET_ADAPTER_FROM_GENLCB(gcb) \
		(ZB_ADAPTER)(gcb->gc_drvpriv)

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 11)
# define get_portid(_info) ((_info)->snd_pid)
#else
# define get_portid(_info) ((_info)->snd_portid)
#endif

enum {
	RSI_USER_A_UNSPEC,
	RSI_USER_A_MSG,
	__RSI_USER_A_MAX,
};

enum {
	RSI_USER_C_UNSPEC,
	RSI_USER_C_CMD,
	__RSI_USER_C_MAX,
};

struct genl_cb {
	unsigned char gc_cmd, *gc_name;
	int gc_seq, gc_pid;
	int gc_done;
	int gc_n_ops;
	void  *gc_drvpriv;
	struct nla_policy *gc_policy;
	struct genl_family *gc_family;
	struct genl_ops *gc_ops;
	struct genl_info *gc_info;
	struct sk_buff *gc_skb;
};

struct rsi_zigb_adapter {
	struct rsi_common *priv;
	struct genl_cb *gcb;
};

enum zb_fsm_state {
	RSI_ZB_FSM_DEVICE_NOT_READY = 0,
	RSI_ZB_FSM_DEVICE_READY,
};

struct rsi_zb_adapter
{
	struct net_device *dev; /* Stores the netdevice pointer */
	struct rsi_common *priv;
	u8     mac_addr[6];
	u32    fsm_state;
	struct genl_cb *gcb;
};

int rsi_zigb_attach(void *priv, struct rsi_proto_ops *ops);
void rsi_zigb_detach(void *priv);
int zigb_ioctl(struct net_device *dev,struct ifreq *ifr, int cmd);
int zigb_xmit(struct sk_buff *skb, struct net_device *dev);
int device_close(struct net_device *dev);
int device_open(struct net_device *dev);
int zigb_genl_recv(struct sk_buff *skb, struct genl_info *info);
int rsi_zigb_recv_pkt(void *priv, u8 *pkt);
int rsi_zigb_send_pkt(struct rsi_common *common, struct sk_buff *skb);
int zigb_genl_send(struct genl_cb *gcb, struct sk_buff *skb);
#endif
