/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/firmware.h>
#include "rsi_mgmt.h"
#include "rsi_common.h"
#include "rsi_hal.h"
#if defined(CONFIG_RSI_BT_ALONE) || defined(CONFIG_RSI_COEX_MODE)
#include "rsi_hci.h"
#endif
#ifdef CONFIG_RSI_COEX_MODE
#include "rsi_coex.h"
#endif

/*
 * 0 - No handshake
 * 1 - GPIO handshake
 */
u8 lp_handshake_mode = NO_HAND_SHAKE;
/*
 * 0 - No handshake
 * 1 - GPIO handshake
 * 2 - Packet handshake
 */
u8 ulp_handshake_mode = PACKET_HAND_SHAKE;

/*
 * Sleep type ULP or LP
 * ULP = 2, LP = 1
 */
u8 ps_sleep_type = 1;
EXPORT_SYMBOL_GPL(ps_sleep_type);
u16 peer_dist;
u16 bt_feature_bitmap;
u16 uart_debug;
/*
 * Default extended options is (0)
 */
u16 ext_opt = 1;
 /*
  * Default extended options is
  * 3 centrals and 1 peripheral
  */
u16 ble_roles = (1 << 4 | 3);
/*
 * Default BDR and EDR are supported on HP chain
 */
u16 bt_bdr_mode;
/*
 * Default three wire coex is disabled
 */
bool three_wire_coex;
/*
 * Default anchor point gap between connected slaves
 */
u16 anchor_point_gap = 1;
/*
 * Default option for Host Interface on demand is
 * (1) Enable Host Interface on Demand Feature
 */
bool host_intf_on_demand;
/*
 * Default sleep clock derivation source is RC clock.
 */
bool crystal_as_sleep_clk;
bool antenna_diversity = 0;
u16 feature_bitmap_9116;
u8 bt_rf_type = 0x01;
u8 ble_tx_pwr_inx =  0x1E;
u8 ble_pwr_save_options = 0x02;
u8 bt_rf_tx_power_mode;
u8 bt_rf_rx_power_mode;
#if defined(CONFIG_RS9116_PURISM)
u8 antenna_sel = ANTENNA_SEL_UFL;
#else
u8 antenna_sel = ANTENNA_SEL_INT;
#endif
/*
 * LMAC BEACON DROP Feature Options
 * 0 - Disable
 * 1 - Enable
 */
bool lmac_bcon_drop = 1;
/*
 * LP/HP Chain Selection in standby associated mode
 * 0 - HP Chain Enabled
 * 1 - LP Chain Enabled
 */
bool standby_assoc_chain_sel = 1;
/*
 * Power Save options
 * 0 - Disable Duty Cycling & Undestined Packet Drop
 * 1 - Enable Duty Cycling
 * 2 - Enable Undestined Packet Drop
 * 3 - Enable Duty Cycling & Undestined Packet Drop
 */
u16 pwr_save_opt = 3;
/*
 *  Driver Mode Value
 * 1 - END-TO-END mode
 * 2 - RF Evaluation Mode
 * 7 - Sniffer mode
 */
bool enabled_uapsd;
/*
 * 0 - Disable
 * 1 - Enable
 */
u8 max_sp_len;
module_param(max_sp_len, byte, 0);
MODULE_PARM_DESC(max_sp_len, "\n0 -All buffered packets will be delivered\n\
1 -Two buffered packets will be delivered\n\
2 -Four buffered packets will be delivered\n\
3 -Six buffered packets will be delivered\n");

u8 driver_mode_value = 1;

/*
 * Enable support of 40MHZ in 2.4GHZ
 * 1 -Enable
 * 0 -Disable
*/
bool enable_40mhz_in_2g;

module_param(bt_rf_type, byte, 0);
module_param(ble_tx_pwr_inx, byte, 0);
module_param(ble_pwr_save_options, byte, 0);
module_param(bt_rf_tx_power_mode, byte, 0);
module_param(bt_rf_rx_power_mode, byte, 0);

module_param(lp_handshake_mode, byte, 0);
module_param(enabled_uapsd, bool, 0);

MODULE_PARM_DESC(enabled_uapsd,
		 "\n0 - Disable UAPSD, 1 - Enable UAPSD\n");
MODULE_PARM_DESC(lp_handshake_mode,
		 "\n0 - No handshake, 1 - GPIO handshake\n");

module_param(ulp_handshake_mode, byte, 0);
MODULE_PARM_DESC(ulp_handshake_mode,
		 "\n0 - No handshake, 1 - GPIO based, 2 - Pkt based\n");

module_param(ps_sleep_type, byte, 0660);
MODULE_PARM_DESC(ps_sleep_type,
		 "\nPower save type. '1' for  LP and '2' for ULP\n");

module_param(peer_dist, ushort, 0);
MODULE_PARM_DESC(peer_dist, "\npeer distance to config ack timeout value\n");

module_param(bt_feature_bitmap, ushort, 0);
MODULE_PARM_DESC(bt_feature_bitmap, "\nFeature bitmap for BT\n");

module_param(uart_debug, ushort, 0);
MODULE_PARM_DESC(uart_debug, "\nFeature bitmap for uart debug\n");

module_param(ext_opt, ushort, 0);
MODULE_PARM_DESC(ext_opt, "\nExtended options - TBD\n");

module_param(ble_roles, ushort, 0);
MODULE_PARM_DESC(ble_roles, "\nBle Supported Roles BIT[3:0] Max Num of the \
Central Connections BIT[7:4] Max Num of the Peripheral Connections\n");

module_param(bt_bdr_mode, ushort, 0);
MODULE_PARM_DESC(bt_bdr_mode, "\nBDR mode selection in classic. BIT(0) 0:BDR \
and EDR , 1:BDR only mode, BIT(1) 0:BDR HP Chain, 1:BDR LP chain\n");

module_param(three_wire_coex, bool, 0);
MODULE_PARM_DESC(three_wire_coex, "\nThree wire coex selection. 0:Disable, \
1:Enable\n");

module_param(anchor_point_gap, ushort, 0);
MODULE_PARM_DESC(anchor_point_gap, "\nUser configurability of Anchor Point \
gap between connected slaves\n");

module_param(host_intf_on_demand, bool, 0);
MODULE_PARM_DESC(host_intf_on_demand, "\nHost Interface on Demand Feature (0) \
Disable Host Interface on Demand Feature (1) Enable Host Interface on Demand \
Feature\n");

module_param(crystal_as_sleep_clk, bool, 0);
MODULE_PARM_DESC(crystal_as_sleep_clk, "\nSleep clock selection (0) RC clock \
as sleep clock (1) 32KHz crystal as sleep clock\n");

module_param(antenna_diversity, bool, 0);
MODULE_PARM_DESC(antenna_diversity, "\n Anetanna diversity selection(Only for \
STA mode).\n '0' for disable and '1' for enable\n");

module_param(antenna_sel, byte, 0);
MODULE_PARM_DESC(antenna_sel, "\n Antenna selection. '2' for  intenal antenna \
and '3' for External antenna\n");

module_param(feature_bitmap_9116, ushort, 0);
MODULE_PARM_DESC(feature_bitmap_9116, "\n9116 Feature Bitmap BIT(0) 0: AGC_PD \
Enable, 1: AGC_PD Disable BIT(7:1) Reserved\n");

module_param(lmac_bcon_drop, bool, 0);
module_param(standby_assoc_chain_sel, bool, 0);
module_param(pwr_save_opt, ushort, 0);
module_param(driver_mode_value, byte, 0);
module_param(enable_40mhz_in_2g, bool, 0);
MODULE_PARM_DESC(enable_40mhz_in_2g, "\nSupport of 40Mhz in 2.4Gh\n\
'1' - Enable.\n'0' - Disable\n");


u16 rsi_zone_enabled = ERR_ZONE;
module_param(rsi_zone_enabled, ushort, S_IRUGO);
MODULE_PARM_DESC(rsi_zone_enabled, "\nBIT(0) - ERROR ZONE\n\
BIT(1) - INFO ZONE\nBIT(2) - INIT ZONE\nBIT(3) - MGMT TX ZONE\n\
BIT(4) - MGMT RX ZONE\nBIT(5) - DATA TX ZONE\nBIT(6) - DATA RX ZONE\n\
BIT(7) - FSM ZONE\nBIT(8) - ISR ZONE\nBIT(9) - INT_MGMT_ZONE\n\
BIT(10) - MGMT_DEBUG_ZONE\n");

/* Default operating mode is Wi-Fi alone */
int dev_oper_mode_count;
#ifdef CONFIG_CARACALLA_BOARD
#if defined (CONFIG_RSI_COEX_MODE) || defined(CONFIG_RSI_BT_ALONE)
u16 dev_oper_mode = DEV_OPMODE_STA_BT_DUAL;
#else
u16 dev_oper_mode = DEV_OPMODE_WIFI_ALONE;
#endif
#else
#ifdef CONFIG_RSI_MULTI_MODE
u16 dev_oper_mode[5] = {DEV_OPMODE_WIFI_ALONE,
						0xff, 0xff, 0xff, 0xff};
#else
u16 dev_oper_mode = DEV_OPMODE_WIFI_ALONE;
#endif
#endif

#ifdef CONFIG_RSI_MULTI_MODE
module_param_array(dev_oper_mode, ushort, &dev_oper_mode_count, S_IRUGO);
#else
module_param(dev_oper_mode, ushort, S_IRUGO);
#endif
MODULE_PARM_DESC(dev_oper_mode, "\n1  -	Wi-Fi Alone\n4  -	BT Alone\n\
8  -	BT LE Alone\n12 -	BT classic + BT LE\n\
5  -	Wi-Fi STA + BT classic\n9  -	Wi-Fi STA + BT LE\n\
13 -	Wi-Fi STA + BT classic + BT LE\n\
6  -	AP + BT classic\n14 -	AP + BT classic + BT LE\n\
16 -	ZIGB ALONE\n17 -	Wi-Fi STA + ZIGB\n\
32 -	ZIGB COORDINATOR\n48 -	ZIGB ROUTER\n");

#if defined(CONFIG_RSI_COEX_MODE) && defined(CONFIG_RSI_ZIGB)
static struct rsi_proto_ops g_proto_ops = {
	.coex_send_pkt = rsi_coex_send_pkt,
	.get_host_intf = rsi_get_host_intf,
	.set_zb_context = rsi_set_zb_context,
	.get_zb_context = rsi_get_zb_context,
};
#endif

/**
 * rsi_dbg() - This function outputs informational messages.
 * @zone: Zone of interest for output message.
 * @fmt: printf-style format for output message.
 *
 * Return: none
 */
void rsi_dbg(u32 zone, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	if (zone & rsi_zone_enabled)
		pr_info("%pV", &vaf);
	va_end(args);
}
EXPORT_SYMBOL_GPL(rsi_dbg);

/**
 * rsi_hex_dump() - This function prints the packet (/msg) in hex bytes.
 * @zone: Zone of interest for output message.
 * @msg_str: Message to be printed with packet
 * @msg: Packet to be printed
 * @len: Length of the packet
 *
 * Return: none
 */
void rsi_hex_dump(u32 zone, char *msg_str, const u8 *msg, u32 len)
{
	int ii;

	if (!(zone & rsi_zone_enabled))
		return;
	printk("%s: (length = %d)\n", msg_str, len);
	for (ii = 0; ii < len; ii++) {
		if (ii && !(ii % 16))
			printk(KERN_CONT "\n");
		printk(KERN_CONT "%02x ", msg[ii]);
	}
	printk(KERN_CONT "\n");
}
EXPORT_SYMBOL_GPL(rsi_hex_dump);

static char *opmode_str(int oper_mode)
{
	switch (oper_mode) {
	case DEV_OPMODE_WIFI_ALONE:
	       return "Wi-Fi alone";
	case DEV_OPMODE_BT_ALONE:
	       return "BT EDR alone";
	case DEV_OPMODE_BT_LE_ALONE:
	       return "BT LE alone";
	case DEV_OPMODE_BT_DUAL:
	       return "BT Dual";
	case DEV_OPMODE_STA_BT:
	       return "Wi-Fi STA + BT EDR";
	case DEV_OPMODE_STA_BT_LE:
	       return "Wi-Fi STA + BT LE";
	case DEV_OPMODE_STA_BT_DUAL:
	       return "Wi-Fi STA + BT DUAL";
	case DEV_OPMODE_AP_BT:
	       return "Wi-Fi AP + BT EDR";
	case DEV_OPMODE_AP_BT_DUAL:
	       return "Wi-Fi AP + BT DUAL";
	case DEV_OPMODE_ZB_ALONE:
	       return "ZIGB alone";
	case DEV_OPMODE_STA_ZB:
	       return "Wi-Fi STA + ZIGB STA";
	case DEV_OPMODE_ZB_COORDINATOR:
	       return "ZIGB Coordinator";
	case DEV_OPMODE_ZB_ROUTER:
	       return "ZIBG Router";
	}
	return "Unknown";
}

void rsi_print_version(struct rsi_common *common)
{
	struct rsi_hw *adapter = common->priv;
	memcpy(common->driver_ver, DRV_VER, ARRAY_SIZE(DRV_VER));
	common->driver_ver[ARRAY_SIZE(DRV_VER)] = '\0';

	rsi_dbg(ERR_ZONE, "================================================\n");
	rsi_dbg(ERR_ZONE, "================ RSI Version Info ==============\n");
	rsi_dbg(ERR_ZONE, "================================================\n");
	if (adapter->device_model == RSI_DEV_9116) {
		rsi_dbg(ERR_ZONE, "FW Version\t:  %04x.%x.%x.%x.%04x\n",
			common->lmac_ver.chip_id, common->lmac_ver.major,
			common->lmac_ver.minor, common->lmac_ver.customer_id,
			common->lmac_ver.build_id);
	} else {
		rsi_dbg(ERR_ZONE, "FW Version\t: %d.%d.%d\n",
			common->lmac_ver.major, common->lmac_ver.minor,
			common->lmac_ver.release_num);
	}
	rsi_dbg(ERR_ZONE, "Driver Version\t: %s", common->driver_ver);
	rsi_dbg(ERR_ZONE, "Operating mode\t: %d [%s]",
		common->oper_mode, opmode_str(common->oper_mode));
	rsi_dbg(ERR_ZONE, "Firmware file\t: %s", common->priv->fw_file_name);
	rsi_dbg(ERR_ZONE, "================================================\n");
}

/**
 * rsi_prepare_skb() - This function prepares the skb.
 * @common: Pointer to the driver private structure.
 * @buffer: Pointer to the packet data.
 * @pkt_len: Length of the packet.
 * @extended_desc: Extended descriptor.
 *
 * Return: Successfully skb.
 */
static struct sk_buff *rsi_prepare_skb(struct rsi_common *common,
				       u8 *buffer,
				       u32 pkt_len,
				       u8 extended_desc)
{
	struct ieee80211_tx_info *info;
	struct skb_info *rx_params;
	struct sk_buff *skb = NULL;
	u8 payload_offset;

	if (WARN(!pkt_len, "%s: Dummy pkt received", __func__))
		return NULL;

	if (pkt_len > (RSI_RCV_BUFFER_LEN * 4)) {
		rsi_dbg(ERR_ZONE, "%s: Pkt size > max rx buf size %d\n",
			__func__, pkt_len);
		pkt_len = RSI_RCV_BUFFER_LEN * 4;
	}

	pkt_len -= extended_desc;
	skb = dev_alloc_skb(pkt_len + FRAME_DESC_SZ);
	if (!skb)
		return NULL;

	payload_offset = (extended_desc + FRAME_DESC_SZ);
	skb_put(skb, pkt_len);
	memcpy((skb->data), (buffer + payload_offset), skb->len);

	info = IEEE80211_SKB_CB(skb);
	rx_params = (struct skb_info *)info->driver_data;
	rx_params->rssi = rsi_get_rssi(buffer);

	rx_params->channel = rsi_get_connected_channel(common->priv);

	return skb;
}

/**
 * rsi_read_pkt() - This function reads frames from the card.
 * @common: Pointer to the driver private structure.
 * @rcv_pkt_len: Received pkt length. In case of USB it is 0.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_read_pkt(struct rsi_common *common, u8 *rx_pkt, s32 rcv_pkt_len)
{
	u8 *frame_desc = NULL, extended_desc = 0;
	u32 index = 0, length = 0, queueno = 0;
	u16 actual_length = 0, offset;
	struct sk_buff *skb = NULL;
#if defined(CONFIG_RSI_COEX_MODE) && defined(CONFIG_RSI_ZIGB)
	struct rsi_mod_ops *zb_ops = g_proto_ops.zb_ops;
	u8 zb_pkt_type;
#endif

	do {
		frame_desc = &rx_pkt[index];
		actual_length = *(u16 *)&frame_desc[0];
		offset = *(u16 *)&frame_desc[2];

		queueno = rsi_get_queueno(frame_desc, offset);
		length = rsi_get_length(frame_desc, offset);

		if (queueno != RSI_ZIGB_Q) {
			if ((actual_length < (4 + FRAME_DESC_SZ)) || (offset < 4)) {
				rsi_dbg(ERR_ZONE,
					"%s: actual_length (%d) is less than 20 or"
					" offset(%d) is less than 4\n",
					__func__, actual_length, offset);
				break;
			}
		}
		if (queueno == RSI_WIFI_DATA_Q || queueno == RSI_WIFI_MGMT_Q)
			extended_desc = rsi_get_extended_desc(frame_desc,
							      offset);

		switch (queueno) {
		case RSI_COEX_Q:
			rsi_hex_dump(MGMT_RX_ZONE,
				     "RX Command co ex packet",
				     frame_desc + offset,
				     FRAME_DESC_SZ + length);
#ifdef CONFIG_RSI_COEX_MODE
			rsi_coex_recv_pkt(common, (frame_desc + offset));
#else
			rsi_mgmt_pkt_recv(common, (frame_desc + offset));
#endif
			break;
		case RSI_WIFI_DATA_Q:
			rsi_hex_dump(DATA_RX_ZONE,
				     "RX Data pkt",
				     frame_desc + offset,
				     FRAME_DESC_SZ + length);
			skb = rsi_prepare_skb(common,
					      (frame_desc + offset),
					      length,
					      extended_desc);
			if (!skb)
				goto fail;

			rsi_indicate_pkt_to_os(common, skb);
			break;

		case RSI_WIFI_MGMT_Q:
			rsi_mgmt_pkt_recv(common, (frame_desc + offset));
			break;
#if defined(CONFIG_RSI_BT_ALONE) || defined(CONFIG_RSI_COEX_MODE)
		case RSI_BT_MGMT_Q:
		case RSI_BT_DATA_Q:
			rsi_hex_dump(DATA_RX_ZONE,
				     "RX BT Pkt",
				     frame_desc + offset,
				     FRAME_DESC_SZ + length);
			rsi_hci_recv_pkt(common, frame_desc + offset);
			break;
#endif

#if defined(CONFIG_RSI_COEX_MODE) && defined(CONFIG_RSI_ZIGB)
		case RSI_ZIGB_Q:
			rsi_hex_dump(DATA_RX_ZONE,
					"RX ZB Pkt",
					frame_desc + offset,
					FRAME_DESC_SZ + length);
#define ZB_RX_PKT_TYPE_OFST	15
#define ZB_CARD_READY_IND	0xff
			zb_pkt_type = frame_desc[offset + ZB_RX_PKT_TYPE_OFST];
			if ((zb_pkt_type == ZB_CARD_READY_IND) &&
			    (common->zb_fsm_state == ZB_DEVICE_NOT_READY)) {
				rsi_dbg(INFO_ZONE, "ZIGB Card ready recvd\n");
				common->zb_fsm_state = ZB_DEVICE_READY;
				if (zb_ops && zb_ops->attach) {
					if (zb_ops->attach(common,
							   &g_proto_ops))
						rsi_dbg(ERR_ZONE,
							"Failed to attach ZIGB module\n");
				}
			} else {
				if ((common->zb_fsm_state == ZB_DEVICE_READY) &&
				    zb_ops && zb_ops->recv_pkt)
					zb_ops->recv_pkt(common,
							 frame_desc + offset);
			}
			break;
#endif

		default:
			rsi_dbg(ERR_ZONE, "%s: pkt from invalid queue: %d\n",
				__func__,   queueno);
			goto fail;
		}

		index  += actual_length;
		rcv_pkt_len -= actual_length;
	} while (rcv_pkt_len > 0);

	return 0;
fail:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(rsi_read_pkt);

/**
 * rsi_tx_scheduler_thread() - This function is a kernel thread to send the
 *			       packets to the device.
 * @common: Pointer to the driver private structure.
 *
 * Return: None.
 */
static void rsi_tx_scheduler_thread(struct rsi_common *common)
{
	int status = 0;
	struct rsi_hw *adapter = common->priv;
	u32 timeout = EVENT_WAIT_FOREVER;

	do {
		if (adapter->determine_event_timeout)
			timeout = adapter->determine_event_timeout(adapter);
		status = rsi_wait_event(&common->tx_thread.event, timeout);
		if (status < 0)
			break;
		rsi_reset_event(&common->tx_thread.event);

		if (common->init_done) {
			status = set_clr_tx_intention(common, WLAN_ID, 1);
			if (!status)
				rsi_core_qos_processor(common);
			else
				rsi_dbg(ERR_ZONE,
					"%s,%d:  Failed to get tx_access\n",
					__func__, __LINE__);
			set_clr_tx_intention(common, WLAN_ID, 0);
		}
	} while (atomic_read(&common->tx_thread.thread_done) == 0);
	complete_and_exit(&common->tx_thread.completion, 0);
}

#ifdef CONFIG_SDIO_INTR_POLL
void rsi_sdio_intr_poll_scheduler_thread(struct rsi_common *common)
{
        struct rsi_hw *adapter = common->priv;
        int status = 0;

        do {
                status = adapter->check_intr_status_reg(adapter);
                if (adapter->isr_pending)
                        adapter->isr_pending = 0;
                msleep(20);

        } while (atomic_read(&common->sdio_intr_poll_thread.thread_done) == 0);
        complete_and_exit(&common->sdio_intr_poll_thread.completion, 0);
}

void init_sdio_intr_status_poll_thread(struct rsi_common *common)
{
	rsi_init_event(&common->sdio_intr_poll_thread.event);
	if (rsi_create_kthread(common,
			       &common->sdio_intr_poll_thread,
			       rsi_sdio_intr_poll_scheduler_thread,
			       "Sdio Intr poll-Thread")) {
		rsi_dbg(ERR_ZONE, "%s: Unable to init sdio intr poll thrd\n",
				__func__);
	}
}
EXPORT_SYMBOL_GPL(init_sdio_intr_status_poll_thread);
#endif

#ifdef CONFIG_RSI_COEX_MODE
enum host_intf rsi_get_host_intf(void *priv)
{
	struct rsi_common *common = (struct rsi_common *)priv;

	return common->priv->rsi_host_intf;
}

void rsi_set_zb_context(void *priv, void *zb_context)
{
	struct rsi_common *common = (struct rsi_common *)priv;

	common->zb_adapter = zb_context;
}

void *rsi_get_zb_context(void *priv)
{
	struct rsi_common *common = (struct rsi_common *)priv;

	return common->zb_adapter;
}
#endif

/**
 * rsi_91x_init() - This function initializes os interface operations.
 * @void: Void.
 *
 * Return: Pointer to the adapter structure on success, NULL on failure.
 */
struct rsi_hw *rsi_91x_init(void)
{
	struct rsi_hw *adapter = NULL;
	struct rsi_common *common = NULL;
	u8 ii = 0;

	adapter = kzalloc(sizeof(*adapter), GFP_KERNEL);
	if (!adapter)
		return NULL;

	adapter->priv = kzalloc(sizeof(*common), GFP_KERNEL);
	if (!adapter->priv) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of priv\n",
			__func__);
		kfree(adapter);
		return NULL;
	}
	common = adapter->priv;
	common->priv = adapter;

	for (ii = 0; ii < NUM_SOFT_QUEUES; ii++)
		skb_queue_head_init(&common->tx_queue[ii]);

#ifdef CONFIG_RSI_11K
	skb_queue_head_init(&common->rrm_queue);
#endif
	rsi_init_event(&common->tx_thread.event);
	mutex_init(&common->mutex);
	mutex_init(&common->tx_lock);
	mutex_init(&common->rx_lock);
	sema_init(&common->tx_bus_lock, 1);
	sema_init(&common->tx_access_lock, 1);
	init_waitqueue_head(&common->techs[WLAN_ID].tx_access_event);
	init_waitqueue_head(&common->techs[COMMON_ID].tx_access_event);
#ifdef CONFIG_RSI_COEX_MODE
	init_waitqueue_head(&common->techs[BT_ZB_ID].tx_access_event);
#endif
	common->scan_workqueue =
		create_singlethread_workqueue("rsi_scan_worker");
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 20, 17)
	rsi_init_event(&common->chan_set_event);
	rsi_init_event(&common->probe_cfm_event);
	rsi_init_event(&common->chan_change_event);
#ifndef CONFIG_RSI_P2P
	rsi_init_event(&common->mgmt_cfm_event);
#endif
	INIT_WORK(&common->scan_work, rsi_scan_start);
#endif
	INIT_WORK(&common->scan_complete_work, rsi_scan_complete);
	rsi_init_event(&common->cancel_hw_scan_event);
#ifdef CONFIG_RSI_MULTI_MODE
	common->dev_oper_mode[0] = dev_oper_mode_count;
	memcpy(&common->dev_oper_mode[1], &dev_oper_mode, 5 * sizeof(u16));
#else
	common->dev_oper_mode = dev_oper_mode;
#endif
	common->lp_ps_handshake_mode = lp_handshake_mode;
	common->ulp_ps_handshake_mode = ulp_handshake_mode;
	common->peer_dist = peer_dist;
	common->bt_feature_bitmap = bt_feature_bitmap;
	common->uart_debug = uart_debug;
	common->ext_opt = ext_opt;
	common->ble_roles = ble_roles;
	common->bt_bdr_mode = bt_bdr_mode;
	common->three_wire_coex = three_wire_coex;
	common->anchor_point_gap = anchor_point_gap;
	common->host_intf_on_demand = host_intf_on_demand;
	common->crystal_as_sleep_clk = crystal_as_sleep_clk;
	common->feature_bitmap_9116 = feature_bitmap_9116;
	common->host_intf_on_demand = host_intf_on_demand;
	common->bt_rf_type = bt_rf_type;
	common->obm_ant_sel_val = antenna_sel;
	common->antenna_diversity = antenna_diversity;
	common->ble_tx_pwr_inx = ble_tx_pwr_inx;
	common->ble_pwr_save_options = ble_pwr_save_options;
	common->bt_rf_tx_power_mode = bt_rf_tx_power_mode;
	common->bt_rf_rx_power_mode = bt_rf_rx_power_mode; 
	common->w9116_features.ps_options = pwr_save_opt;
	common->wlan_pwrsave_options = ((lmac_bcon_drop << 5) |
					(standby_assoc_chain_sel << 4));
	common->driver_mode = driver_mode_value;
	common->enable_40mhz_in_2g = enable_40mhz_in_2g;
	common->enabled_uapsd = enabled_uapsd;
	common->max_sp_len = max_sp_len;

	if (rsi_create_kthread(common,
			       &common->tx_thread,
			       rsi_tx_scheduler_thread,
			       "Tx-Thread")) {
		rsi_dbg(ERR_ZONE, "%s: Unable to init tx thrd\n", __func__);
		goto err;
	}

#ifdef CONFIG_RSI_COEX_MODE
	if (rsi_coex_init(common)) {
		rsi_dbg(ERR_ZONE, "Failed to init COEX module\n");
		goto err;
	}
#endif
	/* Power save related */
	rsi_default_ps_params(adapter);
	spin_lock_init(&adapter->ps_lock);
	common->uapsd_bitmap = 0;
	common->rsi_scan_count = 0;

	/* BGScan related */
	init_bgscan_params(common);

	/* Wi-Fi direct related */
#if LINUX_VERSION_CODE < KERNEL_VERSION (4, 15, 0)
	common->roc_timer.data = (unsigned long)common;
	common->roc_timer.function = (void *)&rsi_roc_timeout;
	init_timer(&common->roc_timer);
#else
	timer_setup(&common->roc_timer, rsi_roc_timeout, 0);
#endif
	init_completion(&common->wlan_init_completion);

	common->init_done = true;
	return adapter;

err:
	kfree(common);
	kfree(adapter);
	return NULL;
}
EXPORT_SYMBOL_GPL(rsi_91x_init);

/**
 * rsi_91x_deinit() - This function de-intializes os intf operations.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: None.
 */
void rsi_91x_deinit(struct rsi_hw *adapter)
{
	struct rsi_common *common = adapter->priv;
	u8 ii;

	rsi_dbg(INFO_ZONE, "%s: Deinit core module...\n", __func__);

	flush_workqueue(common->scan_workqueue);
	destroy_workqueue(common->scan_workqueue);
	rsi_kill_thread(&common->tx_thread);

	for (ii = 0; ii < NUM_SOFT_QUEUES; ii++)
		skb_queue_purge(&common->tx_queue[ii]);

#ifdef CONFIG_RSI_COEX_MODE
	if (common->coex_mode > 1) {
#ifdef CONFIG_RSI_ZIGB
		if ((common->zb_fsm_state == ZB_DEVICE_READY) &&
		    g_proto_ops.zb_ops->detach) {
			rsi_dbg(INFO_ZONE,
				"%s: Detaching the zigbee module\n", __func__);
			g_proto_ops.zb_ops->detach(common);
		}
#endif
	}
	rsi_coex_deinit(common);
#endif
#ifdef CONFIG_RSI_MULTI_MODE
	rsi_dbg(ERR_ZONE, "%s: reset drv instance: %d\n",
			__func__, adapter->drv_instance_index);
	DRV_INSTANCE_SET(adapter->drv_instance_index, 0);
#endif
	common->init_done = false;
#if defined(USE_GPIO_HANDSHAKE)
	if (common->ulp_ps_handshake_mode == GPIO_HAND_SHAKE)
		gpio_deinit();
#endif

	kfree(common);
	kfree(adapter->rsi_dev);
	kfree(adapter);
}
EXPORT_SYMBOL_GPL(rsi_91x_deinit);

/**
 * rsi_91x_hal_module_init() - This function is invoked when the module is
 *			       loaded into the kernel.
 *			       It registers the client driver.
 * @void: Void.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_91x_hal_module_init(void)
{
	rsi_dbg(INIT_ZONE, "%s: Module init called\n", __func__);
#if defined(CONFIG_RSI_COEX_MODE) && defined(CONFIG_RSI_ZIGB)
	g_proto_ops.zb_ops = rsi_get_zb_ops();
	if (!g_proto_ops.zb_ops)
		rsi_dbg(ERR_ZONE, "Failed to get ZIGB ops\n");
#endif

	return 0;
}

/**
 * rsi_91x_hal_module_exit() - This function is called at the time of
 *			       removing/unloading the module.
 *			       It unregisters the client driver.
 * @void: Void.
 *
 * Return: None.
 */
static void rsi_91x_hal_module_exit(void)
{
	rsi_dbg(INIT_ZONE, "%s: Module exit called\n", __func__);
}

module_init(rsi_91x_hal_module_init);
module_exit(rsi_91x_hal_module_exit);
MODULE_AUTHOR("Redpine Signals Inc");
MODULE_DESCRIPTION("Station driver for RSI 91x devices");
MODULE_SUPPORTED_DEVICE("RSI-91x");
MODULE_VERSION(DRV_VER);
MODULE_LICENSE("Dual BSD/GPL");
