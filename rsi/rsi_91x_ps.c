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

#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/version.h>
#include "rsi_debugfs.h"
#include "rsi_mgmt.h"
#include "rsi_common.h"
#include "rsi_ps.h"
#include "rsi_gpio.h"

/**
 * str_psstate() - return the ps state in string format.
 *
 * @state - PS state.
 *
 * return: PS state in string format.
 */
char *str_psstate(enum ps_state state)
{
	switch (state) {
	case PS_NONE:
		return "PS_NONE";
	case PS_DISABLE_REQ_SENT:
		return "PS_DISABLE_REQ_SENT";
	case PS_ENABLE_REQ_SENT:
		return "PS_ENABLE_REQ_SENT";
	case PS_ENABLED:
		return "PS_ENABLED";
	default:
		return "INVALID_STATE";
	}
	return "INVALID_STATE";
}

/**
 * rsi_modify_ps_state() - Modify PS state to a new state.
 *
 * @adapter: pointer to rsi_hw structure.
 * @nstate: new PS state.
 *
 * return: new state.
 */
static inline void rsi_modify_ps_state(struct rsi_hw *adapter,
				       enum ps_state nstate)
{
	rsi_dbg(INFO_ZONE, "PS state changed %s => %s\n",
		str_psstate(adapter->ps_state),
		str_psstate(nstate));

	adapter->ps_state = nstate;
}

/**
 * rsi_default_ps_params() - Initalization of default powersave parameters.
 *
 * @adapter: pointer to rsi_hw structure.
 *
 * return: void.
 */
void rsi_default_ps_params(struct rsi_hw *adapter)
{
	struct rsi_ps_info *ps_info = &adapter->ps_info;

	ps_info->enabled = true;
	ps_info->sleep_type = ps_sleep_type;
	ps_info->tx_threshold = 0;
	ps_info->rx_threshold = 0;
	ps_info->tx_hysterisis = 0;
	ps_info->rx_hysterisis = 0;
	ps_info->monitor_interval = 0;
	ps_info->listen_interval = 0;
	ps_info->num_bcns_per_lis_int = 0;
	ps_info->dtim_interval_duration = 1;
	ps_info->num_dtims_per_sleep = 0;
	ps_info->deep_sleep_wakeup_period = 100;
	ps_info->uapsd_wakeup_period = RSI_UAPSD_WAKEUP_PERIOD;
}
EXPORT_SYMBOL_GPL(rsi_default_ps_params);

/**
 * rsi_enable_ps() - enable power save
 *
 * @adapter: Pointer to rsi_hw structure.
 *
 * return: void.
 */
void rsi_enable_ps(struct rsi_hw *adapter)
{
	if (rsi_send_ps_request(adapter, true)) {
		rsi_dbg(ERR_ZONE,
			"%s: Failed to send PS request to device\n",
			__func__);
		return;
	}

	rsi_modify_ps_state(adapter, PS_ENABLE_REQ_SENT);
}

/**
 * rsi_disable_ps() - disable power save
 *
 * @adapter: Pointer to rsi_hw structure.
 *
 * return: void.
 */
void rsi_disable_ps(struct rsi_hw *adapter)
{
	if (rsi_send_ps_request(adapter, false)) {
		rsi_dbg(ERR_ZONE,
			"%s: Failed to send PS request to device\n",
			__func__);
		return;
	}

	rsi_modify_ps_state(adapter, PS_DISABLE_REQ_SENT);
}

/**
 * rsi_conf_uapsd() - configures UAPSD powersave.
 *
 * @adapter - Pointer to rsi_hw structure.
 *
 * return: void.
 */
void rsi_conf_uapsd(struct rsi_hw *adapter)
{
	if (adapter->ps_state != PS_ENABLED)
		return;

	if (rsi_send_ps_request(adapter, false)) {
		rsi_dbg(ERR_ZONE,
			"%s: Failed to send PS request to device\n",
			__func__);
		return;
	}

	if (rsi_send_ps_request(adapter, true)) {
		rsi_dbg(ERR_ZONE,
			"%s: Failed to send PS request to device\n",
			__func__);
	}
}

/**
 * rsi_handle_ps_confirm() - Processes powersave confirmation.
 *
 * @adapter - Pointer to rsi_hw structure.
 * @msg - Recevied buffer.
 *
 * return: 0 on success.
 */
int rsi_handle_ps_confirm(struct rsi_hw *adapter, u8 *msg)
{
	u16 cfm_type = 0;

	cfm_type = *(u16 *)&msg[PS_CONFIRM_INDEX];

	switch (cfm_type) {
	case SLEEP_REQUEST:
		if (adapter->ps_state == PS_ENABLE_REQ_SENT)
			rsi_modify_ps_state(adapter, PS_ENABLED);
		break;
	case WAKEUP_REQUEST:
		if (adapter->ps_state == PS_DISABLE_REQ_SENT)
			rsi_modify_ps_state(adapter, PS_NONE);
		break;
	default:
		rsi_dbg(ERR_ZONE,
			"Invalid PS confirm type %x in state %s\n",
			cfm_type, str_psstate(adapter->ps_state));
		return -1;
	}

	return 0;
}

int protocol_tx_access(struct rsi_common *common)
{
	u8 ii = 0;

	for (ii = 0; ii < MAX_IDS; ii++) {
		if (common->techs[ii].tx_intention)
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL(protocol_tx_access);

void sleep_exit_recvd(struct rsi_common *common)
{
	u8 proto_id = 0;

	down(&common->tx_access_lock);
	common->common_hal_tx_access = true;
	up(&common->tx_access_lock);
	while (proto_id < MAX_IDS) {
		if (common->techs[proto_id].wait_for_tx_access) {
			common->techs[proto_id].wait_for_tx_access = 0;
			rsi_dbg(INFO_ZONE,
				"Waking up Event for proto %d\n", proto_id);
			wake_up(&common->techs[proto_id].tx_access_event);
		}
		proto_id++;
	}
}
EXPORT_SYMBOL(sleep_exit_recvd);

#if defined(USE_GPIO_HANDSHAKE)
#define MAX_RETRY_LIMIT 30
/*
 * process_tx_gpio_hand_shake() - This function performs GPIO handshake
 * between host driver and LMAC to grant tx_access for host.
 *
 * @proto_id: ID of the protocol that calls the function.
 * @set: Used to set/reset tx_intention for host.
 *
 * Return: None.
 */
int process_tx_gpio_handshake(struct rsi_common *common, u8 proto_id, u8 set)
{
	struct wireless_techs *tech;
	int retry_count = 0;

	tech = &common->techs[proto_id];
	if (!set) {
		tech->tx_intention = false;
		/*
		 * Every thread sets their tx_intention bits indivdually before
		 * dequeuing their respective packets, once if any thread
		 * finishes its job it should clear its tx_intention bit, but
		 * it needs to check all the other threads tx_intention bits
		 * before pulling the host gpio down, because other threads
		 * still need the tx access.
		 */
		if (!protocol_tx_access(common)) {
			common->common_hal_tx_access = false;
			set_host_status(set);
		}
		return 0;
	}
	/* Set intention */
	set_host_status(set);
	tech->tx_intention = true;

retry_to_wake_lmac:
	if (!get_device_status()) {
		while (1) {
			msleep(1); // Device need some delay to wake up
			if (get_device_status()) {
				/*
				 * Check the GPIO status after some delay, just
				 * to confirm its stablity.
				 */
				msleep(2);
				if (get_device_status()) {
					common->common_hal_tx_access = true;
					break;
				}
				continue;
			}
			if (retry_count++ > MAX_RETRY_LIMIT) {
				rsi_dbg(ERR_ZONE, "%s: Retry limit crossed\n",
					__func__);
				return -ETIME;
			}
		}
	} else {
		msleep(2);
		if (get_device_status())
			common->common_hal_tx_access = true;
		else
			goto retry_to_wake_lmac;
	}
	return 0;
}
#endif
int set_clr_tx_intention(struct rsi_common *common, u8 proto_id, u8 set)
{
	struct wireless_techs *tech;
	struct rsi_hw *adapter;
	int status = 0;	

	adapter = common->priv;
	tech = &common->techs[proto_id];
	down(&common->tx_access_lock);
#if defined(USE_GPIO_HANDSHAKE)
	if (common->lp_ps_handshake_mode == GPIO_HAND_SHAKE ||
	    common->ulp_ps_handshake_mode == GPIO_HAND_SHAKE)
	{
		status =  process_tx_gpio_handshake(common, proto_id, set);
		up(&common->tx_access_lock);
		return status;
	}
#endif
	if (!set) {
		tech->tx_intention = 0;
		if (common->sleep_entry_received &&
		    !common->ulp_sleep_ack_sent) {
			if (!protocol_tx_access(common)) {
				common->common_hal_tx_access = false;
				rsi_send_ack_for_ulp_entry(common);
			}
		}
		up(&common->tx_access_lock);
	}else {
		tech->tx_intention = 1;
		if (adapter->rsi_host_intf == RSI_HOST_INTF_USB && 
				adapter->usb_intf_in_suspend) {
			rsi_dbg(INFO_ZONE,
					"Changing resume timeout as pkt in pending\n");
			mod_timer(&common->suspend_timer,
					msecs_to_jiffies(1) + jiffies);
		}
		up(&common->tx_access_lock);
		if (!common->common_hal_tx_access) {
			tech->wait_for_tx_access = true;
			if (wait_event_timeout(tech->tx_access_event,
						tech->wait_for_tx_access == 0,
						msecs_to_jiffies(6000))) {
				if (!common->common_hal_tx_access) {
					rsi_dbg(ERR_ZONE,
							"%s,%d: Unable to get tx_access\n",
							__func__, __LINE__);
					return -EPERM;
				}
			} else {
				rsi_dbg(ERR_ZONE, "%s,%d: Wait event failed\n",
						__func__, __LINE__);
				return -ETIME;
			}
		}
	}
	return status;
}
EXPORT_SYMBOL(set_clr_tx_intention);
