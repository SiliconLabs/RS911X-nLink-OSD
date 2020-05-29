/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived
 * 	   from this software without specific prior written permission.
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
#include "rsi_mgmt.h"
#include "rsi_common.h"
#include "rsi_ps.h"
#include "rsi_hal.h"
#ifdef CONFIG_RSI_COEX_MODE
#include "rsi_coex.h"
#endif
#ifdef CONFIG_RSI_11K
#include "rsi_rrm.h"
#endif

#if 0
static struct rsi_config_vals dev_config_vals[] = {
	{
		.lp_ps_handshake = 0,
		.ulp_ps_handshake = 0,
		.sleep_config_params = 0,
		.ext_pa_or_bt_coex_en = 0,
	},
};
#endif

#ifdef CONFIG_RSI_11K
struct reg_class reg_db[MAX_REGIONS][MAX_REG_CLASS] = { {
	{ 1, 20, {36, 40, 44, 48} },
	{ 2, 20, {52, 56, 60, 64} },
	{ 3, 20, {149, 153, 157, 161} },
	{ 4, 20, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140} },
	{ 5, 20, {149, 153, 157, 161, 165} },
	{ 6, 5, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10} },
	{ 7, 5, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10} },
	{ 8, 10, {11, 13, 15, 17, 19} },
	{ 9, 10, {11, 13, 15, 17, 19} },
	{ 10, 20, {21, 25} },
	{ 11, 20, {21, 25} },
	{ 12, 25, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11} },
	{ 13, 20, {133, 137} },
	{ 14, 10, {132, 134, 136, 138} },
	{ 15, 5, {131, 132, 133, 134, 135, 136, 137, 138} },
	{ 22, 40, {36, 44} },
	{ 23, 40, {52, 60} },
	{ 24, 40, {100, 108, 116, 124, 132} },
	{ 25, 40, {149, 157} },
	{ 26, 40, {149, 157} },
	{ 27, 40, {40, 48} },
	{ 28, 40, {56, 64} },
	{ 29, 40, {104, 112, 120, 128, 136} },
	{ 30, 40, {153, 161} },
	{ 31, 40, {153, 161} },
	{ 32, 40, {1, 2, 3, 4, 5, 6, 7} },
	{ 33, 40, {5, 6, 7, 8, 9, 10, 11 } }
	}, {
	{ 1, 20, {36, 40, 44, 48} },
	{ 2, 20, {52, 56, 60, 64} },
	{ 3, 20, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140} },
	{ 4, 25, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13} },
	{ 5, 40, {36, 44} },
	{ 6, 40, {52, 60} },
	{ 7, 40, {100, 108, 116, 124, 132} },
	{ 8, 40, {40, 48} },
	{ 9, 40, {56, 64} },
	{ 10, 40, {104, 112, 120, 128, 136} },
	{ 11, 40, {1, 2, 3, 4, 5, 6, 7, 8, 9} },
	{ 12, 40, {5, 6, 7, 8, 9, 10, 11, 12, 13} },
	{ 16, 20, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140} },
	{ 17, 20, {149, 153, 157, 161, 165, 169} }
	}, {
	{ 1, 20, {34, 38, 42, 46, 36, 40, 44, 48} },
	{ 2, 20, {8, 12, 16} },
	{ 3, 20, {8, 12, 16} },
	{ 4, 20, {8, 12, 16} },
	{ 5, 20, {8, 12, 16} },
	{ 6, 20, {8, 12, 16} },
	{ 7, 20, {184, 188, 192, 196} },
	{ 8, 20, {184, 188, 192, 196} },
	{ 9, 20, {184, 188, 192, 196} },
	{ 10, 20, {184, 188, 192, 196} },
	{ 11, 20, {184, 188, 192, 196} },
	{ 12, 10, {7, 8, 9, 11} },
	{ 13, 10, {7, 8, 9, 11} },
	{ 14, 10, {7, 8, 9, 11} },
	{ 15, 10, {7, 8, 9, 11} },
	{ 16, 10, {183, 184, 185, 187, 188, 189} },
	{ 17, 10, {183, 184, 185, 187, 188, 189} },
	{ 18, 10, {183, 184, 185, 187, 188, 189} },
	{ 19, 10, {183, 184, 185, 187, 188, 189} },
	{ 20, 10, {183, 184, 185, 187, 188, 189} },
	{ 21, 5, {6, 7, 8, 9, 10, 11} },
	{ 22, 5, {6, 7, 8, 9, 10, 11} },
	{ 23, 5, {6, 7, 8, 9, 10, 11} },
	{ 24, 5, {6, 7, 8, 9, 10, 11} },
	{ 25, 5, {182, 183, 184, 185, 186, 187, 188, 189} },
	{ 26, 5, {182, 183, 184, 185, 186, 187, 188, 189} },
	{ 27, 5, {182, 183, 184, 185, 186, 187, 188, 189} },
	{ 28, 5, {182, 183, 184, 185, 186, 187, 188, 189} },
	{ 29, 5, {182, 183, 184, 185, 186, 187, 188, 189} },
	{ 30, 25, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13} },
	{ 31, 25, {14} },
	{ 32, 20, {52, 56, 60, 64} },
	{ 33, 20, {52, 56, 60, 64} },
	{ 34, 20, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140} },
	{ 35, 20, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140} },
	{ 36, 40, {36, 44} },
	{ 37, 40, {52, 60} },
	{ 38, 40, {52, 60} },
	{ 39, 40, {100, 108, 116, 124, 132} },
	{ 40, 40, {100, 108, 116, 124, 132} },
	{ 41, 40, {40, 48} },
	{ 42, 40, {56, 64} },
	{ 43, 40, {56, 64} },
	{ 44, 40, {104, 112, 120, 128, 136} },
	{ 45, 40, {104, 112, 120, 128, 136} },
	{ 46, 40, {184, 192} },
	{ 47, 40, {184, 192} },
	{ 48, 40, {184, 192} },
	{ 49, 40, {184, 192} },
	{ 50, 40, {184, 192} },
	{ 51, 40, {188, 196} },
	{ 52, 40, {188, 196} },
	{ 53, 40, {188, 196} },
	{ 54, 40, {188, 196} },
	{ 55, 40, {188, 196} },
	{ 56, 40, {1, 2, 3, 4, 5, 6, 7, 8, 9} },
	{ 57, 40, {5, 6, 7, 8, 9, 10, 11, 12, 13} },
	{ 58, 20, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140} },
	}, {
	{ 81, 25, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13} },
	{ 82, 25, {14} },
	{ 83, 40, {1, 2, 3, 4, 5, 6, 7, 8, 9} },
	{ 84, 40, {5, 6, 7, 8, 9, 10, 11, 12, 13} },
	{ 94, 20, {133, 137} },
	{ 95, 10, {132, 134, 136, 138} },
	{ 96, 5, {131, 132, 133, 134, 135, 136, 137, 138} },
	{ 101, 20, {21, 25} },
	{ 102, 10, {11, 13, 15, 17, 19} },
	{ 103, 5, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10} },
	{ 104, 40, {184, 192} },
	{ 105, 40, {188, 196} },
	{ 106, 20, {191, 195} },
	{ 107, 10, {189, 191, 193, 195, 197} },
	{ 108, 5, {188, 189, 190, 191, 192, 193, 194, 195, 196, 197} },
	{ 109, 20, {184, 188, 192, 196} },
	{ 110, 10, {183, 184, 185, 186, 187, 188, 189} },
	{ 111, 5, {183, 184, 185, 186, 187, 188, 189} },
	{ 112, 20, {8, 12, 16} },
	{ 113, 10, {7, 8, 9, 10, 11} },
	{ 114, 5, {6, 7, 8, 9, 10, 11} },
	{ 115, 20, {36, 40, 44, 48} },
	{ 116, 40, {36, 44} },
	{ 117, 40, {40, 48} },
	{ 118, 20, {52, 56, 60, 64} },
	{ 119, 40, {52, 60}, },
	{ 120, 40, {56, 64}, },
	{ 121, 20, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140} },
	{ 122, 40, {100, 108, 116, 124, 132} },
	{ 123, 40, {104, 112, 120, 128, 136} },
	{ 124, 20, {149, 153, 157, 161} },
	{ 125, 20, {149, 153, 157, 161, 165, 169} },
	{ 126, 40, {149, 157} },
	{ 127, 40, {153, 161} }
	}
};
#endif

/* Bootup Parameters for 20MHz */
static struct bootup_params boot_params_20 = {
	.magic_number = cpu_to_le16(0x5aa5),
	.crystal_good_time = 0x0,
	.valid = cpu_to_le32(VALID_20),
	.reserved_for_valids = 0x0,
	.bootup_mode_info = 0x0,
	.digital_loop_back_params = 0x0,
	.rtls_timestamp_en = 0x0,
	.host_spi_intr_cfg = 0x0,
	.device_clk_info = {{
		/* WLAN params */
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_20 << 8) |
							 (TAPLL_M_VAL_20)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_20),
			},
			.pll960_info_g = {
				.pll_reg_1 =
					cpu_to_le16((PLL960_P_VAL_20 << 8) |
						    (PLL960_N_VAL_20)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_20),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x1,
			.switch_qspi_clk = 0x1,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x1,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = cpu_to_le16(0x111),
			.umac_clock_reg_config = cpu_to_le16(0x48),
			.qspi_uart_clock_reg_config = cpu_to_le16(0x1211)
		}
	},
	/* Bluetooth params */
	{
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_20 << 8) |
							 (TAPLL_M_VAL_20)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_20),
			},
			.pll960_info_g = {
				.pll_reg_1 =
					cpu_to_le16((PLL960_P_VAL_20 << 8) |
						    (PLL960_N_VAL_20)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_20),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x0,
			.switch_qspi_clk = 0x0,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x0,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = 0x0,
			.umac_clock_reg_config = 0x0,
			.qspi_uart_clock_reg_config = 0x0
		}
	},
	/* Zigbee params */
	{
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_20 << 8) |
							 (TAPLL_M_VAL_20)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_20),
			},
			.pll960_info_g = {
				.pll_reg_1 =
					cpu_to_le16((PLL960_P_VAL_20 << 8) |
						    (PLL960_N_VAL_20)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_20),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x0,
			.switch_qspi_clk = 0x0,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x0,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = 0x0,
			.umac_clock_reg_config = 0x0,
			.qspi_uart_clock_reg_config = 0x0
		}
	}
	},
	/* ULP Params */
	.buckboost_wakeup_cnt = 0x0,
	.pmu_wakeup_wait = 0x0,
	.shutdown_wait_time = 0x0,
	.pmu_slp_clkout_sel = 0x0,
	.wdt_prog_value = 0x0,
	.wdt_soc_rst_delay = 0x0,
	.dcdc_operation_mode = 0x0,
	.soc_reset_wait_cnt = 0x0,
	.waiting_time_at_fresh_sleep = 0x0,
	.max_threshold_to_avoid_sleep = 0x0,
	.beacon_resedue_alg_en = 0,
};

/* Bootup Parameters for 20MHz */
static struct bootup_params_9116 boot_params_9116_20 = {
	.magic_number = cpu_to_le16(0x5aa5),
	.crystal_good_time = 0x0,
	.valid = cpu_to_le32(VALID_20),
	.reserved_for_valids = 0x0,
	.bootup_mode_info = 0x0,
	.digital_loop_back_params = 0x0,
	.rtls_timestamp_en = 0x0,
	.host_spi_intr_cfg = 0x0,
	.device_clk_info_9116 = {{
		/* WLAN params */
		.pll_config_9116_g = {
			.pll_ctrl_set_reg = 0xd518,
			.pll_ctrl_clr_reg = 0x2ae7,
			.pll_modem_conig_reg = 0x2000,
			.soc_clk_config_reg = 0x0C18,
			.adc_dac_strm1_config_reg = 0x1100,
			.adc_dac_strm2_config_reg = 0x6600,
		},
		.switch_clk_9116_g = {
			.switch_clk_info =
				cpu_to_le32((RSI_SWITCH_TASS_CLK |
					    RSI_SWITCH_WLAN_BBP_LMAC_CLK_REG |
					    RSI_SWITCH_BBP_LMAC_CLK_REG)),
			.tass_clock_reg = 0x083C0503,
			.wlan_bbp_lmac_clk_reg_val = 0x01042001,
			.zbbt_bbp_lmac_clk_reg_val = 0x02010001,
			.bbp_lmac_clk_en_val = 0x0000003b,
		}
	},
	},
	/* ULP Params */
	.buckboost_wakeup_cnt = 0x0,
	.pmu_wakeup_wait = 0x0,
	.shutdown_wait_time = 0x0,
	.pmu_slp_clkout_sel = 0x0,
	.wdt_prog_value = 0x0,
	.wdt_soc_rst_delay = 0x0,
	.dcdc_operation_mode = 0x0,
	.soc_reset_wait_cnt = 0x0,
	.waiting_time_at_fresh_sleep = 0x0,
	.max_threshold_to_avoid_sleep = 0x0,
	.beacon_resedue_alg_en = 0,
};

/* Bootup parameters for 40MHz */
static struct bootup_params boot_params_40 = {
	.magic_number = cpu_to_le16(0x5aa5),
	.crystal_good_time = 0x0,
	.valid = cpu_to_le32(VALID_40),
	.reserved_for_valids = 0x0,
	.bootup_mode_info = 0x0,
	.digital_loop_back_params = 0x0,
	.rtls_timestamp_en = 0x0,
	.host_spi_intr_cfg = 0x0,
	.device_clk_info = {{
		/* WLAN params */
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_40 << 8) |
							 (TAPLL_M_VAL_40)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_40),
			},
			.pll960_info_g = {
				.pll_reg_1 =
					cpu_to_le16((PLL960_P_VAL_40 << 8) |
						    (PLL960_N_VAL_40)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_40),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x1,
			.switch_qspi_clk = 0x1,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x1,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = cpu_to_le16(0x1121),
			.umac_clock_reg_config = cpu_to_le16(0x48),
			.qspi_uart_clock_reg_config = cpu_to_le16(0x1211)
		}
	},
	/* Bluetooth Params */
	{
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_40 << 8) |
							 (TAPLL_M_VAL_40)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_40),
			},
			.pll960_info_g = {
				.pll_reg_1 =
					cpu_to_le16((PLL960_P_VAL_40 << 8) |
						    (PLL960_N_VAL_40)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_40),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x0,
			.switch_qspi_clk = 0x0,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x0,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = 0x0,
			.umac_clock_reg_config = 0x0,
			.qspi_uart_clock_reg_config = 0x0
		}
	},
	/* Zigbee Params */
	{
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_40 << 8) |
							 (TAPLL_M_VAL_40)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_40),
			},
			.pll960_info_g = {
				.pll_reg_1 =
					cpu_to_le16((PLL960_P_VAL_40 << 8) |
						    (PLL960_N_VAL_40)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_40),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x0,
			.switch_qspi_clk = 0x0,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x0,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = 0x0,
			.umac_clock_reg_config = 0x0,
			.qspi_uart_clock_reg_config = 0x0
		}
	}
	},
	/* ULP Params */
	.buckboost_wakeup_cnt = 0x0,
	.pmu_wakeup_wait = 0x0,
	.shutdown_wait_time = 0x0,
	.pmu_slp_clkout_sel = 0x0,
	.wdt_prog_value = 0x0,
	.wdt_soc_rst_delay = 0x0,
	.dcdc_operation_mode = 0x0,
	.soc_reset_wait_cnt = 0x0,
	.waiting_time_at_fresh_sleep = 0x0,
	.max_threshold_to_avoid_sleep = 0x0,
	.beacon_resedue_alg_en = 0,
};

/* Bootup parameters for 40MHz - 9116 */
static struct bootup_params_9116 boot_params_9116_40 = {
	.magic_number = cpu_to_le16(0x5aa5),
	.crystal_good_time = 0x0,
	.valid = cpu_to_le32(VALID_40),
	.reserved_for_valids = 0x0,
	.bootup_mode_info = 0x0,
	.digital_loop_back_params = 0x0,
	.rtls_timestamp_en = 0x0,
	.host_spi_intr_cfg = 0x0,
	.device_clk_info_9116 = {{
		/* WLAN params */
		.pll_config_9116_g = {
			.pll_ctrl_set_reg = 0xd518,
			.pll_ctrl_clr_reg = 0x2ae7,
			.pll_modem_conig_reg = 0x3000,
			.soc_clk_config_reg = 0x0C18,
			.adc_dac_strm1_config_reg = 0x0000,
			.adc_dac_strm2_config_reg = 0x6600,
		},
		.switch_clk_9116_g = {
			.switch_clk_info =
				cpu_to_le32((RSI_SWITCH_TASS_CLK |
					    RSI_SWITCH_WLAN_BBP_LMAC_CLK_REG |
					    RSI_SWITCH_BBP_LMAC_CLK_REG |
					    RSI_MODEM_CLK_160MHZ)),
			.tass_clock_reg = 0x083C0503,
#ifdef CONFIG_FPGA_40MHZ
			.wlan_bbp_lmac_clk_reg_val = 0x01081002,
#else
			.wlan_bbp_lmac_clk_reg_val = 0x01042002,
#endif
			.zbbt_bbp_lmac_clk_reg_val = 0x04010002,
			.bbp_lmac_clk_en_val = 0x0000003b,
		}
	},
	},
	/* ULP Params */
	.buckboost_wakeup_cnt = 0x0,
	.pmu_wakeup_wait = 0x0,
	.shutdown_wait_time = 0x0,
	.pmu_slp_clkout_sel = 0x0,
	.wdt_prog_value = 0x0,
	.wdt_soc_rst_delay = 0x0,
	.dcdc_operation_mode = 0x0,
	.soc_reset_wait_cnt = 0x0,
	.waiting_time_at_fresh_sleep = 0x0,
	.max_threshold_to_avoid_sleep = 0x0,
	.beacon_resedue_alg_en = 0,
};

#define UNUSED_GPIO	1
#define USED_GPIO	0
static struct rsi_ulp_gpio_vals unused_ulp_gpio_bitmap = {
	.motion_sensor_gpio_ulp_wakeup = UNUSED_GPIO,
	.sleep_ind_from_device = UNUSED_GPIO,
	.ulp_gpio_2 = UNUSED_GPIO,
	.push_button_ulp_wakeup = UNUSED_GPIO,
};

struct rsi_soc_gpio_vals unused_soc_gpio_bitmap = {
	.pspi_csn_0		= USED_GPIO,	//GPIO_0
	.pspi_csn_1		= USED_GPIO,	//GPIO_1
	.host_wakeup_intr	= USED_GPIO,	//GPIO_2
	.pspi_data_0		= USED_GPIO,	//GPIO_3
	.pspi_data_1		= USED_GPIO,	//GPIO_4
	.pspi_data_2		= USED_GPIO,	//GPIO_5
	.pspi_data_3		= USED_GPIO,	//GPIO_6
	.i2c_scl		= USED_GPIO,	//GPIO_7
	.i2c_sda		= USED_GPIO,	//GPIO_8
	.uart1_rx		= UNUSED_GPIO,	//GPIO_9
	.uart1_tx		= UNUSED_GPIO,	//GPIO_10
	.uart1_rts_i2s_clk	= UNUSED_GPIO,	//GPIO_11
	.uart1_cts_i2s_ws	= UNUSED_GPIO,	//GPIO_12
	.dbg_uart_rx_i2s_din	= UNUSED_GPIO,	//GPIO_13
	.dbg_uart_tx_i2s_dout	= UNUSED_GPIO,	//GPIO_14
	.lp_wakeup_boot_bypass	= UNUSED_GPIO,	//GPIO_15
	.led_0			= USED_GPIO,	//GPIO_16
	.btcoex_wlan_active_ext_pa_ant_sel_A = UNUSED_GPIO, //GPIO_17
	.btcoex_bt_priority_ext_pa_ant_sel_B = UNUSED_GPIO, //GPIO_18
	.btcoex_bt_active_ext_pa_on_off = UNUSED_GPIO, //GPIO_19
	.rf_reset		= USED_GPIO, //GPIO_20
	.sleep_ind_from_device	= UNUSED_GPIO,
};

static u16 mcs[] = {13, 26, 39, 52, 78, 104, 117, 130};

#ifdef CONFIG_RSI_11K
int ieee80211_chan_to_bw(struct rsi_hw *adapter, u8 op_class, u8 channel_num)
{
	int i, j;
	u8 region = adapter->dfs_region;

	for (i = 0; i < ARRAY_SIZE(reg_db[region - 1]); i++) {
		if (reg_db[region - 1][i].op_class == op_class) {
			for (j = 0; j < ARRAY_SIZE(reg_db[region - 1][i].chans);
					j++) {
				if (reg_db[region - 1][i].chans[j] ==
				    channel_num)
					return reg_db[region - 1][i].bandwidth;
			}
		}
	}
	return -1;
}

int is_dfs_channel(struct rsi_hw *adapter, int channel)
{
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *ch;
	struct wiphy *wiphy = adapter->hw->wiphy;
	int i;

	sband = wiphy->bands[NL80211_BAND_5GHZ];
	for (i = 0; i < sband->n_channels; i++) {
		ch = &sband->channels[i];
		if (ch->flags & IEEE80211_CHAN_DISABLED)
			continue;
		if ((ch->hw_value == channel) &&
		    (ch->flags & IEEE80211_CHAN_RADAR))
			return 1;
	}
	return 0;
}
#endif

/**
 * rsi_set_default_parameters() - This function sets default parameters.
 * @common: Pointer to the driver private structure.
 *
 * Return: none
 */
static void rsi_set_default_parameters(struct rsi_common *common)
{
	common->band = NL80211_BAND_2GHZ;
	common->channel_width = BW_20MHZ;
	common->rts_threshold = IEEE80211_MAX_RTS_THRESHOLD;
	common->channel = 1;
	common->min_rate = 0xffff;
	common->fsm_state = FSM_CARD_NOT_READY;
	common->iface_down = true;
	common->endpoint = EP_2GHZ_20MHZ;
	common->ta_aggr = 3;
	common->skip_fw_load = 0; /* Default disable skipping fw loading */
	common->rf_power_val = 0; /* Default 1.9V */
	common->device_gpio_type = TA_GPIO; /* Default TA GPIO */
	common->country_code = 840; /* Default US */
	common->wlan_rf_power_mode = 0;
	common->bt_rf_power_mode = 0;
	common->tx_power = RSI_TXPOWER_MAX;
	common->dtim_cnt = 2;
	common->beacon_interval = 100;
	common->antenna_gain[0] = 0;
	common->antenna_gain[1] = 0;
	common->w9116_features.pll_mode = 0x0 ;
	common->w9116_features.rf_type = 1;
	common->w9116_features.wireless_mode = 0;
	common->w9116_features.enable_ppe = 0;
	common->w9116_features.afe_type = 1;
	common->w9116_features.dpd = 0;
	common->w9116_features.sifs_tx_enable = 0;
}

void init_bgscan_params(struct rsi_common *common)
{
	common->bgscan_info.bgscan_threshold = 10;
	common->bgscan_info.roam_threshold = 10;
	common->bgscan_info.bgscan_periodicity = 30;
	common->bgscan_info.num_bg_channels = 0;
	common->bgscan_info.two_probe = 1;
	common->bgscan_info.active_scan_duration = 20;
	common->bgscan_info.passive_scan_duration = 70;
	common->bgscan_info.channels2scan[0] = 1;
	common->bgscan_info.channels2scan[1] = 2;
	common->bgscan_info.channels2scan[2] = 3;
	common->bgscan_info.channels2scan[3] = 4;
	common->bgscan_info.channels2scan[4] = 5;
	common->bgscan_info.channels2scan[5] = 6;
	common->bgscan_info.channels2scan[6] = 7;
	common->bgscan_info.channels2scan[7] = 8;
	common->bgscan_info.channels2scan[8] = 9;
	common->bgscan_info.channels2scan[9] = 10;
	common->bgscan_info.channels2scan[10] = 11;
//	common->bgscan_info.channels2scan[11] = 12;
//	common->bgscan_info.channels2scan[12] = 13;
//	common->bgscan_info.channels2scan[13] = 14;
#if 0
	common->bgscan_info.channels2scan[11] = 36;
	common->bgscan_info.channels2scan[12] = 40;
	common->bgscan_info.channels2scan[13] = 44;
	common->bgscan_info.channels2scan[14] = 48;
	common->bgscan_info.channels2scan[15] = 52;
	common->bgscan_info.channels2scan[16] = 56;
	common->bgscan_info.channels2scan[17] = 60;
	common->bgscan_info.channels2scan[18] = 64;
	common->bgscan_info.channels2scan[19] = 100;
	common->bgscan_info.channels2scan[20] = 104;
#endif
}

/**
 * rsi_set_contention_vals() - This function sets the contention values for the
 *			       backoff procedure.
 * @common: Pointer to the driver private structure.
 *
 * Return: None.
 */
static void rsi_set_contention_vals(struct rsi_common *common)
{
	u8 ii = 0;

	for (; ii < NUM_EDCA_QUEUES; ii++) {
		common->tx_qinfo[ii].wme_params =
			(((common->edca_params[ii].cw_min / 2) +
			  (common->edca_params[ii].aifs)) *
			  WMM_SHORT_SLOT_TIME + SIFS_DURATION);
		common->tx_qinfo[ii].weight = common->tx_qinfo[ii].wme_params;
		common->tx_qinfo[ii].pkt_contended = 0;
	}
}

/**
 * rsi_send_internal_mgmt_frame() - This function sends management frames to
 *				    firmware.Also schedules packet to queue
 *				    for transmission.
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_send_internal_mgmt_frame(struct rsi_common *common,
					struct sk_buff *skb)
{
	struct skb_info *tx_params;

	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: SKB is NULL\n", __func__);
		return -EINVAL;
	}
	skb->data[1] |= BIT(7);
	tx_params = (struct skb_info *)&IEEE80211_SKB_CB(skb)->driver_data;
	tx_params->flags |= INTERNAL_MGMT_PKT;
	skb->priority = MGMT_SOFT_Q;
	if (skb->data[2] == PEER_NOTIFY)
		skb_queue_head(&common->tx_queue[MGMT_SOFT_Q], skb);
	else
		skb_queue_tail(&common->tx_queue[MGMT_SOFT_Q], skb);
	rsi_set_event(&common->tx_thread.event);
	return 0;
}

/**
 * rsi_load_radio_caps() - This function is used to send radio capabilities
 *			   values to firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding negative error code on failure.
 */
int rsi_load_radio_caps(struct rsi_common *common)
{
	struct rsi_radio_caps *radio_caps;
	struct rsi_hw *adapter = common->priv;
	u16 inx = 0;
	u16 value;
	u8 ii, jj = 0;
	u8 radio_id = 0;
	u16 gc[20] = {0xf0, 0xf0, 0xf0, 0xf0,
		      0xf0, 0xf0, 0xf0, 0xf0,
		      0xf0, 0xf0, 0xf0, 0xf0,
		      0xf0, 0xf0, 0xf0, 0xf0,
		      0xf0, 0xf0, 0xf0, 0xf0};
	struct sk_buff *skb;

	rsi_dbg(INT_MGMT_ZONE, "<===== SENDING RADIO_CAPABILITIES =====>\n");

	skb = dev_alloc_skb(sizeof(struct rsi_radio_caps));
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, sizeof(struct rsi_radio_caps));
	radio_caps = (struct rsi_radio_caps *)skb->data;

	radio_caps->desc_word[1] = cpu_to_le16(RADIO_CAPABILITIES);
	radio_caps->desc_word[4] = cpu_to_le16(common->channel);
	radio_caps->desc_word[4] |= cpu_to_le16(RSI_RF_TYPE << 8);

	radio_caps->desc_word[7] |= cpu_to_le16(RSI_LMAC_CLOCK_80MHZ);
	if (common->channel_width == BW_40MHZ) {
		radio_caps->desc_word[7] |= cpu_to_le16(RSI_ENABLE_40MHZ);

		if (common->fsm_state == FSM_MAC_INIT_DONE) {
			struct ieee80211_hw *hw = adapter->hw;
			struct ieee80211_conf *conf = &hw->conf;

			if (conf_is_ht40_plus(conf)) {
				radio_caps->desc_word[5] =
					cpu_to_le16(LOWER_20_ENABLE);
				radio_caps->desc_word[5] |=
					cpu_to_le16(LOWER_20_ENABLE >> 12);
			} else if (conf_is_ht40_minus(conf)) {
				radio_caps->desc_word[5] =
					cpu_to_le16(UPPER_20_ENABLE);
				radio_caps->desc_word[5] |=
					cpu_to_le16(UPPER_20_ENABLE >> 12);
			} else {
				radio_caps->desc_word[5] =
					cpu_to_le16(BW_40MHZ << 12);
				radio_caps->desc_word[5] |=
					cpu_to_le16(FULL40M_ENABLE);
			}
		}
	}

	if (adapter->device_model == RSI_DEV_9116) {
		if (common->channel_width == BW_20MHZ) {
			value = le16_to_cpu(radio_caps->desc_word[7]);
			value = (value & (~0x3));
			radio_caps->desc_word[7] = cpu_to_le16(value);
		}
	}

	radio_caps->sifs_tx_11n = cpu_to_le16(SIFS_TX_11N_VALUE);
	radio_caps->sifs_tx_11b = cpu_to_le16(SIFS_TX_11B_VALUE);
	radio_caps->slot_rx_11n = cpu_to_le16(SHORT_SLOT_VALUE);
	radio_caps->ofdm_ack_tout = cpu_to_le16(OFDM_ACK_TOUT_VALUE);
	radio_caps->cck_ack_tout = cpu_to_le16(CCK_ACK_TOUT_VALUE);
	radio_caps->preamble_type = cpu_to_le16(LONG_PREAMBLE);

	radio_caps->desc_word[7] |= cpu_to_le16(radio_id << 8);
	for (ii = 0; ii < (MAX_HW_QUEUES - 4); ii++) {
		if (ii > 5)
			jj = 1;
		radio_caps->qos_params[ii].cont_win_min_q =
			cpu_to_le16(common->edca_params[ii%4].cw_min + jj);
		radio_caps->qos_params[ii].cont_win_max_q =
			cpu_to_le16(common->edca_params[ii%4].cw_max + jj);
		radio_caps->qos_params[ii].aifsn_val_q =
			cpu_to_le16(common->edca_params[ii%4].aifs);
		radio_caps->qos_params[ii].txop_q =
			cpu_to_le16(common->edca_params[ii%4].txop << 5);
	}

	for (ii = 0; ii < MAX_HW_QUEUES; ii++) {
		if (!(radio_caps->qos_params[ii].cont_win_min_q)) {
			radio_caps->qos_params[ii].cont_win_min_q =
							cpu_to_le16(7);
			radio_caps->qos_params[ii].cont_win_max_q =
							cpu_to_le16(0x3f);
			radio_caps->qos_params[ii].aifsn_val_q = cpu_to_le16(2);
			radio_caps->qos_params[ii].txop_q = 0;
		}
	}
	radio_caps->qos_params[BROADCAST_HW_Q].txop_q = 0xffff;
	radio_caps->qos_params[MGMT_HW_Q].txop_q = 0;
	radio_caps->qos_params[BEACON_HW_Q].txop_q = 0xffff;

	memcpy(&common->rate_pwr[0], &gc[0], 40);
	for (ii = 0; ii < 20; ii++)
		radio_caps->gcpd_per_rate[inx++] =
			cpu_to_le16(common->rate_pwr[ii]  & 0x00FF);

	radio_caps->desc_word[0] = cpu_to_le16((sizeof(struct rsi_radio_caps) -
						FRAME_DESC_SZ) |
						(RSI_WIFI_MGMT_Q << 12));

	skb_put(skb, (sizeof(struct rsi_radio_caps)));
	rsi_hex_dump(INT_MGMT_ZONE, "RADIO-CAP FRAME", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_mgmt_pkt_to_core() - This function is the entry point for Mgmt module.
 * @common: Pointer to the driver private structure.
 * @msg: Pointer to received packet.
 * @msg_len: Length of the received packet.
 * @type: Type of received packet.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_mgmt_pkt_to_core(struct rsi_common *common,
				u8 *msg,
				s32 msg_len)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_tx_info *info;
	struct skb_info *rx_params;
	u8 pad_bytes = msg[4];
	u8 pkt_recv;
	struct sk_buff *skb;
	char *buffer;
	struct ieee80211_hdr *wlh;
#ifdef CONFIG_RSI_11K
	u8 *data;
#endif
	if (!adapter->sc_nvifs)
		return -ENOLINK;

	if (common->iface_down)
		return -ENODEV;

	msg_len -= pad_bytes;
	if ((msg_len <= 0) || (!msg)) {
		rsi_dbg(MGMT_RX_ZONE,
			"%s: Invalid rx msg of len = %d\n",
			__func__, msg_len);
		return -EINVAL;
	}
#ifdef CONFIG_RSI_11K
	wlh = (struct ieee80211_hdr *)&msg[FRAME_DESC_SZ + pad_bytes];
	rsi_hex_dump(ERR_ZONE, "RX RRM packet", (u8 *)wlh, msg_len);

	if (ieee80211_is_action(wlh->frame_control)) {
		data = (u8 *)wlh + MIN_802_11_HDR_LEN;
		if (data[0] == WLAN_ACTION_RADIO_MEASUREMENT) {
			rsi_dbg(MGMT_RX_ZONE,
				"%s: Received Radio Measurement Request\n",
				__func__);
			if (!rsi_rrm_parse_radio_action_frame(common,
							      (u8 *)wlh,
							      msg_len))
				return 0;
		} else if (data[0] == WLAN_ACTION_SPECTRUM_MANAGEMENT) {
			rsi_dbg(MGMT_RX_ZONE,
				"%s: Received spectrum Measurement Request\n",
				__func__);
			rsi_rrm_parse_spectrum_action_frame(common, wlh,
							    data);
				return 0;
		} else {
			rsi_dbg(MGMT_RX_ZONE,
				"%s: Received invalid category code\n",
				__func__);
			return 0;
		}
	}
#endif
	skb = dev_alloc_skb(msg_len);
	if (!skb)
		return -ENOMEM;

	buffer = skb_put(skb, msg_len);

	memcpy(buffer,
		(u8 *)(msg +  FRAME_DESC_SZ + pad_bytes),
		msg_len);

	pkt_recv = buffer[0];

	info = IEEE80211_SKB_CB(skb);
	rx_params = (struct skb_info *)info->driver_data;
	rx_params->rssi = rsi_get_rssi(msg);
	rx_params->channel = rsi_get_channel(msg);
	rsi_dbg(MGMT_RX_ZONE,
		"%s: rssi=%d channel=%d\n",
		__func__, rx_params->rssi, rx_params->channel);
	wlh = (struct ieee80211_hdr *)skb->data;
	rsi_dbg(MGMT_RX_ZONE, "RX Dot11 Mgmt Pkt Type: %s\n",
		dot11_pkt_type(wlh->frame_control));
	if (ieee80211_is_auth(wlh->frame_control)) {
		rsi_dbg(MGMT_DEBUG_ZONE,
			"<==== Recvd AUTHENTICATION Packet ====>\n");
		rsi_hex_dump(MGMT_DEBUG_ZONE, "AUTH-FRAME",
					      skb->data, skb->len);
	} else if (ieee80211_is_assoc_resp(wlh->frame_control)) {
		rsi_dbg(MGMT_DEBUG_ZONE,
			"<==== Recvd ASSOCIATION RESPONSE Packet ====>\n");
		rsi_hex_dump(MGMT_DEBUG_ZONE, "ASSOC-RESP",
					       skb->data, skb->len);
	} else if (ieee80211_is_deauth(wlh->frame_control)) {
		rsi_dbg(MGMT_DEBUG_ZONE,
			"<==== Recvd DE-AUTH Packet ====>\n");
		rsi_hex_dump(MGMT_DEBUG_ZONE, "DE-AUTH FRAME",
					      skb->data, skb->len);
	}

	rsi_indicate_pkt_to_os(common, skb);

	return 0;
}

/**
 * rsi_send_sta_notify_frame() - This function sends the station notify
 *				     frame to firmware.
 * @common: Pointer to the driver private structure.
 * @opmode: Operating mode of device.
 * @notify_event: Notification about station connection.
 * @bssid: bssid.
 * @qos_enable: Qos is enabled.
 * @aid: Aid (unique for all STA).
 *
 * Return: status: 0 on success, corresponding negative error code on failure.
 */
int rsi_send_sta_notify_frame(struct rsi_common *common,
			      enum opmode opmode,
			      u8 notify_event,
			      const unsigned char *bssid,
			      u8 qos_enable,
			      u16 aid,
			      u16 sta_id)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_vif *vif = adapter->vifs[adapter->sc_nvifs - 1];
	struct sk_buff *skb = NULL;
	struct rsi_peer_notify *peer_notify;
	int status;
	u16 vap_id = 0;
	int frame_len = sizeof(*peer_notify);

	rsi_dbg(INT_MGMT_ZONE,
		"<==== Sending Peer Notify Packet ====>\n");

	skb = dev_alloc_skb(frame_len);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}
	memset(skb->data, 0, frame_len);

	peer_notify = (struct rsi_peer_notify *)skb->data;

	if (opmode == STA_OPMODE)
		peer_notify->command = cpu_to_le16(PEER_TYPE_AP << 1);
	else if (opmode == AP_OPMODE)
		peer_notify->command = cpu_to_le16(PEER_TYPE_STA << 1);

	switch (notify_event) {
	case STA_CONNECTED:
		peer_notify->command |= cpu_to_le16(RSI_ADD_PEER);
		break;
	case STA_DISCONNECTED:
		peer_notify->command |= cpu_to_le16(RSI_DELETE_PEER);
		break;
	default:
		break;
	}
	adapter->peer_notify = true;
	peer_notify->command |= cpu_to_le16((aid & 0xfff) << 4);
	ether_addr_copy(peer_notify->mac_addr, bssid);
	peer_notify->mpdu_density = cpu_to_le16(0x08); //FIXME check this
	peer_notify->sta_flags = cpu_to_le32((qos_enable) ? 1 : 0);
	peer_notify->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
						(RSI_WIFI_MGMT_Q << 12));
	peer_notify->desc_word[1] = cpu_to_le16(PEER_NOTIFY);
	peer_notify->desc_word[7] |= cpu_to_le16(sta_id | vap_id << 8);

	skb_put(skb, frame_len);
	if (notify_event == STA_CONNECTED) {
		rsi_hex_dump(INT_MGMT_ZONE, "ADD PEER FRAME",
						skb->data, skb->len);
	} else {
		rsi_hex_dump(INT_MGMT_ZONE, "DELETE PEER FRAME",
						skb->data, skb->len);
	}
	status = rsi_send_internal_mgmt_frame(common, skb);

	if ((vif->type == NL80211_IFTYPE_STATION) &&
	    (!status) && qos_enable) {
		rsi_set_contention_vals(common);
		mdelay(1);
		status = rsi_load_radio_caps(common);
	}

	return status;
}

/**
 * rsi_send_aggr_params_frame() - This function sends the ampdu
 *					 indication frame to firmware.
 * @common: Pointer to the driver private structure.
 * @tid: traffic identifier.
 * @ssn: ssn.
 * @buf_size: buffer size.
 * @event: notification about station connection.
 *
 * Return: 0 on success, corresponding negative error code on failure.
 */
int rsi_send_aggr_params_frame(struct rsi_common *common,
			       u16 tid,
			       u16 ssn,
			       u8 buf_size,
			       u8 event,
			       u8 sta_id)
{
	struct sk_buff *skb = NULL;
	struct rsi_mac_frame *mgmt_frame;
	u8 peer_id = 0;
//	u8 window_size = 1;

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	rsi_dbg(INT_MGMT_ZONE,
		"<===== SENDING AMPDU_IND FRAME =====>\n");

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(AMPDU_IND);

	if (event == STA_TX_ADDBA_DONE) {
		mgmt_frame->desc_word[4] = cpu_to_le16(ssn);
		mgmt_frame->desc_word[5] = cpu_to_le16(buf_size);
		//mgmt_frame->desc_word[5] = cpu_to_le16(window_size);
		mgmt_frame->desc_word[7] =
			cpu_to_le16((tid |
				    (START_AMPDU_AGGR << 4) |
				    (peer_id << 8)));
	} else if (event == STA_RX_ADDBA_DONE) {
		mgmt_frame->desc_word[4] = cpu_to_le16(ssn);
		mgmt_frame->desc_word[7] = cpu_to_le16(tid |
						       (START_AMPDU_AGGR << 4) |
						       (RX_BA_INDICATION << 5) |
						       (peer_id << 8));
	} else if (event == STA_TX_DELBA) {
		mgmt_frame->desc_word[7] = cpu_to_le16(tid |
						       (STOP_AMPDU_AGGR << 4) |
						       (peer_id << 8));
	} else if (event == STA_RX_DELBA) {
		mgmt_frame->desc_word[7] = cpu_to_le16(tid |
						       (STOP_AMPDU_AGGR << 4) |
						       (RX_BA_INDICATION << 5) |
						       (peer_id << 8));
	}

	skb_put(skb, FRAME_DESC_SZ);
	rsi_hex_dump(INT_MGMT_ZONE, "AMPDU_IND FRAME", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_program_bb_rf() - This function starts base band and RF programming.
 *			 This is called after initial configurations are done.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding negative error code on failure.
 */
int rsi_program_bb_rf(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct rsi_mac_frame *mgmt_frame;

	rsi_dbg(INT_MGMT_ZONE, "<===== SENDING BBP_PROG_IN_TA FRAME =====>\n");

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(BBP_PROG_IN_TA);
	mgmt_frame->desc_word[4] = cpu_to_le16(common->endpoint);
	mgmt_frame->desc_word[3] = cpu_to_le16(common->rf_pwr_mode);

	if (common->rf_reset) {
		mgmt_frame->desc_word[7] =  cpu_to_le16(RF_RESET_ENABLE);
		rsi_dbg(MGMT_TX_ZONE, "%s: ===> RF RESET REQUEST SENT <===\n",
			__func__);
		common->rf_reset = 0;
	}
	common->bb_rf_prog_count = 1;
	mgmt_frame->desc_word[7] |= cpu_to_le16(PUT_BBP_RESET |
				     BBP_REG_WRITE | (RSI_RF_TYPE << 4));

	skb_put(skb, FRAME_DESC_SZ);
	rsi_hex_dump(INT_MGMT_ZONE, "BBP_PROG_IN_TA", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_set_vap_capabilities() - This function send vap capability to firmware.
 * @common: Pointer to the driver private structure.
 * @opmode: Operating mode of device.
 *
 * Return: 0 on success, corresponding negative error code on failure.
 */
int rsi_set_vap_capabilities(struct rsi_common *common,
			     enum opmode mode,
			     u8 *mac_addr,
			     u8 vap_id,
			     u8 vap_status)
{
	struct sk_buff *skb = NULL;
	struct rsi_vap_caps *vap_caps;
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hw *hw = adapter->hw;
	struct ieee80211_conf *conf = &hw->conf;

	rsi_dbg(INT_MGMT_ZONE,
		"<===== SENDING VAP_CAPABILITIES FRAME =====>\n");

	rsi_dbg(INFO_ZONE, "Config VAP: id=%d mode=%d status=%d ",
		vap_id, mode, vap_status);
	rsi_hex_dump(INFO_ZONE, "mac", mac_addr, 6);
	skb = dev_alloc_skb(sizeof(struct rsi_vap_caps));
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, sizeof(struct rsi_vap_caps));
	vap_caps = (struct rsi_vap_caps *)skb->data;

	vap_caps->desc_word[0] = cpu_to_le16((sizeof(struct rsi_vap_caps) -
					     FRAME_DESC_SZ) |
					     (RSI_WIFI_MGMT_Q << 12));
	vap_caps->desc_word[1] = cpu_to_le16(VAP_CAPABILITIES);
	vap_caps->desc_word[2] = cpu_to_le16(vap_status << 8);
	vap_caps->desc_word[4] = cpu_to_le16(mode |
					     (common->channel_width << 8));
	vap_caps->desc_word[7] = cpu_to_le16((vap_id << 8) |
					     (common->mac_id << 4) |
					     common->radio_id);

	memcpy(vap_caps->mac_addr, mac_addr, IEEE80211_ADDR_LEN);
	vap_caps->keep_alive_period = cpu_to_le16(90);
	vap_caps->frag_threshold = cpu_to_le16(IEEE80211_MAX_FRAG_THRESHOLD);

	vap_caps->rts_threshold = cpu_to_le16(common->rts_threshold);
	vap_caps->default_mgmt_rate = cpu_to_le32(RSI_RATE_6);

	if (common->band == NL80211_BAND_5GHZ) {
		vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_6);
	} else {
		if (common->p2p_enabled) {
			vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_6);
		} else {
		vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_1);
	}
	}

	if (conf_is_ht40(conf)) {
		if (conf_is_ht40_minus(conf))
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(UPPER_20_ENABLE << 16);
		else if (conf_is_ht40_plus(conf))
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(LOWER_20_ENABLE << 16);
		else
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(FULL40M_ENABLE << 16);
	}

	vap_caps->default_data_rate = 0;
	vap_caps->beacon_interval = cpu_to_le16(common->beacon_interval);
	vap_caps->dtim_period = cpu_to_le16(common->dtim_cnt);
	vap_caps->beacon_miss_threshold = cpu_to_le16(HW_BMISS_THRESHOLD);

	skb_put(skb, sizeof(*vap_caps));
	rsi_hex_dump(INT_MGMT_ZONE, "VAP-CAP FRAME", skb->data, skb->len);

	common->last_vap_type = mode;
	ether_addr_copy(common->last_vap_addr, mac_addr);
	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_load_key() - This function is used to load keys within the firmware.
 * @common: Pointer to the driver private structure.
 * @data: Pointer to the key data.
 * @key_len: Key length to be loaded.
 * @key_type: Type of key: GROUP/PAIRWISE.
 * @key_id: Key index.
 * @cipher: Type of cipher used.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_load_key(struct rsi_common *common,
		 u8 *data,
		 u16 key_len,
		 u8 key_type,
		 u8 key_id,
		 u32 cipher,
		 s16 sta_id)
{
	struct ieee80211_vif *vif = common->priv->vifs[common->priv->sc_nvifs - 1];
	struct sk_buff *skb = NULL;
	struct rsi_set_key *set_key;
	u16 key_descriptor = 0;
	u8 key_t1 = 0;
	u8 vap_id = 0;

	skb = dev_alloc_skb(sizeof(struct rsi_set_key));
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, sizeof(struct rsi_set_key));
	set_key = (struct rsi_set_key *)skb->data;

	switch (key_type) {
	case RSI_GROUP_KEY:
		key_t1 = 1 << 1;
		if ((vif->type == NL80211_IFTYPE_AP) ||
		    (vif->type == NL80211_IFTYPE_P2P_GO))
			key_descriptor = BIT(7);
		break;
	case RSI_PAIRWISE_KEY:
		if (((vif->type == NL80211_IFTYPE_AP) ||
		     (vif->type == NL80211_IFTYPE_P2P_GO)) &&
		    (sta_id >= common->max_stations)) {
			rsi_dbg(INFO_ZONE, "Invalid Sta_id %d\n", sta_id);
			return -1;
		}
		key_t1 = 0 << 1;
		if ((cipher != WLAN_CIPHER_SUITE_WEP40) &&
		    (cipher != WLAN_CIPHER_SUITE_WEP104))
			key_id = 0;
		break;
	}
	if ((cipher == WLAN_CIPHER_SUITE_WEP40) ||
	    (cipher == WLAN_CIPHER_SUITE_WEP104)) {
		key_descriptor |= BIT(2);
		if (key_len >= 13) {
			key_descriptor |= BIT(3);
		}
	} else if (cipher != KEY_TYPE_CLEAR) {
		key_descriptor |= BIT(4);
		if (cipher == WLAN_CIPHER_SUITE_TKIP)
			key_descriptor |= BIT(5);
	}
	key_descriptor |= (key_t1 | BIT(13) | (key_id << 14));

	set_key->desc_word[0] = cpu_to_le16((sizeof(struct rsi_set_key) -
					    FRAME_DESC_SZ) |
					    (RSI_WIFI_MGMT_Q << 12));
	set_key->desc_word[1] = cpu_to_le16(SET_KEY_REQ);
	set_key->desc_word[4] = cpu_to_le16(key_descriptor);
	set_key->desc_word[7] = cpu_to_le16(sta_id | (vap_id << 8));

	if (data) {
		if ((cipher == WLAN_CIPHER_SUITE_WEP40) ||
		    (cipher == WLAN_CIPHER_SUITE_WEP104)) {
			memcpy(&set_key->key[key_id][1], data, key_len * 2);
		} else {
			memcpy(&set_key->key[0][0], data, key_len);
		}
		memcpy(set_key->tx_mic_key, &data[16], 8);
		memcpy(set_key->rx_mic_key, &data[24], 8);
	} else {
		memset(&set_key[FRAME_DESC_SZ], 0,
		       sizeof(struct rsi_set_key) - FRAME_DESC_SZ);				
	}

	skb_put(skb, sizeof(struct rsi_set_key));
	rsi_hex_dump(INT_MGMT_ZONE, "KEY FRAME", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_common_dev_params() - This function send the common device
 *				configuration parameters to device.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_send_common_dev_params(struct rsi_common *common)
{
	struct sk_buff *skb = NULL;
	u32 frame_len = 0;
	struct rsi_config_vals *dev_cfgs = NULL;

	frame_len = sizeof(struct rsi_config_vals);

	rsi_dbg(INT_MGMT_ZONE,
		"<==== Sending common device config params =====>\n");
	skb = dev_alloc_skb(frame_len);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Unable to allocate skb\n", __func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, frame_len);

	dev_cfgs = (struct rsi_config_vals *)&skb->data[0];
	memset(dev_cfgs, 0, (sizeof(struct rsi_config_vals)));

	dev_cfgs->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
					     (RSI_COEX_Q << 12));
	dev_cfgs->desc_word[1] = cpu_to_le16(COMMON_DEV_CONFIG);

	dev_cfgs->lp_ps_handshake = common->lp_ps_handshake_mode;
	dev_cfgs->ulp_ps_handshake = common->ulp_ps_handshake_mode;

	dev_cfgs->unused_ulp_gpio = *(u8 *)&unused_ulp_gpio_bitmap;
	dev_cfgs->unused_soc_gpio_bitmap =
		cpu_to_le32(*(u32 *)&unused_soc_gpio_bitmap);

	/*
	 * For zigb functionality, send DEV_OPMODE_ZB_ALONE
	 * as opermode to firmware, irrespective of router or
	 * coordinator
	 */
	if (common->oper_mode == DEV_OPMODE_ZB_COORDINATOR ||
	    common->oper_mode == DEV_OPMODE_ZB_ROUTER)
		dev_cfgs->opermode = DEV_OPMODE_ZB_ALONE;
	else
		dev_cfgs->opermode = common->oper_mode;

	dev_cfgs->wlan_rf_pwr_mode = common->wlan_rf_power_mode;
	dev_cfgs->driver_mode = common->driver_mode;
	dev_cfgs->region_code = NL80211_DFS_FCC;
	dev_cfgs->antenna_sel_val = common->obm_ant_sel_val;
	dev_cfgs->dev_peer_dist = common->peer_dist;
	dev_cfgs->dev_bt_feature_bitmap = common->bt_feature_bitmap;
	dev_cfgs->uart_dbg = common->uart_debug;
	if (common->priv->device_model == RSI_DEV_9116) {
		/*
		 * In 9116_feature_bitmap, BITS(3:0) are used for module type
		 * selection, BIT(4) is used for host interface on demand
		 * feature option. BIT(5) is used for selecting sleep clock
		 * source. All variables below are module param of 2 byte
		 * size.
		 */
		dev_cfgs->features_9116 = (common->ext_opt & 0xF) |
					  (common->host_intf_on_demand << 4) |
					  (common->crystal_as_sleep_clk << 5) |
					  (common->feature_bitmap_9116 << 11);
		dev_cfgs->dev_ble_roles = common->ble_roles;
		/* In bt_bdr, bt_bdr_mode used a byte[0:7],
		 * three_wire_coex use one bit [8]. Module param of 2 byte
		 * size.
		 */
		dev_cfgs->bt_bdr = ((common->three_wire_coex << 8) |
				    common->bt_bdr_mode);
		dev_cfgs->dev_anchor_point_gap = common->anchor_point_gap;
	}

	skb_put(skb, frame_len);
	rsi_hex_dump(INT_MGMT_ZONE, "COMMON-DEV PARAM", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/*
 * rsi_load_bootup_params() - This function send bootup params to the firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
static int rsi_load_bootup_params(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct rsi_boot_params *boot_params = NULL;
	struct rsi_boot_params_9116 *boot_params_9116 = NULL;
	struct rsi_hw *adapter = common->priv;

	rsi_dbg(INT_MGMT_ZONE,
		"<===== SENDING BOOTUP_PARAMS_REQUEST FRAME =====>\n");

	if (adapter->device_model == RSI_DEV_9116)
		skb = dev_alloc_skb(sizeof(struct rsi_boot_params_9116));
	else
		skb = dev_alloc_skb(sizeof(struct rsi_boot_params));

	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}
	if (adapter->device_model == RSI_DEV_9116) {
		memset(skb->data, 0, sizeof(struct rsi_boot_params_9116));
		boot_params_9116 = (struct rsi_boot_params_9116 *)skb->data;
	} else {
		memset(skb->data, 0, sizeof(struct rsi_boot_params));
		boot_params = (struct rsi_boot_params *)skb->data;
	}

	if (common->channel_width == BW_40MHZ) {
		if (adapter->device_model == RSI_DEV_9116) {
			memcpy(&boot_params_9116->bootup_params,
			       &boot_params_9116_40,
			       sizeof(struct bootup_params_9116));
			rsi_dbg(INT_MGMT_ZONE,
				"%s: Packet 40MHZ UMAC_CLK =%d\n", __func__,
				UMAC_CLK_40BW);
			boot_params_9116->desc_word[7] =
				cpu_to_le16(UMAC_CLK_40BW);
		} else {
			memcpy(&boot_params->bootup_params,
			       &boot_params_40,
			       sizeof(struct bootup_params));
			rsi_dbg(INT_MGMT_ZONE,
				"%s: Packet 40MHZ UMAC_CLK = %d\n", __func__,
				UMAC_CLK_40BW);
			boot_params->desc_word[7] = cpu_to_le16(UMAC_CLK_40BW);
		}
	} else {
		if (adapter->device_model == RSI_DEV_9116)
			memcpy(&boot_params_9116->bootup_params,
			       &boot_params_9116_20,
			       sizeof(struct bootup_params_9116));
		else
			memcpy(&boot_params->bootup_params,
			       &boot_params_20,
			       sizeof(struct bootup_params));

		if (adapter->device_model == RSI_DEV_9116) {
			if (boot_params_9116_20.valid !=
					cpu_to_le32(VALID_20)) {
				boot_params_9116->desc_word[7] =
					cpu_to_le16(UMAC_CLK_20BW);
				rsi_dbg(INT_MGMT_ZONE,
					"Packet 20MHZ WITH UMAC_CLK = %d\n",
					 UMAC_CLK_20BW);
			} else {
				boot_params_9116->desc_word[7] =
					cpu_to_le16(UMAC_CLK_40MHZ);
				rsi_dbg(INT_MGMT_ZONE,
					"Packet 20MHZ WITH UMAC_CLK = %d\n",
					UMAC_CLK_40MHZ);
			}
		} else {
			if (boot_params_20.valid != cpu_to_le32(VALID_20)) {
				boot_params->desc_word[7] =
					cpu_to_le16(UMAC_CLK_20BW);
				rsi_dbg(INT_MGMT_ZONE,
					"Packet 20MHZ WITH UMAC_CLK = %d\n",
					UMAC_CLK_20BW);
			} else {
				boot_params->desc_word[7] =
					cpu_to_le16(UMAC_CLK_40MHZ);
				rsi_dbg(INT_MGMT_ZONE,
					"Packet 20MHZ WITH UMAC_CLK = %d\n",
					UMAC_CLK_40MHZ);
			}
		}
	}

	/**
	 * Bit{0:11} indicates length of the Packet
	 * Bit{12:15} indicates host queue number
	 */
	if (adapter->device_model == RSI_DEV_9116) {
		boot_params_9116->desc_word[0] =
			cpu_to_le16(sizeof(struct bootup_params_9116) |
				    (RSI_WIFI_MGMT_Q << 12));
		boot_params_9116->desc_word[1] =
			cpu_to_le16(BOOTUP_PARAMS_REQUEST);

		skb_put(skb, sizeof(struct rsi_boot_params_9116));

		rsi_dbg(MGMT_TX_ZONE, "%s: Boot params- %p\n",
			__func__, boot_params_9116);
	} else {
		boot_params->desc_word[0] =
			cpu_to_le16(sizeof(struct bootup_params) |
				    (RSI_WIFI_MGMT_Q << 12));
		boot_params->desc_word[1] =
			cpu_to_le16(BOOTUP_PARAMS_REQUEST);
		skb_put(skb, sizeof(struct rsi_boot_params));
		rsi_dbg(MGMT_TX_ZONE, "%s: Boot params- %p\n",
			__func__, boot_params);
	}
	rsi_hex_dump(INT_MGMT_ZONE, "BOOT-UP PARAM", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_reset_mac() - This function prepares reset MAC request and sends an
 *			  internal management frame to indicate it to firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
static int rsi_send_reset_mac(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct rsi_mac_frame *mgmt_frame;

	rsi_dbg(INT_MGMT_ZONE,
		"<===== SENDING RESET_MAC_REQUEST FRAME =====>\n");

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(RESET_MAC_REQ);
	mgmt_frame->desc_word[4] |= cpu_to_le16(RETRY_COUNT << 8);

	/*TA level aggregation of pkts to host */
	if (common->driver_mode == SNIFFER_MODE) {
		mgmt_frame->desc_word[3] = cpu_to_le16(SNIFFER_ENABLE);
		mgmt_frame->desc_word[3] |=  cpu_to_le16(DSBL_TA_AGGR << 8);
	} else {
		mgmt_frame->desc_word[3] |=  common->ta_aggr << 8;
	}

	if (common->antenna_diversity)
		mgmt_frame->desc_word[6] = common->antenna_diversity;

	skb_put(skb, FRAME_DESC_SZ);
	rsi_hex_dump(INT_MGMT_ZONE, "RESET_MAC FRAME", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_band_check() - This function programs the band
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_band_check(struct rsi_common *common,
		   struct ieee80211_channel *curchan)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hw *hw = adapter->hw;
	u8 prev_bw = common->channel_width;
	u8 prev_ep = common->endpoint;
	int status = 0;

	if (common->band != curchan->band) {
		common->rf_reset = 1;
		common->band = curchan->band;
	}

	if ((hw->conf.chandef.width == NL80211_CHAN_WIDTH_20_NOHT) ||
	    (hw->conf.chandef.width == NL80211_CHAN_WIDTH_20))
		common->channel_width = BW_20MHZ;
	else
		common->channel_width = BW_40MHZ;

	if (common->band == NL80211_BAND_2GHZ) {
		if (common->channel_width)
			common->endpoint = EP_2GHZ_40MHZ;
		else
			common->endpoint = EP_2GHZ_20MHZ;
	} else {
		if (common->channel_width)
			common->endpoint = EP_5GHZ_40MHZ;
		else
			common->endpoint = EP_5GHZ_20MHZ;
	}

	if (common->endpoint != prev_ep) {
		status = rsi_program_bb_rf(common);
		if (status)
			return status;
	}

	if (common->channel_width != prev_bw) {
		status = rsi_load_bootup_params(common);
		if (status)
			return status;

		status = rsi_load_radio_caps(common);
		if (status)
			return status;
	}

	return status;
}

#ifdef CONFIG_CARACALLA_BOARD
void rsi_apply_carcalla_power_values(struct rsi_hw *adapter,
				     struct ieee80211_vif *vif,
				     struct ieee80211_channel *channel)
{
	u16 max_power = 20;

	if (!conf_is_ht(&adapter->hw->conf)) {
		if (vif->bss_conf.basic_rates == 0xf) {
			if (channel->hw_value == 12)
				max_power = 15;
			else if (channel->hw_value == 13)
				max_power = 7;
			else
				return;
		} else {
			if (channel->hw_value == 12)
				max_power = 8; 
			else if (channel->hw_value == 13)
				max_power = 7; 
			else
				return;
		}
	} else if (conf_is_ht20(&adapter->hw->conf)) {
		if (channel->hw_value == 12)
			max_power = 7; 
		else if (channel->hw_value == 13)
			max_power = 5;
		else
			return;
	} else {
		if (channel->hw_value == 6)
			max_power = 9; 
		else if (channel->hw_value == 9)
			max_power = 5; 
		else if (channel->hw_value == 10)
			max_power = 4;
		else
			return;
	}
	channel->max_power = max_power;
	channel->max_antenna_gain = 0;
}
#endif

/**
 * rsi_set_channel() - This function programs the channel.
 * @common: Pointer to the driver private structure.
 * @channel: Channel value to be set.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_set_channel(struct rsi_common *common,
		    struct ieee80211_channel *channel)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_vif *vif = adapter->vifs[adapter->sc_nvifs - 1];
	struct sk_buff *skb = NULL;
	struct rsi_mac_frame *mgmt_frame;

	rsi_dbg(INT_MGMT_ZONE,
		"<===== SENDING SCAN_REQUEST FRAME =====>\n");

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	if (!channel) {
		dev_kfree_skb(skb);
		return 0;
	}
	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(SCAN_REQUEST);
	mgmt_frame->desc_word[4] = cpu_to_le16(channel->hw_value);

#if 0
	channel->max_antenna_gain = common->antenna_gain;
	mgmt_frame->desc_word[4] |=
		cpu_to_le16(((char)(channel->max_antenna_gain)) << 8);
	mgmt_frame->desc_word[5] =
		cpu_to_le16((char)(channel->max_antenna_gain));
#endif
	if (vif->type == NL80211_IFTYPE_AP && adapter->auto_chan_sel) {
		mgmt_frame->desc_word[2] = cpu_to_le16(TIMER_ENABLE);
		mgmt_frame->desc_word[3] = cpu_to_le16(ACS_TIMEOUT_TYPE);
		mgmt_frame->desc_word[3] |= cpu_to_le16(ACS_TIMEOUT_TIME << 8);
		adapter->rsi_survey[adapter->idx].channel = channel;
	}
	mgmt_frame->desc_word[4] |=
		cpu_to_le16(common->antenna_gain[0] << 8);
	mgmt_frame->desc_word[5] =
		cpu_to_le16(common->antenna_gain[1]);

	mgmt_frame->desc_word[7] = cpu_to_le16(PUT_BBP_RESET |
					       BBP_REG_WRITE |
					       (RSI_RF_TYPE << 4));

	if ((channel->flags & IEEE80211_CHAN_NO_IR) ||
	    (channel->flags & IEEE80211_CHAN_RADAR)) {
		mgmt_frame->desc_word[4] |= BIT(15);
	} else {
		if (common->tx_power < channel->max_power)
			mgmt_frame->desc_word[6] =
				cpu_to_le16(common->tx_power);
		else
			mgmt_frame->desc_word[6] =
				cpu_to_le16(channel->max_power);
	}
	if (common->driver_mode == SNIFFER_MODE)
		mgmt_frame->desc_word[6] = cpu_to_le16(TX_PWR_IN_SNIFFER_MODE);
#ifdef CONFIG_CARACALLA_BOARD
	rsi_apply_carcalla_power_values(adapter, vif, channel);
	mgmt_frame->desc_word[6] = cpu_to_le16(channel->max_power);

	if ((channel->hw_value == 12) || (channel->hw_value == 13))
		mgmt_frame->desc_word[7] = cpu_to_le16(TARGET_BOARD_CARACALLA);
	if (conf_is_ht40(&adapter->hw->conf)) {
		if ((channel->hw_value == 6) ||
		    (channel->hw_value == 9) ||
		    (channel->hw_value == 10))
		mgmt_frame->desc_word[7] = cpu_to_le16(TARGET_BOARD_CARACALLA);
	}
#endif
	rsi_dbg(INFO_ZONE, "reg_domain = %d, TX power = %d\n",
		adapter->dfs_region, mgmt_frame->desc_word[6]);
	mgmt_frame->desc_word[7] |= cpu_to_le16(adapter->dfs_region);

	if (common->channel_width == BW_40MHZ)
		mgmt_frame->desc_word[5] |= cpu_to_le16(0x1 << 8);

	common->channel = channel->hw_value;

	skb_put(skb, FRAME_DESC_SZ);
	rsi_hex_dump(INT_MGMT_ZONE, "SCAN-REQ FRAME", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_radio_params_update() - This function sends the radio
 *				parameters update to device
 * @common: Pointer to the driver private structure.
 * @channel: Channel value to be set.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_send_radio_params_update(struct rsi_common *common)
{
	struct rsi_mac_frame *mgmt_frame;
	struct sk_buff *skb = NULL;

	rsi_dbg(INT_MGMT_ZONE,
		"<===== Sending Radio Params update frame ====>\n");

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(RADIO_PARAMS_UPDATE);
	mgmt_frame->desc_word[3] = cpu_to_le16(BIT(0));

	mgmt_frame->desc_word[3] |= cpu_to_le16(common->tx_power << 8);

	skb_put(skb, FRAME_DESC_SZ);
	rsi_hex_dump(INT_MGMT_ZONE, "RADIO-CAP UPDATE", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_vap_dynamic_update() - This function programs the threshold.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_send_vap_dynamic_update(struct rsi_common *common)
{
	struct sk_buff *skb = NULL;
	struct rsi_dynamic_s *dynamic_frame = NULL;

	rsi_dbg(INT_MGMT_ZONE,
		"<==== Sending vap update indication frame ====>\n");

	skb = dev_alloc_skb(sizeof(struct rsi_dynamic_s));
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, sizeof(struct rsi_dynamic_s));
	dynamic_frame = (struct rsi_dynamic_s *)skb->data;

	dynamic_frame->desc_word[0] = cpu_to_le16(
					(sizeof(dynamic_frame->frame_body)) |
					(RSI_WIFI_MGMT_Q << 12));
	dynamic_frame->desc_word[1] = cpu_to_le16(VAP_DYNAMIC_UPDATE);
	dynamic_frame->desc_word[4] = cpu_to_le16(common->rts_threshold);
#if 0
	dynamic_frame->desc_word[5] = cpu_to_le16(common->frag_threshold);
	dynamic_frame->desc_word[5] = cpu_to_le16(2352);
#endif

#ifdef CONFIG_RSI_WOW
//	if (common->suspend_flag) {
	if (1) {
		dynamic_frame->desc_word[6] =
			cpu_to_le16(24); /* bmiss_threshold */
		dynamic_frame->frame_body.keep_alive_period =
			cpu_to_le16(5);
	} else
		dynamic_frame->frame_body.keep_alive_period = cpu_to_le16(90);
#else
	dynamic_frame->frame_body.keep_alive_period = cpu_to_le16(90);
#endif

#if 0
	dynamic_frame->frame_body.mgmt_rate = cpu_to_le32(RSI_RATE_6);

	dynamic_frame->desc_word[2] |= cpu_to_le32(BIT(1));/* Self cts enable */

	dynamic_frame->desc_word[3] |= cpu_to_le16(BIT(0));/* fixed rate */
	dynamic_frame->frame_body.data_rate = cpu_to_le16(0);
#endif

	dynamic_frame->desc_word[7] |= cpu_to_le16((0 << 8)); /* vap id */

	skb_put(skb, sizeof(struct rsi_dynamic_s));
	rsi_hex_dump(INT_MGMT_ZONE, "VAP UPDATE", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_flash_read() - This function sends the frash read frame to device
 * @adapter: Pointer to the hardware structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
int rsi_flash_read(struct rsi_hw *adapter)
{
	struct rsi_common *common = adapter->priv;
	struct rsi_mac_frame *cmd_frame = NULL;
	struct sk_buff *skb;

	rsi_dbg(INT_MGMT_ZONE, "<==== Sending flash read frame ====>\n");

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, FRAME_DESC_SZ);
	cmd_frame = (struct rsi_mac_frame *)skb->data;

	/* FrameType */
	cmd_frame->desc_word[1] = cpu_to_le16(EEPROM_READ);

	/* Format of length and offset differs for
	 * autoflashing and swbl flashing
	 */
	cmd_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);

	/* Number of bytes to read */
	rsi_dbg(INFO_ZONE, " eeprom length  0x%x, %d\n",
		adapter->eeprom.length, adapter->eeprom.length);
	cmd_frame->desc_word[3] = cpu_to_le16(adapter->eeprom.length << 4);

	cmd_frame->desc_word[2] |= cpu_to_le16(3 << 8);
	if (adapter->eeprom_init) {
		rsi_dbg(INFO_ZONE, "spi init sent");
		cmd_frame->desc_word[2] |= cpu_to_le16(BIT(13));
	}

	/* Address to read */
	cmd_frame->desc_word[4] = cpu_to_le16(adapter->eeprom.offset);
	cmd_frame->desc_word[5] = cpu_to_le16(adapter->eeprom.offset >> 16);
	cmd_frame->desc_word[6] = cpu_to_le16(0); //delay = 0

	skb_put(skb, FRAME_DESC_SZ);
	rsi_hex_dump(INT_MGMT_ZONE, "Internal Mgmt Pkt", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_compare() - This function is used to compare two integers
 * @a: pointer to the first integer
 * @b: pointer to the second integer
 *
 * Return: 0 if both are equal, -1 if the first is smaller, else 1
 */
static int rsi_compare(const void *a, const void *b)
{
	u16 _a = *(const u16 *)(a);
	u16 _b = *(const u16 *)(b);

	if (_a > _b)
		return -1;

	if (_a < _b)
		return 1;

	return 0;
}

/**
 * rsi_map_rates() - This function is used to map selected rates to hw rates.
 * @rate: The standard rate to be mapped.
 * @offset: Offset that will be returned.
 *
 * Return: 0 if it is a mcs rate, else 1
 */
static bool rsi_map_rates(u16 rate, int *offset)
{
	int kk;

	for (kk = 0; kk < ARRAY_SIZE(rsi_mcsrates); kk++) {
		if (rate == mcs[kk]) {
			*offset = kk;
			return false;
		}
	}

	for (kk = 0; kk < ARRAY_SIZE(rsi_rates); kk++) {
		if (rate == rsi_rates[kk].bitrate / 5) {
			*offset = kk;
			break;
		}
	}
	return true;
}

/**
 * rsi_send_auto_rate_request() - This function is to set rates for connection
 *				  and send autorate request to firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
static int rsi_send_auto_rate_request(struct rsi_common *common,
				      struct ieee80211_sta *sta,
				      u16 sta_id)
{
	struct ieee80211_vif *vif = common->priv->vifs[0];
	struct sk_buff *skb;
	struct rsi_auto_rate *auto_rate;
	int ii = 0, jj = 0, kk = 0;
	struct ieee80211_hw *hw = common->priv->hw;
	u8 band = hw->conf.chandef.chan->band;
	u8 num_supported_rates = 0;
	u8 rate_table_offset, rate_offset = 0;
	u32 rate_bitmap = 0;
	u16 *selected_rates, min_rate;
	bool is_ht = false, is_sgi = false;

	rsi_dbg(INT_MGMT_ZONE,
		"<===== SENDING AUTO_RATE_IND FRAME =====>\n");

	skb = dev_alloc_skb(MAX_MGMT_PKT_SIZE);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, MAX_MGMT_PKT_SIZE);

	selected_rates = kzalloc(2 * RSI_TBL_SZ, GFP_KERNEL);
	if (!selected_rates) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of mem\n",
			__func__);
		dev_kfree_skb(skb);
		return -ENOMEM;
	}
	memset(selected_rates, 0, 2 * RSI_TBL_SZ);

	auto_rate = (struct rsi_auto_rate *)skb->data;

	auto_rate->aarf_rssi = cpu_to_le16(((u16)3 << 6) | (u16)(18 & 0x3f));
	auto_rate->collision_tolerance = cpu_to_le16(3);
	auto_rate->failure_limit = cpu_to_le16(3);
	auto_rate->initial_boundary = cpu_to_le16(3);
	auto_rate->max_threshold_limt = cpu_to_le16(27);

	auto_rate->desc_word[1] = cpu_to_le16(AUTO_RATE_IND);

	if (common->channel_width == BW_40MHZ)
		auto_rate->desc_word[7] = cpu_to_le16(1);
	auto_rate->desc_word[7] |= cpu_to_le16(sta_id << 8);

	if (vif->type == NL80211_IFTYPE_STATION) {
		rate_bitmap = common->bitrate_mask[band];
		is_ht = common->vif_info[0].is_ht;
		is_sgi = common->vif_info[0].sgi;
	} else {
		rate_bitmap = sta->supp_rates[band];
		is_ht = sta->ht_cap.ht_supported;
		if ((sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_20) ||
		    (sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_40))
			is_sgi = true;
	}
	rsi_dbg(INFO_ZONE, "rate_bitmap = %x\n", rate_bitmap);
	rsi_dbg(INFO_ZONE, "is_ht = %d\n", is_ht);

	if (band == NL80211_BAND_2GHZ) {
		if ((rate_bitmap == 0) && (is_ht))
			min_rate = RSI_RATE_MCS0;
		else
			min_rate = RSI_RATE_1;
		rate_table_offset = 0;
	} else {
		if ((rate_bitmap == 0) && (is_ht))
			min_rate = RSI_RATE_MCS0;
		else
			min_rate = RSI_RATE_6;
		rate_table_offset = 4;
	}

	for (ii = 0, jj = 0;
	     ii < (ARRAY_SIZE(rsi_rates) - rate_table_offset); ii++) {
		if (rate_bitmap & BIT(ii)) {
			selected_rates[jj++] =
			(rsi_rates[ii + rate_table_offset].bitrate / 5);
			rate_offset++;
		}
	}
	num_supported_rates = jj;

	if (is_ht) {
		for (ii = 0; ii < ARRAY_SIZE(mcs); ii++)
			selected_rates[jj++] = mcs[ii];
		num_supported_rates += ARRAY_SIZE(mcs);
		rate_offset += ARRAY_SIZE(mcs);
	}

	sort(selected_rates, jj, sizeof(u16), &rsi_compare, NULL);

	/* mapping the rates to RSI rates */
	for (ii = 0; ii < jj; ii++) {
		if (rsi_map_rates(selected_rates[ii], &kk)) {
			auto_rate->supported_rates[ii] =
				cpu_to_le16(rsi_rates[kk].hw_value);
		} else {
			auto_rate->supported_rates[ii] =
				cpu_to_le16(rsi_mcsrates[kk]);
		}
	}

	/* loading HT rates in the bottom half of the auto rate table */
	if (is_ht) {
		for (ii = rate_offset, kk = ARRAY_SIZE(rsi_mcsrates) - 1;
		     ii < rate_offset + 2 * ARRAY_SIZE(rsi_mcsrates); ii++) {
			if (is_sgi || conf_is_ht40(&common->priv->hw->conf)) {
				auto_rate->supported_rates[ii++] =
					cpu_to_le16(rsi_mcsrates[kk] |
							ENABLE_SHORTGI_RATE);
			} else {
				auto_rate->supported_rates[ii++] =
					cpu_to_le16(rsi_mcsrates[kk]);
			}
			auto_rate->supported_rates[ii] =
				cpu_to_le16(rsi_mcsrates[kk--]);
		}

		for (; ii < (RSI_TBL_SZ - 1); ii++) {
			auto_rate->supported_rates[ii] =
				cpu_to_le16(rsi_mcsrates[0]);
		}
	}

	for (; ii < RSI_TBL_SZ; ii++)
		auto_rate->supported_rates[ii] = cpu_to_le16(min_rate);

	auto_rate->num_supported_rates = cpu_to_le16(num_supported_rates * 2);
	auto_rate->moderate_rate_inx = cpu_to_le16(num_supported_rates / 2);
	num_supported_rates *= 2;

	auto_rate->desc_word[0] = cpu_to_le16((sizeof(*auto_rate) -
					      FRAME_DESC_SZ) |
					      (RSI_WIFI_MGMT_Q << 12));

	skb_put(skb, sizeof(struct rsi_auto_rate));
	kfree(selected_rates);
	rsi_hex_dump(INT_MGMT_ZONE, "AUTO_RATE FRAME:", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_validate_bgscan_channels() - This function is used to validate
 *				the user configured bgscan channels for
 *				current regulatory domain
 * @chn_num: It holds the user or default channel for validation.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
void rsi_validate_bgscan_channels(struct rsi_hw *adapter,
				  struct bgscan_config_params *params)
{
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *ch;
	struct wiphy *wiphy = adapter->hw->wiphy; 
	u16 bgscan_channels[MAX_BGSCAN_CHANNELS] = {1, 2, 3, 4, 5, 6, 7, 8, 9,
						    10, 11, 12, 13, 14, 36, 40,
						    44, 48, 52, 56, 60, 64, 100,
						    104, 108, 112, 116, 120, 124,
						    128, 132, 136, 140, 149, 153,
						    157, 161, 165};

	int ch_num, i;
	int num_valid_chs = 0, cnt;
	struct rsi_common *common = adapter->priv;

	/* If user passes 0 for num of bgscan channels, take all channels */
	if (params->num_user_channels == 0) {
		if (adapter->priv->num_supp_bands > 1)
			params->num_user_channels = MAX_BGSCAN_CHANNELS;
		else
			params->num_user_channels = 14;

		for (cnt = 0; cnt < params->num_user_channels; cnt++)
			params->user_channels[cnt] = bgscan_channels[cnt];
	}

	rsi_dbg(INFO_ZONE, "Final bgscan channels:\n");
	for (cnt = 0; cnt < params->num_user_channels; cnt++) {
		ch_num = params->user_channels[cnt];

		if ((ch_num < 1) ||
		    ((ch_num > 14) && (ch_num < 36)) ||
		    ((ch_num > 64) && (ch_num < 100)) ||
		    ((ch_num > 140) && (ch_num < 149)) ||
		    (ch_num > 165))
			continue;
		if ((ch_num >= 36) && (ch_num < 149) && (ch_num % 4))
			continue;

		if (ch_num > 14)
			sband = wiphy->bands[NL80211_BAND_5GHZ];
		else
			sband = wiphy->bands[NL80211_BAND_2GHZ];

		for (i = 0; i < sband->n_channels; i++) {
			ch = &sband->channels[i];

			if (ch->hw_value == ch_num)
				break;
		}
		if (i >= sband->n_channels)
			continue;

		/* Check channel availability for the current reg domain */
		if (ch->flags & IEEE80211_CHAN_DISABLED)
			continue;

		if (adapter->device_model == RSI_DEV_9113) {
			params->channels2scan[num_valid_chs] = ch_num;
			rsi_dbg(INFO_ZONE, "%d ", ch_num);
			if ((ch->flags & IEEE80211_CHAN_RADAR)) {
				rsi_dbg(INFO_ZONE, "[DFS]");
				params->channels2scan[num_valid_chs] |=
					(cpu_to_le16(BIT(15))); /* DFS indication */
			}
			num_valid_chs++;
		} else {
			if (common->band == NL80211_BAND_2GHZ && ch_num <= 14) {
				params->channels2scan[num_valid_chs] = ch_num;
				rsi_dbg(INFO_ZONE, "%d ", ch_num);
				num_valid_chs++;
			} else if (common->band == NL80211_BAND_5GHZ && ch_num > 14) {
				params->channels2scan[num_valid_chs] = ch_num;
				rsi_dbg(INFO_ZONE, "%d ", ch_num);
				if ((ch->flags & IEEE80211_CHAN_RADAR)) {
					rsi_dbg(INFO_ZONE, "[DFS]");
					params->channels2scan[num_valid_chs] |=
						(cpu_to_le16(BIT(15)));
				}
				num_valid_chs++;
			}
		}
	}

	params->num_bg_channels = num_valid_chs;
}

/**
 * rsi_send_bgscan_params() - This function sends the background
 *			      scan parameters to firmware.
 * @common: Pointer to the driver private structure.
 * @enable: bgscan enable/disable
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_send_bgscan_params(struct rsi_common *common, int enable)
{
	struct rsi_bgscan_params *bgscan;
	struct bgscan_config_params *info = &common->bgscan_info;
	struct sk_buff *skb;
	u16 frame_len = sizeof(*bgscan);

	rsi_dbg(INT_MGMT_ZONE,
		"<===== Sending bgscan params frame ====>\n");

	rsi_validate_bgscan_channels(common->priv, info);
	if (!info->num_bg_channels) {
		rsi_dbg(ERR_ZONE, "##### No valid bgscan channels #####\n");
		return -1;
	}
	
	skb = dev_alloc_skb(frame_len);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, frame_len);

	bgscan = (struct rsi_bgscan_params *)skb->data;

	bgscan->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
					   (RSI_WIFI_MGMT_Q << 12));
	bgscan->desc_word[1] = cpu_to_le16(BG_SCAN_PARAMS);

	bgscan->bgscan_threshold = cpu_to_le16(info->bgscan_threshold);
	bgscan->roam_threshold = cpu_to_le16(info->roam_threshold);
	if (enable) {
		bgscan->bgscan_periodicity =
			cpu_to_le16(info->bgscan_periodicity);
	}
	bgscan->active_scan_duration =
			cpu_to_le16(info->active_scan_duration);
	bgscan->passive_scan_duration =
			cpu_to_le16(info->passive_scan_duration);
	bgscan->two_probe = info->two_probe;

	if (common->debugfs_bgscan) {
		bgscan->num_bg_channels = common->bgscan_info.num_user_channels;
		memcpy(bgscan->channels2scan,
			common->bgscan_info.user_channels,
			bgscan->num_bg_channels * 2);
	} else {
		memcpy(bgscan->channels2scan,
			info->channels2scan,
			info->num_bg_channels * 2);
		bgscan->num_bg_channels = info->num_bg_channels;
	}
	skb_put(skb, frame_len);
	rsi_hex_dump(INT_MGMT_ZONE, "Internal Mgmt Pkt", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_bgscan_probe_req() - This function sends the background
 *                               scan probe request to firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_send_bgscan_probe_req(struct rsi_common *common)
{
	struct rsi_bgscan_probe *bgscan;
	struct sk_buff *skb;
	u16 frame_len = sizeof(*bgscan);
	u16 len = 1500;
	u16 pbreq_len = 0;

	rsi_dbg(INT_MGMT_ZONE,
		"<==== Sending bgscan probe req frame ====>\n");

	skb = dev_alloc_skb(frame_len + len);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, frame_len + len);

	bgscan = (struct rsi_bgscan_probe *)skb->data;

	bgscan->desc_word[1] = cpu_to_le16(BG_SCAN_PROBE_REQ);
	bgscan->flags = cpu_to_le16(HOST_BG_SCAN_TRIG);
	if (common->band == NL80211_BAND_5GHZ) {
		bgscan->mgmt_rate = cpu_to_le16(RSI_RATE_6);
		bgscan->channel_num = cpu_to_le16(40);
	} else {
		bgscan->mgmt_rate = cpu_to_le16(RSI_RATE_1);
		bgscan->channel_num = cpu_to_le16(11);
	}

	bgscan->channel_scan_time = cpu_to_le16(20);
	if (common->bgscan_probe_req_len > 0) {
		pbreq_len = common->bgscan_probe_req_len;
		bgscan->probe_req_length = pbreq_len;
		memcpy(&skb->data[frame_len], common->bgscan_probe_req,
		       common->bgscan_probe_req_len);
	}

	bgscan->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ + pbreq_len) |
					   (RSI_WIFI_MGMT_Q << 12));

	skb_put(skb, frame_len + pbreq_len);
	rsi_hex_dump(INT_MGMT_ZONE, "Internal Mgmt Pkt", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_inform_bss_status() - This function informs about bss status with the
 *			     help of sta notify params by sending an internal
 *			     management frame to firmware.
 * @common: Pointer to the driver private structure.
 * @status: Bss status type.
 * @bssid: Bssid.
 * @qos_enable: Qos is enabled.
 * @aid: Aid (unique for all STAs).
 *
 * Return: None.
 */
void rsi_inform_bss_status(struct rsi_common *common,
			   enum opmode opmode,
			   u8 status,
			   const u8 *bssid,
			   u8 qos_enable,
			   u16 aid,
			   struct ieee80211_sta *sta,
			   u16 sta_id, u16 assoc_cap)
{
	if (status) {
		if (opmode == STA_OPMODE)
			common->hw_data_qs_blocked = true;
		rsi_send_sta_notify_frame(common,
					  opmode,
					  STA_CONNECTED,
					  bssid,
					  qos_enable,
					  aid,
					  sta_id);
		if (common->min_rate == 0xffff) {
			rsi_dbg(INFO_ZONE, "Send auto rate request\n");
			rsi_send_auto_rate_request(common, sta, sta_id);
		}
		if (opmode == STA_OPMODE) {
			rsi_load_radio_caps(common);
			if (!(assoc_cap & BIT(4))) {	
				rsi_dbg(INFO_ZONE, "unblock data in Open case\n");
				if (!rsi_send_block_unblock_frame(common, false))
					common->hw_data_qs_blocked = false;
			}
		} else if ((opmode == AP_OPMODE && common->hw_data_qs_blocked)) {
			rsi_dbg(INFO_ZONE, "unblock data in AP mode\n");
			if(!rsi_send_block_unblock_frame(common, false))
				common->hw_data_qs_blocked = false;
		}
	} else {
		if (opmode == STA_OPMODE)
			common->hw_data_qs_blocked = true;
#ifdef CONFIG_RSI_WOW
		if (!(common->wow_flags & RSI_WOW_ENABLED)) {
#endif
		rsi_send_sta_notify_frame(common,
					  opmode,
					  STA_DISCONNECTED,
					  bssid,
					  qos_enable,
					  aid,
					  sta_id);
#ifdef CONFIG_RSI_WOW
		}
#endif
		if (opmode == STA_OPMODE)
			rsi_send_block_unblock_frame(common, true);
	}
}

/**
 * rsi_eeprom_read() - This function sends a frame to read the mac address
 *		       from the eeprom.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_eeprom_read(struct rsi_common *common)
{
	struct rsi_mac_frame *mgmt_frame = NULL;
	struct rsi_hw *adapter = common->priv;
	struct sk_buff *skb;

	rsi_dbg(MGMT_TX_ZONE,
		"<==== Sending EEPROM read req frame ====>\n");

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	/* FrameType */
	mgmt_frame->desc_word[1] = cpu_to_le16(EEPROM_READ);
	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);

	/* Number of bytes to read */
	mgmt_frame->desc_word[3] = cpu_to_le16(adapter->eeprom.length << 4);
	mgmt_frame->desc_word[2] |= cpu_to_le16(3 << 8);

	/* Address to read*/
	mgmt_frame->desc_word[4] = cpu_to_le16(adapter->eeprom.offset);
	mgmt_frame->desc_word[5] = cpu_to_le16(adapter->eeprom.offset >> 16);
	mgmt_frame->desc_word[6] = cpu_to_le16(0); //delay = 0

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * This function sends a frame to block/unblock
 * data queues in the firmware
 *
 * @param common Pointer to the driver private structure.
 * @param block event - block if true, unblock if false
 * @return 0 on success, -1 on failure.
 */
int rsi_send_block_unblock_frame(struct rsi_common *common, bool block_event)
{
	struct rsi_mac_frame *mgmt_frame;
	struct sk_buff *skb;

	rsi_dbg(INT_MGMT_ZONE,
		"<==== Sending block/unblock frame ====>\n");

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(BLOCK_HW_QUEUE);
	mgmt_frame->desc_word[3] = cpu_to_le16(0x1);

	if (block_event == true) {
		rsi_dbg(INFO_ZONE, "blocking the data qs\n");
		mgmt_frame->desc_word[4] = cpu_to_le16(0xf);
		mgmt_frame->desc_word[4] |= cpu_to_le16(0xf << 4);
	} else {
		rsi_dbg(INFO_ZONE, "unblocking the data qs\n");
		mgmt_frame->desc_word[5] = cpu_to_le16(0xf);
		mgmt_frame->desc_word[5] |= cpu_to_le16(0xf << 4);
	}

	skb_put(skb, FRAME_DESC_SZ);
	if (block_event == true) {
		rsi_hex_dump(INT_MGMT_ZONE, "BLOCK-DATA QUEUE:",
						skb->data, skb->len);
	} else {
		rsi_hex_dump(INT_MGMT_ZONE, "UNBLOCK-DATA QUEUE:",
						skb->data, skb->len);
	}

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * This function sends a frame to filter the RX packets
 *
 * @param common Pointer to the driver private structure.
 * @param rx_filter_word - Flags of filter packets
 * @return 0 on success, -1 on failure.
 */
int rsi_send_rx_filter_frame(struct rsi_common *common, u16 rx_filter_word)
{
	struct rsi_mac_frame *mgmt_frame;
	struct sk_buff *skb;

	rsi_dbg(INT_MGMT_ZONE,
		"<==== Sending RX filter frame ====>\n");

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(SET_RX_FILTER);
	mgmt_frame->desc_word[4] = cpu_to_le16(rx_filter_word);

	skb_put(skb, FRAME_DESC_SZ);
	rsi_hex_dump(INT_MGMT_ZONE, "RX FILTER FRAME", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}
EXPORT_SYMBOL_GPL(rsi_send_rx_filter_frame); 

/**
 * rsi_send_ps_request() - Sends power save request.
 *
 * @adapter: pointer to rsi_hw structure.
 * @enable: enable or disable power save.
 *
 * returns: 0 on success, negative error code on failure
 */
int rsi_send_ps_request(struct rsi_hw *adapter, bool enable)
{
	struct rsi_common *common = adapter->priv;
	struct ieee80211_bss_conf *bss;
	struct rsi_request_ps *ps = NULL;
	struct rsi_ps_info *ps_info = NULL;
	struct sk_buff *skb = NULL;
	int frame_len = sizeof(*ps);
	bool assoc;

	rsi_dbg(INT_MGMT_ZONE, "<===== SENDING %s REQUEST =====>\n",
		enable ? "PS_ENABLED" : "PS_DISABLED");
	skb = dev_alloc_skb(frame_len);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, frame_len);

	ps = (struct rsi_request_ps *)&skb->data[0];
	ps_info = &adapter->ps_info;

	ps->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
				       (RSI_WIFI_MGMT_Q << 12));
	ps->desc_word[1] = cpu_to_le16(WAKEUP_SLEEP_REQUEST);
	if (enable) {
		ps->ps_sleep.enable = 1;
		ps->desc_word[6] = SLEEP_REQUEST;
	} else {
		ps->ps_sleep.enable = 0;
		ps->desc_word[0] |= BIT(15);
		ps->desc_word[6] = WAKEUP_REQUEST;
	}

	if (common->uapsd_bitmap) {
//		ps->ps_mimic_support = 1;
		ps->ps_uapsd_acs = common->uapsd_bitmap;
		ps->ps_uapsd_acs = (adapter->hw->uapsd_max_sp_len <<
				    IEEE80211_WMM_IE_STA_QOSINFO_SP_SHIFT) |
				    IEEE80211_WMM_IE_STA_QOSINFO_AC_MASK;
		ps->ps_uapsd_wakeup_period = ps_info->uapsd_wakeup_period;
	}
	ps->ps_sleep.sleep_type = ps_info->sleep_type;
	ps->ps_sleep.num_bcns_per_lis_int =
		cpu_to_le16(ps_info->num_bcns_per_lis_int);
	ps->ps_sleep.sleep_duration =
		cpu_to_le32(ps_info->deep_sleep_wakeup_period);

	if (adapter->sc_nvifs == 0) {
		assoc = false;
	} else {
		bss = &adapter->vifs[0]->bss_conf;
		if (bss->assoc)
			assoc = true;
		else
			assoc = false;
	}
	if (assoc)
		ps->ps_sleep.connected_sleep = CONNECTED_SLEEP;
	else
		ps->ps_sleep.connected_sleep = DEEP_SLEEP;

	ps->ps_listen_interval = cpu_to_le32(ps_info->listen_interval);
	ps->ps_dtim_interval_duration = cpu_to_le32(ps_info->dtim_interval_duration);

	if (ps->ps_listen_interval > ps->ps_dtim_interval_duration)
		ps->ps_listen_interval = 0;

	skb_put(skb, frame_len);
	rsi_hex_dump(INT_MGMT_ZONE, "PS-REQ FRAME", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_set_antenna() - This function handles antenna selection functionality.
 *
 * @common: Pointer to the driver private structure.
 * @antenna: bitmap for tx antenna selection
 *
 * Return: 0 on Success, < 0 on failure
 */
int rsi_set_antenna(struct rsi_common *common,
		    u8 antenna)
{
	struct rsi_mac_frame *mgmt_frame;
	struct sk_buff *skb;

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[1] = cpu_to_le16(ANT_SEL_FRAME);
	mgmt_frame->desc_word[2] = cpu_to_le16(1 << 8); /* Antenna selection
							   type */
	mgmt_frame->desc_word[3] = cpu_to_le16(antenna & 0x00ff);
	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

#ifdef CONFIG_RSI_11K
int rsi_get_channel_load_meas(struct rsi_common *common,
			      struct rsi_meas_params chl_meas)
{
	struct rsi_mac_frame *cmd_frame;
	struct sk_buff *skb;
	u8 channel_width;
	u8 dfs = 0;

	rsi_dbg(MGMT_TX_ZONE, " %s: Sending channel load request frame\n",
		__func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, FRAME_DESC_SZ);

	cmd_frame = (struct rsi_mac_frame *)skb->data;

	channel_width = ieee80211_chan_to_bw(common->priv,
					     chl_meas.regulatory_class,
					     chl_meas.channel_num);

	dfs = is_dfs_channel(common->priv, chl_meas.channel_num);

	if (dfs == 1)
		rsi_dbg(ERR_ZONE, "Requested channel is DFS\n");
	else
		rsi_dbg(ERR_ZONE, "Requested channel is NON DFS\n");

	rsi_dbg(INFO_ZONE, "Regulatory value %d\n", chl_meas.regulatory_class);
	rsi_dbg(INFO_ZONE, "channel_Num value %d\n", chl_meas.channel_num);
	rsi_dbg(INFO_ZONE, "Meas_duration value %d\n", chl_meas.meas_duration);
	rsi_dbg(INFO_ZONE, "rand_interval value %d\n", chl_meas.rand_interval);
	rsi_dbg(INFO_ZONE, "channel_width value %d\n", chl_meas.channel_width);
	rsi_dbg(INFO_ZONE, "meas_req_mode value %d\n", chl_meas.meas_req_mode);
	rsi_dbg(INFO_ZONE, "meas_type value %d\n", chl_meas.meas_type);

	switch (channel_width) {
	case RSI_BW_5:
		channel_width = 0;
		break;
	case RSI_BW_10:
		channel_width = 1;
		break;
	case RSI_BW_20:
		channel_width = 2;
		break;
	case RSI_BW_25:
		channel_width = 2;
		break;
	case RSI_BW_40:
		channel_width = 3;
		break;
	default:
		channel_width = 4;
		break;
	}
	rsi_dbg(INFO_ZONE, "Channel_width = %d\n", channel_width);

	cmd_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	cmd_frame->desc_word[1] = cpu_to_le16(RADIO_MEASUREMENT_REQ);
	cmd_frame->desc_word[2] = cpu_to_le16(chl_meas.meas_type << 8);
	cmd_frame->desc_word[3] = cpu_to_le16(chl_meas.meas_req_mode);
	cmd_frame->desc_word[3] |=
		cpu_to_le16(((u16)chl_meas.channel_num) << 8);
	cmd_frame->desc_word[4] = cpu_to_le16(chl_meas.meas_duration);
	cmd_frame->desc_word[5] = cpu_to_le16(chl_meas.rand_interval);
	cmd_frame->desc_word[6] = cpu_to_le16(dfs << 8);
	cmd_frame->desc_word[6] |= cpu_to_le16(channel_width);

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

int rsi_get_frame_meas(struct rsi_common *common,
		       struct rsi_frame_meas_params params)
{
	struct rsi_frame_meas_req *cmd_frame;
	struct sk_buff *skb;
	u8 channel_width, length;
	int ret;

	rsi_dbg(MGMT_TX_ZONE, "%s: Sending frame request frame\n", __func__);

	skb = dev_alloc_skb(sizeof(*cmd_frame));
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, sizeof(*cmd_frame));
	cmd_frame = (struct rsi_frame_meas_req *)skb->data;

	channel_width = ieee80211_chan_to_bw(common->priv,
					     params.mp.regulatory_class,
					     params.mp.channel_num);
	if (channel_width == RSI_BW_5)
		channel_width = 0;
	else if (channel_width == RSI_BW_10)
		channel_width = 1;
	else if (channel_width == RSI_BW_20)
		channel_width = 2;
	else if (channel_width == RSI_BW_25)
		channel_width = 2;
	else if (channel_width == RSI_BW_40)
		channel_width = 3;
	else
		channel_width = 4;
	rsi_dbg(INFO_ZONE, "channel_width = %d\n", channel_width);

	ret = is_dfs_channel(common->priv, params.mp.channel_num);
	if (ret == 1)
		rsi_dbg(ERR_ZONE, "Requested channel is DFS\n");
	else
		rsi_dbg(ERR_ZONE, "Requested channel is NON DFS\n");

	cmd_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	cmd_frame->desc_word[1] |= cpu_to_le16(RADIO_MEASUREMENT_REQ);
	cmd_frame->desc_word[2] = cpu_to_le16(params.mp.meas_type << 8);
	cmd_frame->desc_word[3] = cpu_to_le16(params.mp.meas_req_mode);
	cmd_frame->desc_word[3] |= cpu_to_le16(((u16)params.mp.channel_num) << 8);
	cmd_frame->desc_word[4] = cpu_to_le16(params.mp.meas_duration);
	cmd_frame->desc_word[5] = cpu_to_le16(params.mp.rand_interval);
	cmd_frame->desc_word[6] = cpu_to_le16(ret << 8);
	cmd_frame->desc_word[6] |= cpu_to_le16(channel_width);

	memcpy(cmd_frame->bssid, params.mac_addr, IEEE80211_ADDR_LEN);
	cmd_frame->frm_req_type = params.frame_req_type;

	length = FRAME_DESC_SZ + IEEE80211_ADDR_LEN + 1;
	cmd_frame->desc_word[0] |= cpu_to_le16(length - FRAME_DESC_SZ);
	cmd_frame->desc_word[2] |= cpu_to_le16(0);

	skb_put(skb, length);

	return rsi_send_internal_mgmt_frame(common, skb);
}

int rsi_get_beacon_meas(struct rsi_common *common,
			struct rsi_beacon_meas_params params)
{
	struct rsi_bcn_meas_req *beacon_req = NULL;
	struct sk_buff *skb = NULL;
	u8 channel_width = 0, dfs;
	u8 index, length = 0;

	rsi_dbg(ERR_ZONE, "%s: Sending request to get beacon measurements\n",
		__func__);

	skb = dev_alloc_skb(MAX_MGMT_PKT_SIZE);
	if (!skb)
		return -ENOMEM;
	beacon_req = (struct rsi_bcn_meas_req *)skb->data;
	channel_width = ieee80211_chan_to_bw(common->priv,
					     params.mp.regulatory_class,
					     params.mp.channel_num);
	if (channel_width == RSI_BW_5)
		channel_width = 0;
	else if (channel_width == RSI_BW_10)
		channel_width = 1;
	else if (channel_width == RSI_BW_20)
		channel_width = 2;
	else if (channel_width == RSI_BW_25)
		channel_width = 2;
	else if (channel_width == RSI_BW_40)
		channel_width = 3;
	else
		channel_width = 4;
	rsi_dbg(INFO_ZONE, "channel_width = %d\n", channel_width);
	dfs = is_dfs_channel(common->priv, params.mp.channel_num);
	if (dfs == 1)
		rsi_dbg(INFO_ZONE, "Requested channel is DFS\n");
	else
		rsi_dbg(INFO_ZONE, "Requested channel is NON DFS\n");

	beacon_req->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	beacon_req->desc_word[1] = cpu_to_le16(FLAGS << 8);
	beacon_req->desc_word[1] |= cpu_to_le16(RADIO_MEASUREMENT_REQ);
	beacon_req->desc_word[2] = cpu_to_le16(params.mp.meas_type << 8);
	beacon_req->desc_word[3] = cpu_to_le16(params.mp.meas_req_mode);
	beacon_req->desc_word[3] |=
		cpu_to_le16(((u16)params.mp.channel_num) << 8);
	beacon_req->desc_word[4] = cpu_to_le16(params.mp.meas_duration);
	beacon_req->desc_word[5] = cpu_to_le16(params.mp.rand_interval);
	beacon_req->desc_word[6] = cpu_to_le16(dfs << 8); /* DFS chan */
	beacon_req->desc_word[6] |= cpu_to_le16(channel_width);

	memcpy(beacon_req->bssid, params.mac_addr, IEEE80211_ADDR_LEN);
	memcpy(beacon_req->ssid, &params.ssid_ie[2], SSID_LEN);
	beacon_req->rep_detail = params.rpt_detail;
	beacon_req->meas_mode = params.meas_mode;
	index = 0;
	rsi_dbg(ERR_ZONE, "BEACON REQ REPORTING DETAILS %d\n",
		beacon_req->rep_detail);
	rsi_dbg(ERR_ZONE, "BEACON REQ SSID INFORMATION %s\n",
		beacon_req->ssid);
	rsi_dbg(ERR_ZONE, "beacon_req measurement mode %d\n",
		beacon_req->meas_mode);
	if (params.meas_mode == 1) {
		rsi_dbg(ERR_ZONE, "active beacon request with bgscan probe\n");
		memcpy(beacon_req->bgscan_probe,
		       common->bgscan_probe_req,
		       MIN_802_11_HDR_LEN);
		index += MIN_802_11_HDR_LEN;

		memcpy(&beacon_req->bgscan_probe[index],
		       (u8 *)&params.ssid_ie,
		       (2 + params.ssid_ie[1]));
		index += (2 + params.ssid_ie[1]);

		memcpy(&beacon_req->bgscan_probe[index],
		       common->bgscan_probe_req + MIN_802_11_HDR_LEN + 2,
		       (common->bgscan_probe_req_len - MIN_802_11_HDR_LEN - 2));
		index += (common->bgscan_probe_req_len -
				MIN_802_11_HDR_LEN - 2);
	}

	beacon_req->desc_word[0] |= cpu_to_le16(sizeof(*beacon_req) + index);
	length = FRAME_DESC_SZ + sizeof(*beacon_req) + index;
	rsi_hex_dump(INFO_ZONE, "BEACON FRAME HOST TO LMAC", skb->data, length);
	skb_put(skb, FRAME_DESC_SZ + sizeof(*beacon_req) + index);

	return rsi_send_internal_mgmt_frame(common, skb);
}
#endif

#ifdef CONFIG_RSI_WOW 
int rsi_send_wowlan_request(struct rsi_common *common, u16 flags,
		            u16 sleep_status)
{
        struct rsi_wowlan_req *cmd_frame;
        struct sk_buff *skb;
        u8 length;
        u8 sourceid[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        rsi_dbg(ERR_ZONE, "%s: Sending wowlan request frame\n", __func__);

        skb = dev_alloc_skb(sizeof(*cmd_frame));
        if (!skb) {
                rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
                                __func__);
                return -ENOMEM;
        }
        memset(skb->data, 0, sizeof(*cmd_frame));
        cmd_frame = (struct rsi_wowlan_req *)skb->data;

        cmd_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
        cmd_frame->desc_word[1] |= cpu_to_le16(WOWLAN_CONFIG_PARAMS);
 
        memcpy(cmd_frame->sourceid, &sourceid, IEEE80211_ADDR_LEN);
	
        cmd_frame->host_sleep_status = sleep_status;
	if (common->secinfo.gtk_cipher)
		flags |= RSI_WOW_GTK_REKEY;
	if (sleep_status)
		cmd_frame->wow_flags = flags; /* TODO: check for magic packet */
        rsi_dbg(INFO_ZONE, "Host_Sleep_Status : %d Flags : %d\n",
		cmd_frame->host_sleep_status, cmd_frame->wow_flags );
	
        length = FRAME_DESC_SZ + IEEE80211_ADDR_LEN + 2 + 2;

        cmd_frame->desc_word[0] |= cpu_to_le16(length - FRAME_DESC_SZ);
        cmd_frame->desc_word[2] |= cpu_to_le16(0);
  
  	skb_put(skb, length);

        return rsi_send_internal_mgmt_frame(common, skb);
}
#endif

int rsi_send_beacon(struct rsi_common *common)
{
	struct sk_buff *skb = NULL;
	u8 dword_align_bytes = 0;
	
	skb = dev_alloc_skb(MAX_MGMT_PKT_SIZE);
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, MAX_MGMT_PKT_SIZE);
	
	dword_align_bytes = ((unsigned long)skb->data & 0x3f);
	if (dword_align_bytes) {
		skb_pull(skb, (64 - dword_align_bytes));
	}
	if (rsi_prepare_beacon(common, skb)) {
		rsi_dbg(ERR_ZONE, "Failed to prepare beacon\n");
		return -EINVAL;
	}
	skb_queue_tail(&common->tx_queue[MGMT_BEACON_Q], skb);
	rsi_set_event(&common->tx_thread.event);
	rsi_dbg(DATA_TX_ZONE,
		"%s: Added to beacon queue\n", __func__);

	return 0;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 20, 17)
#if LINUX_VERSION_CODE < KERNEL_VERSION (4, 15, 0)
static void channel_change_event(unsigned long priv)
{
	struct rsi_hw *adapter = (struct rsi_hw *)priv;
	struct rsi_common *common = adapter->priv;
#else
static void channel_change_event(struct timer_list *t)
{
	struct rsi_common *common = from_timer(common, t, scan_timer);
#endif

	rsi_set_event(&common->chan_change_event);
	del_timer(&common->scan_timer);
}

static int init_channel_timer(struct rsi_hw *adapter, u32 timeout)
{
	struct rsi_common *common = adapter->priv;

	rsi_reset_event(&common->chan_change_event);
#if LINUX_VERSION_CODE < KERNEL_VERSION (4, 15, 0)
	init_timer(&common->scan_timer);
	common->scan_timer.data = (unsigned long)adapter;
	common->scan_timer.function = (void *)&channel_change_event;
#else
	timer_setup(&common->scan_timer, channel_change_event, 0);
#endif
	common->scan_timer.expires = msecs_to_jiffies(timeout) + jiffies;

	add_timer(&common->scan_timer);

	return 0;
}
#endif
/**
 * This function prepares Probe request frame and send it to LMAC.
 * @param  Pointer to Adapter structure.
 * @param  Type of scan(Active/Passive).
 * @param  Broadcast probe.
 * @param  Pointer to the destination address.
 * @param  Indicates LMAC/UMAC Q number.
 * @return 0 if success else -1.
 */
int rsi_send_probe_request(struct rsi_common *common,
			   struct cfg80211_scan_request *scan_req,
			   u8 n_ssid,
			   u8 channel,
			   u8 scan_type)
{
	struct cfg80211_ssid *ssid_info;
	struct ieee80211_tx_info *info;
	struct skb_info *tx_params;
	struct sk_buff *skb = NULL;
	struct ieee80211_hdr *hdr = NULL;
	u8 *pos = NULL;
	u32 len = 0;
	u8 ie_ssid_len;
	u8 q_num;

	if (common->priv->sc_nvifs <= 0)
		return 0;
	if (!scan_req)
		return 0;
	rsi_dbg(INT_MGMT_ZONE, "<==== Sending Probe Request Packet ====>\n");
	ssid_info = &scan_req->ssids[n_ssid];
	ie_ssid_len = (ssid_info && ssid_info->ssid_len) ? 
		       ssid_info->ssid_len + 2 : NULL_SSID;

	len = (MIN_802_11_HDR_LEN + scan_req->ie_len + ie_ssid_len);

	skb = dev_alloc_skb(len + DWORD_ALIGNMENT); /* 64 for dword alignment */
	if (!skb) {
		rsi_dbg(ERR_ZONE, "Failed to alloc probe req\n");
		return -ENOMEM;
	}
	skb_put(skb, len);
	memset(skb->data, 0, skb->len);
	skb_reserve(skb, DWORD_ALIGNMENT);

	if (scan_type == 0) {
		pos = skb->data;
	
		/*
		 * probe req frame format
		 * ssid
		 * supported rates
		 * RSN (optional)
		 * extended supported rates
		 * WPA (optional)
		 * user-specified ie's
		 */
		hdr = (struct ieee80211_hdr *)skb->data;
	} else {
		pos = common->bgscan_probe_req;
		hdr = (struct ieee80211_hdr *)common->bgscan_probe_req; 
	}
	hdr->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | 
					 IEEE80211_STYPE_PROBE_REQ);
	hdr->duration_id = 0x0;
	memset(hdr->addr1, 0xff, ETH_ALEN);
	memset(hdr->addr3, 0xFF, 6);
	memcpy(hdr->addr2, common->mac_addr, ETH_ALEN);
	hdr->seq_ctrl = 0x00;
	pos += MIN_802_11_HDR_LEN; 

	*pos++ = WLAN_EID_SSID;
	*pos++ = ie_ssid_len ? ie_ssid_len - 2 : 0; 

	if (ssid_info && (ie_ssid_len != NULL_SSID)) {
		if (scan_req->n_ssids > 0 || common->p2p_enabled) {
			memcpy(pos, ssid_info->ssid, ie_ssid_len - NULL_SSID);
			pos += ie_ssid_len - NULL_SSID;
		} else {
			memcpy(pos, ssid_info->ssid, ie_ssid_len);
			pos += ie_ssid_len;
		}
	}

        if (scan_req->ie_len) 
                memcpy(pos, scan_req->ie, scan_req->ie_len);
       
	if (scan_type == 1) {
		if (len > RSI_MAX_BGS_PROBEREQ_LEN) {
			u16 t_len = MIN_802_11_HDR_LEN;

			/* Cut IEs from last */
			pos = &skb->data[MIN_802_11_HDR_LEN];
			while (true) {
				if ((t_len + pos[1] + 2) >
				    RSI_MAX_BGS_PROBEREQ_LEN) {
					skb_trim(skb, t_len);
					len = t_len;
					break;
				}
				t_len += pos[1] + 2;
				pos += (pos[1] + 2);
			}
		}
		common->bgscan_probe_req_len = len;	
		goto out;
	}

	if ((common->iface_down == true))
		goto out;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 20, 17)
	if (!common->scan_in_prog)
		goto out;
#endif

	info = IEEE80211_SKB_CB(skb);
	tx_params = (struct skb_info *)info->driver_data;
	tx_params->internal_hdr_size = skb_headroom(skb);
#ifndef CONFIG_RSI_P2P
	info->control.vif = common->priv->vifs[0];
	if (!info->control.vif) {
		rsi_dbg(ERR_ZONE,
			"%s: VIF is NULL\n", __func__);
		return 0;
	}
#else
	info->control.vif = common->priv->vifs[1];
	if (!info->control.vif) {
		rsi_dbg(ERR_ZONE,
			"%s: VIF is NULL\n", __func__);
		return 0;
	}
	memcpy(hdr->addr2, info->control.vif->addr, ETH_ALEN);
	skb_trim(skb, skb->len - INVALID_DATA);
#endif
	q_num = MGMT_SOFT_Q;
	skb->priority = q_num;

	if(rsi_prepare_mgmt_desc(common,skb))
		goto out;
	rsi_hex_dump(MGMT_DEBUG_ZONE, "PROBE-REQ FRAME", skb->data, skb->len);
	skb_queue_tail(&common->tx_queue[MGMT_SOFT_Q], skb);
	rsi_set_event(&common->tx_thread.event);
	
	return 0;

out:
	dev_kfree_skb(skb);
	return 0;
}

void rsi_scan_complete(struct work_struct *work)
{
	struct rsi_common *common =
		container_of(work, struct rsi_common, scan_complete_work);
	struct rsi_hw *adapter = common->priv;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0))
	struct cfg80211_scan_info info;
	info.aborted = false;
	/* Waiting before reporting scan_done event to CFG. 
 	 * It is the requirement of iw, to give some time 
 	 * between scan_request and scan_done event so that 
 	 * it creates RX socket to receive event*/
	msleep(1);
	ieee80211_scan_completed(adapter->hw, &info);
#else
	/* Waiting before reporting scan_done event to CFG. 
 	 * It is the requirement of iw, to give some time 
 	 * between scan_request and scan_done event so that 
 	 * it creates RX socket to receive event*/
	msleep(1); 	
	ieee80211_scan_completed(adapter->hw, false);
#endif
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 20, 17)
void rsi_scan_start(struct work_struct *work)
{
	struct ieee80211_channel *cur_chan = NULL;
	struct cfg80211_scan_request *scan_req = NULL;
	struct rsi_common *common =
		container_of(work, struct rsi_common, scan_work);
	u8 ii, jj;
	int status = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0))
	struct cfg80211_scan_info info;
#endif
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_vif *vif = adapter->vifs[adapter->sc_nvifs - 1];

	if (vif->type == NL80211_IFTYPE_AP) {
		if (adapter->auto_chan_sel == ACS_DISABLE) {
			adapter->auto_chan_sel = ACS_ENABLE;
			rsi_dbg(INFO_ZONE, "Auto Channel selection scan start");
		}
		adapter->idx = 0;
	}
	
	scan_req = common->scan_request;
	if (!scan_req)
		return;

	common->scan_in_prog = true;
#ifndef CONFIG_RSI_P2P
	if (adapter->ps_state == PS_ENABLED) {
		rsi_disable_ps(common->priv);
		rsi_reset_event(&common->mgmt_cfm_event);
		rsi_wait_event(&common->mgmt_cfm_event, msecs_to_jiffies(2000));
	}
#endif
	for (ii =0; ii < scan_req->n_channels ; ii++) {
		if (common->iface_down)
			break;
	
		if (!common->scan_in_prog)
			break;
		
		rsi_dbg(INFO_ZONE, "Programming channel no: %d",
			scan_req->channels[ii]->hw_value);
		cur_chan = scan_req->channels[ii];

		rsi_band_check(common, cur_chan);
		if (cur_chan->flags & IEEE80211_CHAN_DISABLED)
			continue;
				 
		if (rsi_set_channel(common, cur_chan)) {
			rsi_dbg(ERR_ZONE, "Failed to set the channel\n");
			break;
		}
		rsi_reset_event(&common->chan_set_event);
		status = rsi_wait_event(&common->chan_set_event, msecs_to_jiffies(50));
		if (status < 0)
			break;
		
		if (!common->scan_in_prog)
			break;
		rsi_reset_event(&common->chan_set_event);

		if ((cur_chan->flags & IEEE80211_CHAN_NO_IR) ||
		    (cur_chan->flags & IEEE80211_CHAN_RADAR)) {
			/* DFS Channel */
			/* Program passive scan duration */
			init_channel_timer(common->priv, PASSIVE_SCAN_DURATION);
		} else if (!adapter->auto_chan_sel) {
			/* Send probe request */
			for (jj = 0; jj < scan_req->n_ssids; jj++) {
				rsi_send_probe_request(common, 
						       scan_req,
						       jj,
						       cur_chan->hw_value,
						       0);
				if (common->iface_down == true) {
					common->scan_in_prog = false;
					return;
				}
				rsi_reset_event(&common->probe_cfm_event);
				status = rsi_wait_event(&common->probe_cfm_event,
					       msecs_to_jiffies(50));
				if (status < 0) {
					rsi_dbg(ERR_ZONE,
						"Did not received probe confirm\n");
					common->scan_in_prog = false;
					return;
				}
				rsi_reset_event(&common->probe_cfm_event);
			}
			init_channel_timer(common->priv, ACTIVE_SCAN_DURATION);
		}
		if (!common->scan_in_prog)
			break;
		if (common->iface_down)
			break;

		status = rsi_wait_event(&common->chan_change_event, 
					EVENT_WAIT_FOREVER);
		if (status < 0)
			break;
		rsi_reset_event(&common->chan_change_event);
	}

	del_timer(&common->scan_timer);
	common->scan_in_prog = false;
#ifndef CONFIG_RSI_P2P
	if (vif->type != NL80211_IFTYPE_AP && adapter->ps_state == PS_NONE) {
		rsi_enable_ps(common->priv);
		rsi_reset_event(&common->mgmt_cfm_event);
		rsi_wait_event(&common->mgmt_cfm_event,
				msecs_to_jiffies(2000));
	}
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0))
	info.aborted = false;
	ieee80211_scan_completed(common->priv->hw, &info);
#else	
	ieee80211_scan_completed(common->priv->hw, false);
#endif

	if (common->hw_scan_cancel) {	
		skb_queue_purge(&common->tx_queue[MGMT_SOFT_Q]);
		rsi_set_event(&common->cancel_hw_scan_event);
	}	
	return;
}
#endif

static int rsi_send_w9116_features(struct rsi_common *common)
{
	struct rsi_wlan_9116_features *w9116_features;
	u16 frame_len = sizeof(struct rsi_wlan_9116_features);
	struct sk_buff *skb;

	rsi_dbg(INT_MGMT_ZONE,
		"<==== Sending wlan 9116 features ====>\n");

	skb = dev_alloc_skb(frame_len);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, frame_len);

	w9116_features = (struct rsi_wlan_9116_features *)skb->data;

	w9116_features->pll_mode = common->w9116_features.pll_mode;
	w9116_features->rf_type = common->w9116_features.rf_type;
	w9116_features->wireless_mode = common->w9116_features.wireless_mode;
	w9116_features->enable_ppe = common->w9116_features.enable_ppe;
	w9116_features->afe_type = common->w9116_features.afe_type;
	if (common->w9116_features.dpd)
		w9116_features->feature_enable |= cpu_to_le32(RSI_DPD);
	if (common->w9116_features.sifs_tx_enable)
		w9116_features->feature_enable |=
			cpu_to_le32(RSI_SIFS_TX_ENABLE);
	if (common->w9116_features.ps_options & RSI_DUTY_CYCLING)
		w9116_features->feature_enable |= cpu_to_le32(RSI_DUTY_CYCLING);
	if (common->w9116_features.ps_options & RSI_END_OF_FRAME)
		w9116_features->feature_enable |= cpu_to_le32(RSI_END_OF_FRAME);
	w9116_features->feature_enable |=
				cpu_to_le32(common->wlan_pwrsave_options);
	if (w9116_features->feature_enable & LMAC_BCON_DROP_EN) {
		w9116_features->feature_enable |=
				cpu_to_le32(DROP_BYTES_FEATURE);
		w9116_features->feature_enable |=
			cpu_to_le32(LMAC_BCON_EN_DIS_THRESHOLD << 28);
	}

	w9116_features->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
						   (RSI_WIFI_MGMT_Q << 12));
	w9116_features->desc_word[1] = cpu_to_le16(FEATURES_ENABLE);
	skb_put(skb, frame_len);
	rsi_hex_dump(INT_MGMT_ZONE, "Internal Mgmt Pkt", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

int rsi_send_bt_reg_params(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct bt_register_param *bt_reg_param;

	rsi_dbg(ERR_ZONE, "%s: Sending BT reg frame\n", __func__);
	skb = dev_alloc_skb(sizeof(*bt_reg_param));
	if (!skb) {
		rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
				__func__);
		return -ENOMEM;
	}
#define BT_PKT_TYPE 0x55
	memset(skb->data, 0, sizeof(struct bt_register_param));
	bt_reg_param = (struct bt_register_param *)skb->data;

	bt_reg_param->desc_word[7] = cpu_to_le16(BT_PKT_TYPE);
	bt_reg_param->desc_word[0] = (RSI_BT_MGMT_Q << 12);
	if (common->priv->device_model == RSI_DEV_9116) {
		bt_reg_param->desc_word[0] |= cpu_to_le16((sizeof(*bt_reg_param) 
							   - FRAME_DESC_SZ));
		bt_reg_param->params[2] = (common->bt_rf_type & 0x0F);
		bt_reg_param->params[3] = common->ble_tx_pwr_inx;
		bt_reg_param->params[4] = (common->ble_pwr_save_options & 0x0F);
		skb_put(skb, sizeof(struct bt_register_param));
	} else {
		bt_reg_param->desc_word[0] |= cpu_to_le16((sizeof(*bt_reg_param) 
					  		   - FRAME_DESC_SZ - 3));
		bt_reg_param->params[0] = (common->bt_rf_tx_power_mode & 0x0F);
		bt_reg_param->params[0] |= (common->bt_rf_rx_power_mode & 0xF0);
		skb_put(skb, sizeof(struct bt_register_param) - 3);
	}
		
	rsi_hex_dump(ERR_ZONE, "BT REG PARAMS", skb->data, skb->len);
#ifdef CONFIG_RSI_COEX_MODE
	return rsi_coex_send_pkt(common, skb, BT_Q);
#else
	return rsi_send_bt_pkt(common, skb);
#endif

}

/**
 * rsi_handle_ta_confirm() - This function handles the confirm frames.
 * @common: Pointer to the driver private structure.
 * @msg: Pointer to received packet.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_handle_ta_confirm(struct rsi_common *common, u8 *msg)
{
	struct rsi_hw *adapter = common->priv;
	u8 sub_type = (msg[15] & 0xff);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0))
	struct cfg80211_scan_info info;
#endif
	struct ieee80211_vif *vif = adapter->vifs[adapter->sc_nvifs - 1];

	rsi_dbg(MGMT_RX_ZONE, "%s: subtype=%d\n", __func__, sub_type);

	switch (sub_type) {
	case COMMON_DEV_CONFIG:
		rsi_dbg(FSM_ZONE,
			"Common Dev Config params confirm received\n");
		if (common->fsm_state == FSM_COMMON_DEV_PARAMS_SENT) {
			if (rsi_load_bootup_params(common)) {
				common->fsm_state = FSM_CARD_NOT_READY;
				goto out;
			} else {
				common->fsm_state = FSM_BOOT_PARAMS_SENT;
			}
		} else {
			rsi_dbg(INFO_ZONE,
				"%s: Received common dev config params cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case BOOTUP_PARAMS_REQUEST:
		rsi_dbg(FSM_ZONE, "Bootup params confirmation.\n");
		if (common->fsm_state == FSM_BOOT_PARAMS_SENT) {
			if (adapter->device_model == RSI_DEV_9116) {
				if (common->band == NL80211_BAND_5GHZ)
					common->num_supp_bands = 2;
				else
					common->num_supp_bands = 1;

				if (rsi_send_reset_mac(common))
					goto out;
				else
					common->fsm_state = FSM_RESET_MAC_SENT;
			} else {
				adapter->eeprom.length = (IEEE80211_ADDR_LEN +
						WLAN_MAC_MAGIC_WORD_LEN +
						WLAN_HOST_MODE_LEN);
				adapter->eeprom.offset = WLAN_MAC_EEPROM_ADDR;
				if (rsi_eeprom_read(common)) {
					common->fsm_state = FSM_CARD_NOT_READY;
					goto out;
				} else
					common->fsm_state =
						FSM_EEPROM_READ_MAC_ADDR;
			}
		} else {
			rsi_dbg(INFO_ZONE,
				"%s: Received bootup params cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case EEPROM_READ:
		rsi_dbg(FSM_ZONE, "EEPROM READ confirm received\n");
		if (common->fsm_state == FSM_EEPROM_READ_MAC_ADDR) {
			u32 msg_len = ((u16 *)msg)[0] & 0xfff;

			if (msg_len <= 0) {
				rsi_dbg(FSM_ZONE,
					"%s: [EEPROM_READ] Invalid len %d\n",
					__func__, msg_len);
				goto out;
			}
			if (msg[16] == MAGIC_WORD) {
				u8 offset = (FRAME_DESC_SZ +
					     WLAN_HOST_MODE_LEN +
					     WLAN_MAC_MAGIC_WORD_LEN);

				memcpy(common->mac_addr,
				       &msg[offset],
				       IEEE80211_ADDR_LEN);
				rsi_hex_dump(INIT_ZONE,
					     "MAC Addr",
					     common->mac_addr, ETH_ALEN);
				adapter->eeprom.length =
					((WLAN_MAC_MAGIC_WORD_LEN + 3) & (~3));
				adapter->eeprom.offset =
					WLAN_EEPROM_RFTYPE_ADDR;
				if (rsi_eeprom_read(common)) {
					rsi_dbg(ERR_ZONE,
						"%s: Failed reading RF band\n",
						__func__);
					common->fsm_state = FSM_CARD_NOT_READY;
				} else {
					common->fsm_state =
						FSM_EEPROM_READ_RF_TYPE;
				}
			} else {
				common->fsm_state = FSM_CARD_NOT_READY;
				break;
			}
		} else if (common->fsm_state == FSM_EEPROM_READ_RF_TYPE) {
			u32 msg_len = ((u16 *)msg)[0] & 0xfff;

			if (msg_len <= 0) {
				rsi_dbg(FSM_ZONE,
					"%s:[EEPROM_READ_CFM] Invalid len %d\n",
					__func__, msg_len);
				goto out;
			}
			if (msg[16] == MAGIC_WORD) {
				if ((msg[17] & 0x3) == 0x3) {
					rsi_dbg(INIT_ZONE,
						"Dual band supported\n");
					common->band = NL80211_BAND_5GHZ;
					common->num_supp_bands = 2;
				} else if ((msg[17] & 0x3) == 0x1) {
					rsi_dbg(INIT_ZONE,
						"Only 2.4Ghz band supported\n");
					common->band = NL80211_BAND_2GHZ;
					common->num_supp_bands = 1;
				}
			} else {
				common->fsm_state = FSM_CARD_NOT_READY;
				break;
			}
			if (rsi_send_reset_mac(common))
				goto out;
			else
				common->fsm_state = FSM_RESET_MAC_SENT;
		} else {
			rsi_dbg(ERR_ZONE,
				"%s: Received eeprom read in %d state\n",
				__func__, common->fsm_state);
			return 0;
		}
		break;

	case RESET_MAC_REQ:
		if (common->fsm_state == FSM_RESET_MAC_SENT) {
			rsi_dbg(FSM_ZONE, "Reset MAC confirm\n");

			if (rsi_load_radio_caps(common))
				goto out;
			else
				common->fsm_state = FSM_RADIO_CAPS_SENT;
		} else {
			rsi_dbg(ERR_ZONE,
				"%s: Received reset mac cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case RADIO_CAPABILITIES:
		if (common->fsm_state == FSM_RADIO_CAPS_SENT) {
			common->rf_reset = 1;
			if ((adapter->device_model == RSI_DEV_9116) &&
			    rsi_send_w9116_features(common)) {
				rsi_dbg(ERR_ZONE,
					"Failed to send 9116 features\n");
				goto out;
			}
			if (rsi_program_bb_rf(common)) {
				goto out;
			} else {
				common->fsm_state = FSM_BB_RF_PROG_SENT;
				rsi_dbg(FSM_ZONE, "Radio caps confirm\n");
			}
		} else {
			rsi_dbg(INFO_ZONE,
				"%s: Received radio caps cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case BB_PROG_VALUES_REQUEST:
	case RF_PROG_VALUES_REQUEST:
	case BBP_PROG_IN_TA:
		rsi_dbg(FSM_ZONE, "BB/RF confirmation.\n");
		if (common->fsm_state == FSM_BB_RF_PROG_SENT) {
			common->bb_rf_prog_count--;
			if (!common->bb_rf_prog_count) {
				common->fsm_state = FSM_MAC_INIT_DONE;
				if (common->reinit_hw) {
					common->hw_data_qs_blocked = false;
					ieee80211_wake_queues(adapter->hw);
					complete(&common->wlan_init_completion);
					common->reinit_hw = false;
				} else
					return rsi_mac80211_attach(common);
			}
		} else {
			rsi_dbg(INFO_ZONE,
				"%s: Received bb_rf cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case AMPDU_IND:
		rsi_dbg(INFO_ZONE, "AMPDU indication.\n");
		break;

	case SCAN_REQUEST:
		rsi_dbg(INFO_ZONE, "Scan confirm.\n");
		if (vif->type == NL80211_IFTYPE_AP &&
		    adapter->auto_chan_sel) {
			u8 id;
			struct acs_stats_s *acs_data =
				(struct acs_stats_s *)(&msg[FRAME_DESC_SZ]);

			if (adapter->idx < MAX_NUM_CHANS) {
				id = adapter->idx;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
				adapter->rsi_survey[id].time_busy =
				    acs_data->chan_busy_time;
				adapter->rsi_survey[id].time = ACS_TIMEOUT_TIME;
#else
				adapter->rsi_survey[id].channel_time_busy =
				    acs_data->chan_busy_time;
				adapter->rsi_survey[id].channel_time =
				    ACS_TIMEOUT_TIME;
#endif
				adapter->rsi_survey[id].noise =
				    -abs(acs_data->noise_floor_rssi);
				adapter->idx++;
			}
		}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 20, 17)
		rsi_set_event(&common->chan_set_event);
#endif
		break;

	case SET_RX_FILTER:
		rsi_dbg(INFO_ZONE, "RX Filter confirmation.\n");
		break;

	case WAKEUP_SLEEP_REQUEST:
		rsi_dbg(INFO_ZONE, "Wakeup/Sleep confirmation.\n");
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 20, 17)
#ifndef CONFIG_RSI_P2P
		rsi_set_event(&common->mgmt_cfm_event);
#endif
#endif
		return rsi_handle_ps_confirm(adapter, msg);

	case BG_SCAN_PROBE_REQ:
		rsi_dbg(INT_MGMT_ZONE,
			"<==== Received BG Scan Complete Event ===>\n");
		if (common->hwscan_en) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0))
			info.aborted = false;
			ieee80211_scan_completed(adapter->hw, &info);
#else	
			ieee80211_scan_completed(adapter->hw, false);
#endif
			common->hwscan_en = false;
		}
		if (common->hw_scan_cancel)
			rsi_set_event(&common->cancel_hw_scan_event);
		break;

	default:
		rsi_dbg(INFO_ZONE,
			"%s: Invalid TA confirm type : %x\n",
			__func__, sub_type);
		break;
	}
	return 0;

out:
	rsi_dbg(ERR_ZONE,
		"%s: Unable to send pkt/Invalid frame received\n",
		__func__);
	return -EINVAL;
}

/**
 *rsi_handle_card_ready() - This function handles the card ready
 *                       indication from firmware.
 *@common: Pointer to the driver private structure.
 *
 *Return: 0 on success, -1 on failure.
 */
int rsi_handle_card_ready(struct rsi_common *common, u8 *msg)
{
	u8 rsi_standard_mac[3] = {0x00, 0x23, 0xa7};

	switch (common->fsm_state) {
	case FSM_CARD_NOT_READY:
		rsi_dbg(INIT_ZONE, "Card ready indication from Common HAL\n");
		common->common_hal_tx_access = true;
		rsi_set_default_parameters(common);
		if (rsi_send_common_dev_params(common) < 0)
			return -EINVAL;
		common->fsm_state = FSM_COMMON_DEV_PARAMS_SENT;
		break;
	case FSM_COMMON_DEV_PARAMS_SENT:
		rsi_dbg(INIT_ZONE, "Card ready indication from WLAN HAL\n");

		if (common->priv->device_model == RSI_DEV_9116) {
			if ((msg[16] != MAGIC_WORD) ||
			    ((msg[16] == MAGIC_WORD) && (msg[21] == 0x00))) {
				memcpy(common->mac_addr, rsi_standard_mac,
				       RSI_MAC_SUB_LEN);
				get_random_bytes(&common->mac_addr[3],
						 RSI_MAC_SUB_LEN);
			} else {
				memcpy(common->mac_addr, &msg[20],
				       ETH_ALEN);
			}
			common->band =
				(msg[27] == RSI_BAND_CHECK ? NL80211_BAND_5GHZ
				 : NL80211_BAND_2GHZ);
			rsi_hex_dump(INIT_ZONE, "MAC Addr",
				     common->mac_addr, ETH_ALEN);
		}
		/* Get usb buffer status register address */
		common->priv->usb_buffer_status_reg = *(u32 *)&msg[8];
		rsi_dbg(INFO_ZONE, "USB buffer status register = %x\n",
			common->priv->usb_buffer_status_reg);

		if (rsi_load_bootup_params(common)) {
			common->fsm_state = FSM_CARD_NOT_READY;
			return -EINVAL;
		}
		common->fsm_state = FSM_BOOT_PARAMS_SENT;
		break;
	default:
		rsi_dbg(ERR_ZONE,
			"%s: card ready indication in invalid state %d.\n",
			__func__, common->fsm_state);
		return -EINVAL;
	}

	return 0;
}

/*
 * rsi_send_ack_for_ulp_entry() - ULP sleep ack is prepared in this
 * function.
 * @common: Pointer to the device private structure.
 */
int rsi_send_ack_for_ulp_entry(struct rsi_common *common)
{
	struct rsi_ulp_params *ulp_params;
	struct sk_buff *skb;

	skb = dev_alloc_skb(sizeof(*ulp_params));
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, sizeof(*ulp_params));
	ulp_params = (struct rsi_ulp_params *)skb->data;
	ulp_params->desc_word[1] = cpu_to_le16(CONFIRM);
	ulp_params->desc_word[6] = common->ulp_token;
	ulp_params->desc_word[7] = cpu_to_le16(ULP_SLEEP_NOTIFY << 8);
	skb_put(skb, sizeof(struct rsi_ulp_params));
	common->ulp_sleep_ack_sent = true;
	return common->priv->host_intf_ops->write_pkt(common->priv,
						      skb->data,
						      skb->len);
}
EXPORT_SYMBOL_GPL(rsi_send_ack_for_ulp_entry);

/*
 * rsi_resolve_ulp_request() - This function serves the ulp request came
 * from lmac and decides wheather to send ack or not.
 * @common: Pointer to the device private structure.
 */
static void rsi_process_ulp_sleep_notify(struct rsi_common *common, u8 *msg)
{
	if ((msg[12] & BIT(0)) == ULP_SLEEP_ENTRY) {
		rsi_dbg(INFO_ZONE, "%s: ULP sleep entry received\n", __func__);
		down(&common->tx_access_lock);
		common->sleep_entry_received = true;
		common->ulp_token = ULP_TOKEN;
		if (!protocol_tx_access(common)) {
			common->common_hal_tx_access = false;
			rsi_send_ack_for_ulp_entry(common);
		}
		up(&common->tx_access_lock);
	} else {
		rsi_dbg(INFO_ZONE, "%s: ULP sleep exit received\n", __func__);
		common->ulp_token = 0xABCD;
		sleep_exit_recvd(common);
	}
}

/**
 * rsi_mgmt_pkt_recv() - This function processes the management packets
 *			 received from the hardware.
 * @common: Pointer to the driver private structure.
 * @msg: Pointer to the received packet.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_mgmt_pkt_recv(struct rsi_common *common, u8 *msg)
{
	struct rsi_hw *adapter = common->priv;
	s32 msg_len = (le16_to_cpu(*(__le16 *)&msg[0]) & 0x0fff);
	u16 msg_type = msg[2];
	struct ieee80211_vif *vif = adapter->vifs[adapter->sc_nvifs - 1];

	switch (msg_type) {
	case TA_CONFIRM_TYPE:
		return rsi_handle_ta_confirm(common, msg);

	case CARD_READY_IND:
		rsi_dbg(INIT_ZONE, "CARD READY INDICATION FROM WLAN.\n");
		return rsi_handle_card_ready(common, msg);

	case TX_STATUS_IND:
		if (msg[15] == PROBEREQ_CONFIRM) {
			rsi_dbg(INT_MGMT_ZONE,
				"<==== Recv PROBEREQ CONFIRM =====>\n");
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 20, 17)
			rsi_set_event(&common->probe_cfm_event);
#endif
		}
		if ((msg[15] & 0xff) == EAPOL4_CONFIRM) {
			u8 status = msg[12];

			if (status) {	
				if(vif->type == NL80211_IFTYPE_STATION) {
					rsi_dbg(ERR_ZONE, "EAPOL 4 confirm\n");
					common->start_bgscan = 1;
					common->eapol4_confirm = 1;
					if (!rsi_send_block_unblock_frame(common,
									  false))
						common->hw_data_qs_blocked = false;
				}
			}
		}
		break;

	case PS_NOTIFY_IND:
		rsi_dbg(FSM_ZONE, "Powersave notify indication.\n");
		break;

	case SLEEP_NOTIFY_IND:
		rsi_dbg(FSM_ZONE, "Sleep notify indication.\n");
		if (msg[2] == ULP_SLEEP_NOTIFY)
			rsi_process_ulp_sleep_notify(common, msg);
		break;

	case DECRYPT_ERROR_IND:
		rsi_dbg(INFO_ZONE, "Error in decrypt.\n");
		break;

	case DEBUG_IND:
		rsi_dbg(INFO_ZONE, "Debugging indication.\n");
		break;

	case RX_MISC_IND:
		rsi_dbg(INFO_ZONE, "RX misc indication.\n");
		break;

	case HW_BMISS_EVENT:
		rsi_dbg(ERR_ZONE, "<==== Hardware beacon miss event ====>\n");
		rsi_indicate_bcnmiss(common);
		rsi_resume_conn_channel(common->priv,
					adapter->vifs[adapter->sc_nvifs - 1]);
		break;

	case BEACON_EVENT_IND:
		if (!common->init_done)
			return -1;
		if (common->iface_down)
			return -1;
		if (!common->beacon_enabled)
			return -1;
		rsi_dbg(MGMT_DEBUG_ZONE,
			"<==== Beacon Interrupt Received ====>\n");
		rsi_send_beacon(common);
		break;

	case WOWLAN_WAKEUP_REASON:
		rsi_hex_dump(INFO_ZONE, "WoWLAN Wakeup Trigger Pkt",
			     msg, msg_len);
		rsi_dbg(ERR_ZONE, "\n\nWakeup Type: %x\n", msg[15]);
		switch(msg[15]) {
		case UNICAST_MAGIC_PKT:
			rsi_dbg(ERR_ZONE,
				"*** Wakeup for Unicast magic packet ***\n");
			break;
		case BROADCAST_MAGICPKT:
			rsi_dbg(ERR_ZONE,
				"*** Wakeup for Broadcast magic packet ***\n");
			break;
		case EAPOL_PKT:
			rsi_dbg(ERR_ZONE,
				"*** Wakeup for GTK renewal ***\n");
			break;
		case DISCONNECT_PKT:
			rsi_dbg(ERR_ZONE,
				"*** Wakeup for Disconnect ***\n");
			break;
		case HW_BMISS_PKT:
			rsi_dbg(ERR_ZONE,
				"*** Wakeup for HW Beacon miss ***\n");
			break;
		default:
			rsi_dbg(ERR_ZONE,
				"##### Un-intentional Wakeup #####\n");
			break;
		}
#ifdef CONFIG_RSI_11K
		case RADIO_MEAS_RPT:
		rsi_hex_dump(ERR_ZONE, "11K RRM RX CMD FRAME",
			     msg, msg_len);
		common->priv->rrm_enq_state = 0;
		common->priv->rrm_state = RRM_RESP_RCVD;
		rsi_rrm_recv_cmd_frame(common, msg, msg_len);
		rsi_rrm_sched_req(common);
		break;
#endif
	case RX_DOT11_MGMT:
		return rsi_mgmt_pkt_to_core(common, msg, msg_len);

	case FW_ERROR_STATE_IND:
		if (msg[15] == RSI_MAX_BGSCAN_CHANNEL_SUPPORTED) {
			rsi_dbg(ERR_ZONE,
				"*** Bgscan Channel's  greater than 24 Not Supported ***\n");
				return -1;
		} else if (msg[15] == RSI_MAX_BGSCAN_PROBE_REQ_LEN) {
			rsi_dbg(ERR_ZONE,
				"*** Bgscan Probe Request Length is greater than Max Size ***\n");
				return -1;
		}
		break;

	default:
		rsi_dbg(INFO_ZONE, "Cmd Frame Type: %d\n", msg_type);
		break;
	}

	return 0;
}


