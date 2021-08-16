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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include "rsi_main.h"
#include "rsi_gpio.h"

/**
 * gpio_deinit(): to de-initialize gpio
 * @return:
 * @params:
 */
void gpio_deinit(struct rsi_common *common)
{
	gpio_free(common->ulp_gpio_read);
	gpio_free(common->ulp_gpio_write);
}
EXPORT_SYMBOL_GPL(gpio_deinit);

/**
 * gpio_init(): to initialize gpio
 * @return:
 * @params:
 */
void gpio_init(struct rsi_common *common)
{
	int rc = 0;
	char *read_gpio = "device_status";
	char *write_gpio = "host_intention";

	/* gpio_free() is dangerous to use.FIXME*/
	gpio_free(common->ulp_gpio_read);
	gpio_free(common->ulp_gpio_write);
	rc = gpio_request(common->ulp_gpio_write, write_gpio);
	if (rc) {
		rsi_dbg(ERR_ZONE, "%s: %s setup failed with err: %d\n",
			__func__, write_gpio, rc);
		return;
	}
	rc = gpio_request(common->ulp_gpio_read, read_gpio);
	if (rc) {
		rsi_dbg(ERR_ZONE, "%s: %s setup failed with err: %d\n",
			__func__, read_gpio, rc);
		return;
	}
	rc = gpio_direction_output(common->ulp_gpio_write, 0);
	if (rc) {
		rsi_dbg(ERR_ZONE, "%s: failed to set %s direction, err: %d\n",
			__func__, write_gpio, rc);
		return;
	}
	rc = gpio_direction_input(common->ulp_gpio_read);
	if (rc) {
		rsi_dbg(ERR_ZONE, "%s: failed to set %s direction, err: %d\n",
			__func__, read_gpio, rc);
		return;
	}
}
EXPORT_SYMBOL_GPL(gpio_init);

/**
 * set_host_status() - This function is used to toggle host gpio.
 *
 * @value: The value of the host gpio either TRUE or FALSE.
 *
 * Return: None.
 */
void set_host_status(int value, struct rsi_common *common)
{
	gpio_set_value(common->ulp_gpio_write, value);
}
EXPORT_SYMBOL_GPL(set_host_status);

/**
 * get_device_status() - This function is used to read the LMAC gpio to find
 * the LMAC sleep status.
 *
 * Return: True if gpio status high, false if gpio status low.
 */
int get_device_status(struct rsi_common *common)
{
	return gpio_get_value(common->ulp_gpio_read);
}
EXPORT_SYMBOL_GPL(get_device_status);

