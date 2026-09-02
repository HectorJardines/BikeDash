/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "../include/app/csc_sensor.h"
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(main_mod, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define PIN 9
#define GPIO0_NODE DT_NODELABEL(gpio0)

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct device *gpio_dev = DEVICE_DT_GET(GPIO0_NODE);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;
	ret = csc_sens_init();

	while (1) {
		k_sleep(K_MSEC(0xFFFF));
	}

	if (ret) {
		LOG_ERR("FAILED TO INTIIALIZE CSC SENSOR\n\r");
		for (;;);	
	}
}
