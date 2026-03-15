/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <hal/nrf_gpio.h>

static int board_promicro_init(void)
{
	/* pull down on P0.13, disables external 3V3 regulator
	 */
	nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 13), NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	/* if external 3V3 regulator is needed, use PULLUP instead of PULLDOWN: nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 13), NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	*/
	return 0;
}

SYS_INIT(board_promicro_init, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);