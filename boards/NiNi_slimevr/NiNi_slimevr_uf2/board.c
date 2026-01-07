/*
 * Copyright (c) 2024 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <hal/nrf_gpio.h>

#ifdef CONFIG_BOARD_NINI_SLIMEVR_UF2_NRF52833_I2C
static int board_nini_slimevr_i2c_init(void)
{
	/*
	 * Pin 28 and Pin 29: Set with pull-up to prevent floating state that can interfere with IMU and MAG
	 */
	nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 28),
		     NRF_GPIO_PIN_DIR_INPUT,
		     NRF_GPIO_PIN_INPUT_CONNECT,
		     NRF_GPIO_PIN_PULLUP,
		     NRF_GPIO_PIN_S0S1,
		     NRF_GPIO_PIN_NOSENSE);

	nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 29),
		     NRF_GPIO_PIN_DIR_INPUT,
		     NRF_GPIO_PIN_INPUT_CONNECT,
		     NRF_GPIO_PIN_PULLUP,
		     NRF_GPIO_PIN_S0S1,
		     NRF_GPIO_PIN_NOSENSE);

	return 0;
}

SYS_INIT(board_nini_slimevr_i2c_init, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif /* CONFIG_BOARD_NINI_SLIMEVR_UF2_NRF52833_I2C */

