/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "system/uptime.h"
// #include "timer.h"
#include "connection/esb.h"
#include "sensor/sensor.h"

#include <zephyr/sys/reboot.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <hal/nrf_gpio.h>
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#if !DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, pwr_gpios) && CONFIG_BOARD_PROMICRO_UF2
#warning "IMU power pins not defined: do not stack IMU on PROMICRO"
#endif
#if !DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, gnd_gpios) && CONFIG_BOARD_PROMICRO_UF2
#warning "IMU gnd pins not defined: do not stack IMU on PROMICRO"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)
#define BUTTON_EXISTS true
#endif

#define ADAFRUIT_BOOTLOADER (CONFIG_BUILD_OUTPUT_UF2 && !CONFIG_BOOTLOADER_MCUBOOT)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, pwr_gpios)
static nrf_gpio_pin_drive_t gpio_drive_from_dt_flags(uint32_t flags)
{
	if (flags & GPIO_OPEN_DRAIN) {
		return NRF_GPIO_PIN_S0D1;
	}
	if (flags & GPIO_OPEN_SOURCE) {
		return NRF_GPIO_PIN_D0S1;
	}
	return NRF_GPIO_PIN_S0S1;
}
#endif

int main(void)
{
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, gnd_gpios)
	uint32_t gnd_psel = NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, gnd_gpios);
	nrf_gpio_cfg(gnd_psel, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
		     NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_pin_clear(gnd_psel);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, pwr_gpios)
	uint32_t pwr_psel = NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, pwr_gpios);
	const uint32_t pwr_flags = DT_GPIO_FLAGS(ZEPHYR_USER_NODE, pwr_gpios);
	const bool pwr_active_low = (pwr_flags & GPIO_ACTIVE_LOW) != 0;
	nrf_gpio_cfg(pwr_psel, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
		     NRF_GPIO_PIN_NOPULL, gpio_drive_from_dt_flags(pwr_flags),
		     NRF_GPIO_PIN_NOSENSE);
	if (pwr_active_low) {
		nrf_gpio_pin_clear(pwr_psel);
	} else {
		nrf_gpio_pin_set(pwr_psel);
	}
#endif
#if IGNORE_RESET && BUTTON_EXISTS
	bool reset_pin_reset = false;
#else
#ifdef NRF_RESET
	bool reset_pin_reset = sys_get_reset_reason() & RESET_RESETREAS_RESETPIN_Msk;
#else
	bool reset_pin_reset = sys_get_reset_reason() & POWER_RESETREAS_RESETPIN_Msk;
#endif
#endif

	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_BOOT); // Boot LED

	uint8_t reboot_counter = reboot_counter_read();
	bool booting_from_shutdown
		= !reboot_counter && (reset_pin_reset || button_read()); // 0 means from user shutdown or failed ram validation

	/* if button is not held after booting from shutdown, power off again
	 * if button press is normal, continue boot
	 * if button is held past the long-hold window, reset pairing only
	 * when multiple-press actions are not enabled
	 */

	if (button_read()) {
		while (button_read()) {
			if (system_uptime_since_boot_ms() > 1000) {
				set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_HIGHEST);
			}
			if (system_uptime_since_boot_ms() > 5000) {
#if CONFIG_USER_EXTRA_ACTIONS
				LOG_INF("Button long hold timeout, continuing boot");
#else
				LOG_INF("Pairing requested");
				esb_reset_pair();
#endif
				break;
			}
			k_msleep(1);
		}
#if USER_SHUTDOWN_ENABLED
		if (system_uptime_since_boot_ms() < 50 && booting_from_shutdown) { // debounce
			sys_request_system_off();
		}
#endif
		if (system_uptime_since_boot_ms() <= 5000) {
			set_led(SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PRIORITY_HIGHEST);
		} else {
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
		}
	} else if (booting_from_shutdown) {
		set_led(SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PRIORITY_BOOT);
	}

	bool docked = dock_read();

	uint8_t reset_mode = -1;

	if (reboot_counter == 0) {
		reboot_counter = 100;
	} else if (reboot_counter > 200) {
		reboot_counter = 200; // How did you get here
	}
	reset_mode = reboot_counter - 100;
	if (reset_pin_reset && !docked) // Count pin resets while not docked
	{
		reboot_counter++;
		reboot_counter_write(reboot_counter);
		LOG_INF("Reset count: %u", reboot_counter);
#if ADAFRUIT_BOOTLOADER                                                                                                \
	&& !(IGNORE_RESET && BUTTON_EXISTS)       // Using Adafruit bootloader, skip DFU if reset button is in use
	sys_skip_dfu(); // Skip DFU
#endif
		k_msleep(1000); // Wait before clearing counter and continuing
	}
	reboot_counter_write(100);
	if (!reset_pin_reset
		&& reset_mode
			   == 0) { // Only need to check once, if the button is pressed again an interrupt is triggered from before
		reset_mode = -1; // Cancel reset_mode (shutdown)
	}

#if USER_SHUTDOWN_ENABLED
	bool charging = chg_read();
	bool charged = stby_read();
	bool plugged = vin_read();

	if (reset_mode == 0 && !booting_from_shutdown && !charging && !charged
		&& !plugged) { // Reset mode user shutdown, only if unplugged and undocked
		sys_user_shutdown();
	}
#endif

	if (!booting_from_shutdown) { // ONESHOT_POWERON automatically sets LED off
		k_usleep(60);
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_BOOT);
	}

	sys_reset_mode(reset_mode);

	return 0;
}
