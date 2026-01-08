#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>

#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
#include <hal/nrf_gpio.h>
#endif

LOG_MODULE_REGISTER(power_btn, LOG_LEVEL_INF);

/* 1s 长按关机 */
#define LONG_PRESS_MS 1000
#define POLL_MS 10

#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Devicetree alias 'sw0' is not defined. Add: / { aliases { sw0 = &button0; }; };"
#endif

static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

static void configure_wakeup_sense_if_possible(void)
{
#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
    /* 让 GPIO 在 System OFF 下也能作为唤醒源：
     * - 你的按键是上拉 + 按下接地 => 低电平有效
     * - 配置 SENSE LOW
     *
     * 注意：这里直接用 nRF HAL，最“硬”也最可靠。
     */
    uint32_t pin = sw0.pin;
    /* sw0.port 通常是 GPIO0；nRF52 pin 号就是 0..31 */
    nrf_gpio_cfg_sense_input(pin,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_SENSE_LOW);
#endif
}

void power_button_longpress_start(void)
{
    int ret = gpio_pin_configure_dt(&sw0, GPIO_INPUT);
    if (ret) {
        LOG_ERR("SW0 configure failed: %d", ret);
        return;
    }

    /* 先把唤醒源配置好（可选但推荐） */
    configure_wakeup_sense_if_possible();

    LOG_INF("Power button long-press armed on P%d.%d (hold %dms to power off)",
            0, sw0.pin, LONG_PRESS_MS);

    int64_t pressed_since = -1;

    while (1) {
        /* gpio-keys 里你配了 ACTIVE_LOW，所以按下时读到 0 */
        int v = gpio_pin_get_dt(&sw0);
        bool pressed = (v == 0);

        int64_t now = k_uptime_get();

        if (pressed) {
            if (pressed_since < 0) {
                pressed_since = now;
            } else if ((now - pressed_since) >= LONG_PRESS_MS) {
                LOG_WRN("Long press detected -> sys_poweroff()");
                k_sleep(K_MSEC(50)); /* 给日志一点时间刷出 */
                sys_poweroff();
                /* 理论上不会返回 */
                pressed_since = -1;
            }
        } else {
            pressed_since = -1;
        }

        k_sleep(K_MSEC(POLL_MS));
    }
}
