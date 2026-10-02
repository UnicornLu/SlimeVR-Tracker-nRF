#include "globals.h"

#include <math.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "led.h"
#include "led_strip_fade.h"
#include "led_sync.h"
#if CONFIG_LED_NETWORK_SYNC
#include "connection/esb.h"
#endif
/* Three-channel rendering only needs these; keeping them conditional also keeps
 * the host LED test harnesses free of extra shims. */
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led0)) && DT_NODE_EXISTS(DT_ALIAS(pwm_led1)) && DT_NODE_EXISTS(DT_ALIAS(pwm_led2)) && defined(CONFIG_LED_TRI_COLOR)
#include "sensor/sensor.h"
#include "system/esb_ota.h"
#endif

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static void led_thread(void);
K_THREAD_DEFINE(led_thread_id, CONFIG_LED_THREAD_STACK_SIZE, led_thread, NULL, NULL, NULL, LED_THREAD_PRIORITY, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_en_gpios)
#define LED_EN_EXISTS true
static const struct gpio_dt_spec led_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_en_gpios);
#endif

#if CONFIG_LED_STRIP
#define LED_STRIP_EXISTS true
#include <zephyr/drivers/led_strip.h>
#define STRIP_NODE DT_ALIAS(led_strip)
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_strip_fade led_fade;
static bool strip_error_logged;
static int64_t strip_error_log_ticks;
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#ifndef LED_EXISTS
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define LED0_EXISTS true
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#endif
#ifndef LED_EXISTS
#ifndef LED_STRIP_EXISTS
#warning "LED GPIO does not exist"
// static const struct gpio_dt_spec led = {0};
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
#define LED1_EXISTS true
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
#define LED2_EXISTS true
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
#define LED3_EXISTS true
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pwm_led0))
#define PWM_LED_EXISTS true
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
#else
#ifndef LED_STRIP_EXISTS
#warning "PWM LED node does not exist"
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led1))
#define PWM_LED1_EXISTS true
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led2))
#define PWM_LED2_EXISTS true
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
#endif

/* Magnetometer calibration progress (0-10000). Written by the calibration
 * thread and read by the LED worker on three-channel boards; defined for every
 * build so cal_mag.c links on all boards. */
volatile uint16_t led_cal_progress;

#if LED_EXISTS || LED_STRIP_EXISTS || PWM_LED_EXISTS
static struct k_spinlock led_request_lock;
K_SEM_DEFINE(led_changed, 0, 1);
K_SEM_DEFINE(led_quiesced, 0, 1);
K_MUTEX_DEFINE(led_shutdown_lock);
static bool shutdown_pending;
static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH]
	= {[0 ...(SYS_LED_PATTERN_DEPTH - 1)] = SYS_LED_PATTERN_OFF};
static uint32_t led_generations[SYS_LED_PATTERN_DEPTH];

static int led_pin_init(void)
{
	LOG_DBG("led_pin_init");
#ifdef LED_STRIP_EXISTS
	led_strip_fade_reset(&led_fade);
#endif
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 0);
#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	gpio_pin_set_dt(&led0, 0);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
	gpio_pin_set_dt(&led1, 0);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
	gpio_pin_set_dt(&led2, 0);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	gpio_pin_set_dt(&led3, 0);
#endif
	return 0;
}

SYS_INIT(led_pin_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void led_pin_reset(void)
{
	LOG_DBG("led_pin_reset");
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_DISCONNECTED);
#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_DISCONNECTED);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_DISCONNECTED);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_DISCONNECTED);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_DISCONNECTED);
#endif
}

static void led_suspend(void)
{
	LOG_DBG("led_suspend");
#ifdef LED_STRIP_EXISTS
	pm_device_action_run(strip, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
	led_pin_reset();
	// disable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 0);
#endif
}

static void led_resume(void)
{
	LOG_DBG("led_resume");
	// enable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 1);
#if LED_STRIP_EXISTS
	/* Rail settling margin, only on the worker's off -> on transition. */
	k_msleep(2);
#endif
#endif
#ifdef LED_STRIP_EXISTS
	pm_device_action_run(strip, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_RESUME);
#endif
	led_pin_init();
}

#ifdef LED_STRIP_EXISTS
#define LED_RGB_COLOR
#else
#ifdef CONFIG_LED_RGB_COLOR
#define LED_RGB_COLOR
#define LED_RG_COLOR
#endif

#if PWM_LED_EXISTS && PWM_LED1_EXISTS && PWM_LED2_EXISTS
#define LED_TRI_COLOR
#else
#undef LED_RGB_COLOR
#undef LED_TRI_COLOR
#if PWM_LED_EXISTS && PWM_LED1_EXISTS
#define LED_DUAL_COLOR
#else
#undef LED_RG_COLOR
#undef LED_DUAL_COLOR
#endif
#endif
#endif

/* Three-channel rendering is opt-in per board: the hardware has to expose all
 * three PWM LED aliases (LED_TRI_COLOR) and the board has to select the
 * tri-color mapping (CONFIG_LED_TRI_COLOR). Every other board keeps the
 * single-arbitration path below, including the network-synced phases. */
#if defined(LED_TRI_COLOR) && defined(CONFIG_LED_TRI_COLOR)
#define LED_TRI_RENDER 1
#endif

/* Color tables and led_pin_set() serve the single-arbitration path only; the
 * three-channel path drives the three PWM channels directly. */
#ifndef LED_TRI_RENDER
#ifdef LED_RGB_COLOR
static int led_pwm_period[5][3] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G, CONFIG_LED_DEFAULT_COLOR_B}, // Default
	{0, 10000, 0},                                                                        // Success
	{10000, 0, 0},                                                                        // Error
	{8000, 2000, 0},                                                                      // Charging
	{0, 0, 10000},                                                                        // Pairing
};
#elif defined(LED_TRI_COLOR)
static int led_pwm_period[5][3] = {
	{0, 0, 10000},   // Default
	{0, 10000, 0},   // Success
	{10000, 0, 0},   // Error
	{6000, 4000, 0}, // Charging
	{0, 0, 10000},   // Pairing
};
#elif defined(LED_RG_COLOR)
static int led_pwm_period[5][2] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G}, // Default
	{0, 10000},                                               // Success
	{10000, 0},                                               // Error
	{8000, 2000},                                             // Charging
	{4000, 6000},                                             // Pairing
};
#elif defined(LED_DUAL_COLOR)
static int led_pwm_period[5][2] = {
	{0, 10000},   // Default
	{0, 10000},   // Success
	{10000, 0},   // Error
	{6000, 4000}, // Charging
	{0, 10000},   // Pairing
};
#else
static int led_pwm_period[5][1] = {
	{10000}, // Default
	{10000}, // Success
	{10000}, // Error
	{10000}, // Charging
	{10000}, // Pairing
};
#endif

// Using brightness and value if PWM is supported, otherwise value is coerced to on/off
// TODO: use computed constants for high/low brightness and color values
#if CONFIG_LED_STRIP_TIMING_LOG
/*
 * Diagnostic for the fade smoothness: the sub-level carry relies on the fade
 * patterns refreshing at their nominal rate, so report the actual frame period
 * (between two strip updates) and how long the update itself blocked, once per
 * 1000 frames - about every 5 s while a pattern is fading.
 */
static void led_strip_timing_note(uint32_t start_ticks)
{
	static uint32_t frames;
	static uint32_t previous_start;
	static uint32_t period_min_us = UINT32_MAX;
	static uint32_t period_max_us;
	static uint64_t period_sum_us;
	static uint32_t update_min_us = UINT32_MAX;
	static uint32_t update_max_us;
	static uint64_t update_sum_us;
	static uint32_t late_frames;

	uint32_t now = k_uptime_ticks();
	uint32_t update_us = k_ticks_to_us_floor32(now - start_ticks);

	if (frames > 0) {
		uint32_t period_us = k_ticks_to_us_floor32(start_ticks - previous_start);

		if (period_us < period_min_us) {
			period_min_us = period_us;
		}
		if (period_us > period_max_us) {
			period_max_us = period_us;
		}
		period_sum_us += period_us;
		if (period_us > 20000) {
			late_frames++;
		}
	}
	if (update_us < update_min_us) {
		update_min_us = update_us;
	}
	if (update_us > update_max_us) {
		update_max_us = update_us;
	}
	update_sum_us += update_us;
	previous_start = start_ticks;
	frames++;

	if (frames >= 1000) {
		LOG_INF("strip frames %u: period %u/%u/%u us (min/avg/max), late(>20ms) %u; "
			"update blocked %u/%u/%u us",
			frames, period_min_us, (uint32_t)(period_sum_us / (frames - 1)),
			period_max_us, late_frames, update_min_us,
			(uint32_t)(update_sum_us / frames), update_max_us);
		frames = 0;
		period_min_us = UINT32_MAX;
		period_max_us = 0;
		period_sum_us = 0;
		update_min_us = UINT32_MAX;
		update_max_us = 0;
		update_sum_us = 0;
		late_frames = 0;
	}
}
#endif


static bool led_pin_set(enum sys_led_color color, int brightness_pptt, int value_pptt)
{
	LOG_DBG("led_pin_set: color %d, brightness %d, value %d", color, brightness_pptt, value_pptt);
	if (brightness_pptt < 0) {
		brightness_pptt = 0;
	} else if (brightness_pptt > 10000) {
		brightness_pptt = 10000;
	}
	if (value_pptt < 0) {
		value_pptt = 0;
	} else if (value_pptt > 10000) {
		value_pptt = 10000;
	}
#if LED_STRIP_EXISTS
	static struct led_rgb pixel[1];
	value_pptt = value_pptt * brightness_pptt / 10000;
	value_pptt = value_pptt * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000;
	struct led_strip_fade next_fade = led_fade;
	const struct led_rgb requested = {
		.r = led_strip_fade_next(&next_fade, led_pwm_period[color][0], value_pptt, 0),
		.g = led_strip_fade_next(&next_fade, led_pwm_period[color][1], value_pptt, 1),
		.b = led_strip_fade_next(&next_fade, led_pwm_period[color][2], value_pptt, 2),
	};
	pixel[0] = requested;
#if CONFIG_LED_STRIP_TIMING_LOG
	uint32_t led_frame_start = k_uptime_ticks();
#endif
	int err = led_strip_update_rgb(strip, pixel, 1);
#if CONFIG_LED_STRIP_TIMING_LOG
	led_strip_timing_note(led_frame_start);
#endif
	if (err < 0) {
		int64_t now = k_uptime_ticks();
		if (!strip_error_logged ||
		    now - strip_error_log_ticks >= CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
			LOG_ERR("strip RGB %u/%u/%u update failed: %d",
				requested.r, requested.g, requested.b, err);
			strip_error_logged = true;
			strip_error_log_ticks = now;
		}
		return false;
	}
	/* A rejected frame must not spend the fractional brightness carry. */
	led_fade = next_fade;
#elif PWM_LED_EXISTS
	value_pptt = value_pptt * brightness_pptt / 10000;
	value_pptt = value_pptt * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000;
	// only supporting color if PWM is supported
	pwm_set_pulse_dt(&pwm_led, pwm_led.period / 10000 * (led_pwm_period[color][0] * value_pptt / 10000));
#if PWM_LED1_EXISTS
	pwm_set_pulse_dt(&pwm_led1, pwm_led1.period / 10000 * (led_pwm_period[color][1] * value_pptt / 10000));
#if PWM_LED2_EXISTS
	pwm_set_pulse_dt(&pwm_led2, pwm_led2.period / 10000 * (led_pwm_period[color][2] * value_pptt / 10000));
#endif
#endif
#else
	gpio_pin_set_dt(&led, value_pptt > 5000);
#endif
	return true;
}
#endif /* !LED_TRI_RENDER */
#endif

/* =====================================================================
 * Three-channel rendering (CONFIG_LED_TRI_COLOR boards with three PWM LEDs).
 *
 * Red, green and blue are arbitrated and phased independently, so the green
 * work breathe and the blue link heartbeat are visible at the same time.
 * The daily/debug assignment tables live in led_compute(); the physical ->
 * semantic routing lives in led_phys_color[] (ledmap console command).
 * Request slots, generations, the wake semaphore and the shutdown barrier are
 * shared with the single-arbitration path below.
 *
 * Timing: grid patterns (work breathe, link heartbeat, unpaired) run on a wall
 * clock that follows receiver time when CONFIG_LED_NETWORK_SYNC is enabled, so
 * trackers on one receiver share the phase. Finite events (one-shots, previews,
 * calibration, ping) always keep local timing.
 * ===================================================================== */
#ifdef LED_TRI_RENDER

enum led_ch { LED_CH_R = 0, LED_CH_G = 1, LED_CH_B = 2, LED_CH_COUNT };

#define CH_R (1u << LED_CH_R)
#define CH_G (1u << LED_CH_G)
#define CH_B (1u << LED_CH_B)
#define CH_ALL (CH_R | CH_G | CH_B)

#define LED_DIM_AFTER_MS 180000 /* resting dim: halve all output after 3 minutes */

struct led_channel {
	enum sys_led_pattern pattern; // pattern currently owning this channel (OFF = dark)
	int owner;                    // request slot owning it (-1 = none)
	uint32_t generation;          // slot generation the phase was started with
	uint32_t state;               // phase state (one-shot step counter)
	uint32_t last_value;          // last rendered value (0-10000)
};

static struct led_channel chans[LED_CH_COUNT];
static enum led_display_mode led_mode = LED_MODE_DAILY;
/* Default 25%: every effect output is scaled by this and nothing exceeds it. */
static uint16_t led_brightness_pptt = 2500;

/* Physical position -> semantic color (0 = R, 1 = G, 2 = B). Identity by
 * default: pwm-led0/1/2 drive red/green/blue, matching the board device tree. */
static uint8_t led_phys_color[LED_CH_COUNT] = {LED_CH_R, LED_CH_G, LED_CH_B};

/* The three PWM channels in physical (LED1/LED2/LED3) order. */
static const struct pwm_dt_spec *const led_pwms[LED_CH_COUNT] = {&pwm_led, &pwm_led1, &pwm_led2};

static uint8_t bind_of_semantic(enum led_ch sem)
{
	for (uint8_t i = 0; i < LED_CH_COUNT; i++) {
		if (led_phys_color[i] == (uint8_t)sem) {
			return i;
		}
	}
	return (uint8_t)sem; /* defensive: a damaged table falls back to identity */
}

static uint32_t led_timebase_ms(void)
{
#if CONFIG_LED_NETWORK_SYNC
	static uint32_t held_delta_ms;
	static bool have_delta;
	uint32_t local = 0;
	uint32_t network = 0;

	if (esb_get_status_clock(&local, &network)) {
		uint32_t local_ms = (uint32_t)(((uint64_t)local * 1000U) / LED_SYNC_HZ);
		held_delta_ms = (uint32_t)(((uint64_t)network * 1000U) / LED_SYNC_HZ) - local_ms;
		have_delta = true;
		return local_ms + held_delta_ms;
	}
	if (have_delta) {
		/* Keep extrapolating from the last known offset: losing the receiver
		 * clock must not jump the grid. The 32-bit network clock wraps every
		 * 36h24m32s and may distort one interval at that boundary. */
		return k_uptime_get_32() + held_delta_ms;
	}
#endif
	return k_uptime_get_32();
}

/* Pattern -> channel mask (assignment table; both display modes share it). */
static uint32_t pattern_mask(enum sys_led_pattern p)
{
	switch (p) {
	case SYS_LED_PATTERN_OFF_FORCE:
	case SYS_LED_PATTERN_OFF:
	case SYS_LED_PATTERN_ONESHOT_POWEROFF:
	case SYS_LED_PATTERN_ERROR_A:
	case SYS_LED_PATTERN_ERROR_B:
	case SYS_LED_PATTERN_ERROR_C:
	case SYS_LED_PATTERN_ERROR_D:
		return CH_ALL; /* global off / errors own all three */
	case SYS_LED_PATTERN_ON:
	case SYS_LED_PATTERN_ONESHOT_PING:
	case SYS_LED_PATTERN_SHORT:
	case SYS_LED_PATTERN_CONNECT_HEARTBEAT:
		return CH_B; /* blue = link and immediate feedback */
	case SYS_LED_PATTERN_ONESHOT_POWERON:
		return CH_G;
	case SYS_LED_PATTERN_LONG:
	case SYS_LED_PATTERN_FLASH:
	case SYS_LED_PATTERN_ONESHOT_PROGRESS:
	case SYS_LED_PATTERN_ONESHOT_COMPLETE:
	case SYS_LED_PATTERN_ON_PERSIST:
	case SYS_LED_PATTERN_ACTIVE_PERSIST:
		return CH_G; /* green = vitals and confirmations */
	case SYS_LED_PATTERN_LONG_PERSIST:
	case SYS_LED_PATTERN_PULSE_PERSIST:
		return CH_R; /* red = power domain */
	case SYS_LED_PATTERN_DFU:
		return CH_B; /* blue = update domain */
	case SYS_LED_PATTERN_CAL_PROGRESS:
		return CH_R | CH_G; /* red -> green progress */
	case SYS_LED_PATTERN_FAST_GREEN:
	case SYS_LED_PATTERN_FAST_BLUE:
	case SYS_LED_PATTERN_FAST_RED:
		return CH_ALL; /* previews suppress everything else, target channel renders */
	case SYS_LED_PATTERN_ONESHOT_X2:
		return CH_G;
	case SYS_LED_PATTERN_ONESHOT_X3:
		return CH_B;
	default:
		return CH_ALL;
	}
}

/* Triangular breath: phase in [0,period) rises to peak over "up", falls back
 * over "down", and is dark otherwise. */
static uint32_t breath_shape(uint32_t phase, uint32_t period, uint32_t up, uint32_t down, uint32_t peak)
{
	ARG_UNUSED(period);
	if (phase < up) {
		return peak * phase / up;
	}
	if (phase < up + down) {
		return peak * (up + down - phase) / down;
	}
	return 0;
}

/* Retire every slot still holding this pattern. Only the LED worker does this,
 * and the caller re-reads the slots every frame, so clearing by value is safe
 * here (a replacement request is picked up on the next frame anyway). */
static void led_oneshot_done(enum sys_led_pattern done)
{
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) {
		if (led_patterns[i] == done) {
			led_patterns[i] = SYS_LED_PATTERN_OFF;
			led_generations[i]++;
		}
	}
	k_spin_unlock(&led_request_lock, key);
}

/* Render one channel. "grid_now" is the synced wall clock used by the grid
 * patterns; "now" is the local clock used by finite effects. */
static uint32_t led_compute(enum led_ch ch, enum sys_led_pattern p, uint32_t *state,
			    uint32_t grid_now, uint32_t *step_ms)
{
	const uint32_t now = k_uptime_get_32();
	const bool daily = (led_mode == LED_MODE_DAILY);
	uint32_t v = 0;
	uint32_t st = 5;

	switch (p) {
	case SYS_LED_PATTERN_OFF_FORCE:
	case SYS_LED_PATTERN_OFF:
		v = 0;
		st = 100;
		break;

	case SYS_LED_PATTERN_ON: /* button hold feedback (blue is weak on 1k, start at 80%) */
		v = daily ? 8000 : 10000;
		st = 200;
		break;

	case SYS_LED_PATTERN_ACTIVE_PERSIST: { /* green: normal operation */
		if (daily) {
			/* 20 s grid: 1.2 s up + 1.2 s down at 0 s, peak = full scale
			 * (this peak is what ledbright scales). */
			v = breath_shape(grid_now % 20000, 20000, 1200, 1200, 10000);
			st = 20;
		} else {
			v = (now % 10000) < 300 ? 10000 : 0; /* 300 ms blip every 10 s */
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_CONNECT_HEARTBEAT: { /* blue: link heartbeat, opposite the green grid */
		uint32_t phase = (grid_now + 10000) % 20000;
		if (daily) {
			v = breath_shape(phase, 20000, 1200, 1200, 5000);
			st = 20;
		} else {
			v = phase < 300 ? 10000 : 0;
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_SHORT: { /* blue: unpaired / searching */
		if (daily) {
			/* Double heartbeat at 10 s and 13 s on the same 20 s grid. */
			uint32_t phase = grid_now % 20000;
			if (phase >= 10000 && phase < 12400) {
				v = breath_shape(phase - 10000, 2400, 1200, 1200, 5000);
			} else if (phase >= 13000 && phase < 15400) {
				v = breath_shape(phase - 13000, 2400, 1200, 1200, 5000);
			}
			st = 20;
		} else {
			v = (now % 1000) < 100 ? 10000 : 0; /* 100/900 flash */
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_LONG:
		v = (now % 1000) < 500 ? 10000 : 0;
		st = 50;
		break;
	case SYS_LED_PATTERN_FLASH:
		v = (now % 400) < 200 ? 10000 : 0;
		st = 40;
		break;

	case SYS_LED_PATTERN_ONESHOT_POWERON: { /* boot confirmation */
		if (*state == 0) {
			*state = now + 1; /* store start+1; 0 means "not started" */
		}
		uint32_t elapsed = (now + 1) - *state;
		if (daily) {
			if (elapsed < 1200) {
				v = 10000;
				st = 20;
			} else {
				led_oneshot_done(p);
				v = 0;
				st = 100;
			}
		} else {
			v = ((elapsed / 200) % 2) ? 0 : 10000;
			if (elapsed >= 1400) {
				led_oneshot_done(p);
			}
			st = 200;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_POWEROFF: { /* full-color fade out before power removal */
		uint32_t i = (*state)++;
		if (i == 0) {
			v = 0;
			st = 100;
		} else if (i <= 200) {
			v = (201 - i) * 50; /* 10000 -> 0, one step per 5 ms */
			st = 5;
		} else {
			set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
			v = 0;
			st = 100;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_PROGRESS: { /* confirmation: single fade or 2 flashes */
		uint32_t i = (*state)++;
		if (daily) {
			if (i <= 60) { /* 1.2 s: 600 ms up + 600 ms down */
				v = (i <= 30) ? 7000 * i / 30 : 7000 * (60 - i) / 30;
				st = 20;
			} else {
				led_oneshot_done(p);
				st = 100;
			}
		} else {
			v = !(i % 2) * 10000;
			if (i == 5) {
				led_oneshot_done(p);
			}
			st = 200;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_COMPLETE: { /* success: full fade or 4 flashes */
		uint32_t i = (*state)++;
		if (daily) {
			if (i <= 60) { /* 1.2 s */
				v = (i <= 30) ? 10000 * i / 30 : 10000 * (60 - i) / 30;
				st = 20;
			} else {
				led_oneshot_done(p);
				st = 100;
			}
		} else {
			v = !(i % 2) * 10000;
			if (i == 9) {
				led_oneshot_done(p);
			}
			st = 200;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_PING: {
		uint32_t i = (*state)++;
		v = (i % 2) * 10000;
		if (i == 20) {
			led_oneshot_done(p);
		}
		st = 200;
		break;
	}

	case SYS_LED_PATTERN_FAST_GREEN: /* long-press zone previews (no callers yet) */
	case SYS_LED_PATTERN_FAST_BLUE:
	case SYS_LED_PATTERN_FAST_RED: {
		enum led_ch target = (p == SYS_LED_PATTERN_FAST_GREEN) ? LED_CH_G :
				     (p == SYS_LED_PATTERN_FAST_BLUE) ? LED_CH_B : LED_CH_R;
		if (ch == target) {
			v = (now % 200) < 100 ? 10000 : 0;
			st = 20;
		} else {
			v = 0; /* only the target color is visible */
			st = 100;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_X2: /* click confirmations (no callers yet) */
	case SYS_LED_PATTERN_ONESHOT_X3: {
		if (*state == 0) {
			*state = now + 1;
		}
		uint32_t elapsed = (now + 1) - *state;
		uint32_t total = (p == SYS_LED_PATTERN_ONESHOT_X2) ? 600 : 900;
		if (elapsed < total) {
			v = (elapsed % 300) < 150 ? 10000 : 0;
			st = 50;
		} else {
			led_oneshot_done(p);
			v = 0;
			st = 100;
		}
		break;
	}

	case SYS_LED_PATTERN_ON_PERSIST: /* charged: green steady */
		v = daily ? 4000 : 6000;
		st = 500;
		break;

	case SYS_LED_PATTERN_PULSE_PERSIST: { /* charging: dim red micro-breathe */
		if (daily) {
			uint32_t wob = breath_shape(now % 3000, 3000, 1500, 1500, 500);
			v = 1200 + wob / 2; /* 12% +- 2.5%; red draws far more current */
			st = 40;
		} else {
			v = 2000;
			st = 200;
		}
		break;
	}

	case SYS_LED_PATTERN_LONG_PERSIST: { /* low battery: weak red breathing / double flash */
		const uint32_t peak = 800;
		if (daily) {
			v = breath_shape(now % 6000, 6000, 750, 750, peak);
			st = 20;
		} else {
			uint32_t phase = now % 1050;
			bool on = (phase < 150) || (phase >= 300 && phase < 450);
			v = on ? peak : 0;
			st = 30;
		}
		break;
	}

	case SYS_LED_PATTERN_DFU: { /* OTA: blue fast breathing (1.2 s) / fast flash */
		if (daily) {
			v = 7000 * breath_shape(now % 1200, 1200, 600, 600, 10000) / 10000;
			st = 15;
		} else {
			v = (now % 200) < 100 ? 7000 : 0;
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_CAL_PROGRESS: { /* magnetometer calibration: red -> green */
		uint32_t prog = led_cal_progress;
		if (prog > 10000) {
			prog = 10000;
		}
		/* Red is 4-5x more efficient than green on 1k: halve the red leg so the
		 * start is not blinding, keep green at full for visibility. */
		uint32_t base = (ch == LED_CH_R) ? (10000 - prog) / 2 : prog;
		if (daily) {
			uint32_t wob = breath_shape(now % 2000, 2000, 1000, 1000, 1000);
			v = base * (9000 + wob) / 10000; /* +-10% micro-breathe */
			st = 20;
		} else {
			v = ((now % 1000) < 500) ? base : 0; /* color is the progress bar */
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_ERROR_A:
	case SYS_LED_PATTERN_ERROR_B:
	case SYS_LED_PATTERN_ERROR_C:
	case SYS_LED_PATTERN_ERROR_D: { /* errors own all three channels */
		if (daily) {
			if (ch == LED_CH_R) {
				v = breath_shape(now % 5000, 5000, 600, 600, 4000);
			}
			st = 20;
		} else {
			uint32_t seg = (now / 500) % 3;
			v = (seg == (uint32_t)ch) ? 10000 : 0;
			st = 50;
		}
		break;
	}

	default:
		v = 0;
		st = 100;
		break;
	}

	*step_ms = st;
	return v;
}

/* Lowest-numbered slot whose mask claims this channel; OFF yields. */
static enum sys_led_pattern resolve_channel(const enum sys_led_pattern *slots, enum led_ch ch, int *owner)
{
	for (int prio = 0; prio < SYS_LED_PATTERN_DEPTH; prio++) {
		enum sys_led_pattern p = slots[prio];
		if (p == SYS_LED_PATTERN_OFF) {
			continue;
		}
		if (pattern_mask(p) & (1u << ch)) {
			*owner = prio;
			return p;
		}
	}
	*owner = -1;
	return SYS_LED_PATTERN_OFF;
}

static void led_snapshot(enum sys_led_pattern *slots, uint32_t *generations)
{
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	memcpy(slots, led_patterns, sizeof(led_patterns));
	memcpy(generations, led_generations, sizeof(led_generations));
	k_spin_unlock(&led_request_lock, key);
}

/* Apply the runtime brightness (and the resting dim) and route each semantic
 * channel to its physical PWM position. */
static void led_channels_apply(void)
{
	uint32_t br = led_brightness_pptt;

	/* Halve everything after 3 minutes without meaningful motion; any motion
	 * restores it immediately. OTA is exempt so progress stays readable. */
	if (sensor_ms_since_motion() > LED_DIM_AFTER_MS && !esb_ota_is_active()) {
		br = br / 2;
	}
	for (int sem = 0; sem < LED_CH_COUNT; sem++) {
		uint32_t v = chans[sem].last_value * br / 10000;
		uint8_t phys = bind_of_semantic((enum led_ch)sem);

		if (v > 10000) {
			v = 10000;
		}
		if (phys >= LED_CH_COUNT) {
			phys = sem;
		}
		const struct pwm_dt_spec *spec = led_pwms[phys];
		pwm_set_pulse_dt(spec, spec->period / 10000 * v);
	}
}

static void led_thread(void)
{
	bool powered = false;

	/* Restore display preferences (0xFF after memset means "not initialized"). */
	if (retained->led_mode == (uint8_t)LED_MODE_DEBUG) {
		led_mode = LED_MODE_DEBUG;
	}
	if (retained->led_bright <= 100) {
		led_brightness_pptt = (uint16_t)(retained->led_bright * 100);
	}
	if (retained->led_bind[0] <= LED_CH_B && retained->led_bind[1] <= LED_CH_B &&
	    retained->led_bind[2] <= LED_CH_B) {
		memcpy(led_phys_color, retained->led_bind, LED_CH_COUNT);
	}
	for (int i = 0; i < LED_CH_COUNT; i++) {
		if (!device_is_ready(led_pwms[i]->dev)) {
			LOG_ERR("LED pwm device %d not ready", i);
		}
	}

	for (;;) {
		enum sys_led_pattern slots[SYS_LED_PATTERN_DEPTH];
		uint32_t generations[SYS_LED_PATTERN_DEPTH];
		k_spinlock_key_t key = k_spin_lock(&led_request_lock);
		bool shutting_down = shutdown_pending;
		k_spin_unlock(&led_request_lock, key);

		if (shutting_down) {
			if (powered) {
				for (int i = 0; i < LED_CH_COUNT; i++) {
					const struct pwm_dt_spec *spec = led_pwms[i];
					pwm_set_pulse_dt(spec, 0);
				}
				led_suspend();
				powered = false;
			}
			key = k_spin_lock(&led_request_lock);
			led_patterns[SYS_LED_PRIORITY_HIGHEST] = SYS_LED_PATTERN_OFF_FORCE;
			led_generations[SYS_LED_PRIORITY_HIGHEST]++;
			shutdown_pending = false;
			k_spin_unlock(&led_request_lock, key);
			k_sem_give(&led_quiesced);
			continue;
		}

		led_snapshot(slots, generations);
		const uint32_t grid_now = led_timebase_ms();
		uint32_t min_step = 100;
		bool on = false;

		for (int ch = 0; ch < LED_CH_COUNT; ch++) {
			int owner = -1;
			enum sys_led_pattern p = resolve_channel(slots, (enum led_ch)ch, &owner);
			uint32_t generation = (owner >= 0) ? generations[owner] : 0;
			uint32_t step = 100;

			if (p != chans[ch].pattern || owner != chans[ch].owner ||
			    generation != chans[ch].generation) {
				/* A new owner restarts only this channel's phase. A repeated
				 * request for the same pattern does not restart the grid. */
				chans[ch].pattern = p;
				chans[ch].owner = owner;
				chans[ch].generation = generation;
				chans[ch].state = 0;
			}
			chans[ch].last_value = led_compute((enum led_ch)ch, p, &chans[ch].state,
							   grid_now, &step);
			if (step < min_step) {
				min_step = step;
			}
			if (p > SYS_LED_PATTERN_OFF) {
				on = true;
			}
		}

		if (!on) {
			if (powered) {
				for (int i = 0; i < LED_CH_COUNT; i++) {
					const struct pwm_dt_spec *spec = led_pwms[i];
					pwm_set_pulse_dt(spec, 0);
				}
				led_suspend();
				powered = false;
			}
			k_sem_take(&led_changed, K_FOREVER);
			continue;
		}
		if (!powered) {
			led_resume(); /* PWM devices must be active before rendering */
			powered = true;
		}
		led_channels_apply();
		/* Wake early on a new request, otherwise move to the next frame. */
		(void)k_sem_take(&led_changed, K_MSEC(min_step));
	}
}

void set_led(enum sys_led_pattern pattern, int priority)
{
	if (priority < 0 || priority >= SYS_LED_PATTERN_DEPTH) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	if (pattern == SYS_LED_PATTERN_OFF_FORCE) {
		/* OFF_FORCE suppresses every other request, whatever the caller. */
		for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) {
			led_patterns[i] = SYS_LED_PATTERN_OFF;
		}
		led_patterns[SYS_LED_PRIORITY_HIGHEST] = pattern;
		led_generations[SYS_LED_PRIORITY_HIGHEST]++;
	} else if (led_patterns[priority] != pattern) {
		led_patterns[priority] = pattern;
		led_generations[priority]++;
	}
	k_spin_unlock(&led_request_lock, key);
	/* Never self-wake from the worker: the frame loop would spin. */
	if (k_current_get() != led_thread_id) {
		k_sem_give(&led_changed);
	}
}

void led_shutdown(void)
{
	/* Shutdown callers may overlap; the driver finishes its transfer before
	 * the barrier releases. Not valid from the LED worker or an ISR. */
	k_mutex_lock(&led_shutdown_lock, K_FOREVER);
	k_sem_reset(&led_quiesced);
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	shutdown_pending = true;
	k_spin_unlock(&led_request_lock, key);
	k_sem_give(&led_changed);
	k_sem_take(&led_quiesced, K_FOREVER);
	k_mutex_unlock(&led_shutdown_lock);
}

void set_led_mode(enum led_display_mode mode)
{
	led_mode = (mode == LED_MODE_DEBUG) ? LED_MODE_DEBUG : LED_MODE_DAILY;
	retained->led_mode = (uint8_t)led_mode;
	retained_update();
}

enum led_display_mode get_led_mode(void)
{
	return led_mode;
}

void set_led_brightness(uint8_t percent)
{
	if (percent > 100) {
		percent = 100;
	}
	led_brightness_pptt = (uint16_t)(percent * 100); /* 0 = all off */
	retained->led_bright = percent;
	retained_update();
}

uint8_t get_led_brightness(void)
{
	return (uint8_t)(led_brightness_pptt / 100);
}

bool set_led_binding(const uint8_t phys_colors[LED_CH_COUNT])
{
	bool seen[LED_CH_COUNT] = {false, false, false};

	for (int i = 0; i < LED_CH_COUNT; i++) {
		if (phys_colors[i] >= LED_CH_COUNT || seen[phys_colors[i]]) {
			return false; /* out of range or duplicate (e.g. all three blue) */
		}
		seen[phys_colors[i]] = true;
	}
	memcpy(led_phys_color, phys_colors, LED_CH_COUNT);
	memcpy(retained->led_bind, phys_colors, LED_CH_COUNT);
	retained_update();
	return true;
}

void get_led_binding(uint8_t phys_colors[LED_CH_COUNT])
{
	memcpy(phys_colors, led_phys_color, LED_CH_COUNT);
}

void reset_led_binding(void)
{
	static const uint8_t identity[LED_CH_COUNT] = {LED_CH_R, LED_CH_G, LED_CH_B};

	(void)set_led_binding(identity);
}

bool led_uses_color_channels(void)
{
	return true;
}

#else /* !LED_TRI_RENDER */

/* Only the worker owns driver calls, including power and strip transfers. */
void set_led(enum sys_led_pattern pattern, int priority)
{
#if LED_EXISTS || LED_STRIP_EXISTS || PWM_LED_EXISTS
	if (priority < 0 || priority >= SYS_LED_PATTERN_DEPTH) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	if (led_patterns[priority] != pattern) {
		led_patterns[priority] = pattern;
		led_generations[priority]++;
	}
	k_spin_unlock(&led_request_lock, key);
	k_sem_give(&led_changed);
#else
	(void)pattern;
	(void)priority;
#endif
}

void led_shutdown(void)
{
#if LED_EXISTS || LED_STRIP_EXISTS || PWM_LED_EXISTS
	/* Shutdown callers may overlap; this operation cannot be superseded by
	 * an ordinary request while the driver finishes its current transfer. */
	k_mutex_lock(&led_shutdown_lock, K_FOREVER);
	k_sem_reset(&led_quiesced);
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	shutdown_pending = true;
	k_spin_unlock(&led_request_lock, key);
	k_sem_give(&led_changed);
	k_sem_take(&led_quiesced, K_FOREVER);
	k_mutex_unlock(&led_shutdown_lock);
#endif
}

#if LED_EXISTS || LED_STRIP_EXISTS || PWM_LED_EXISTS
struct led_request {
	enum sys_led_pattern pattern;
	int owner;
	uint32_t generation;
};

static struct led_request led_request_snapshot(void)
{
	struct led_request request = {.pattern = SYS_LED_PATTERN_OFF, .owner = -1};
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) {
		if (led_patterns[i] != SYS_LED_PATTERN_OFF) {
			request.pattern = led_patterns[i];
			request.owner = i;
			request.generation = led_generations[i];
			break;
		}
	}
	k_spin_unlock(&led_request_lock, key);
	return request;
}

static bool led_complete(struct led_request request, enum sys_led_pattern result)
{
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	int winner = -1;
	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) {
		if (led_patterns[i] != SYS_LED_PATTERN_OFF) {
			winner = i;
			break;
		}
	}
	bool matches = request.owner >= 0 && winner == request.owner
		&& led_patterns[request.owner] == request.pattern
		&& led_generations[request.owner] == request.generation;
	if (matches) {
		led_patterns[request.owner] = result;
		led_generations[request.owner]++;
	}
	k_spin_unlock(&led_request_lock, key);
	k_sem_give(&led_changed);
	return matches;
}

static uint32_t led_local_ticks(void)
{
	/* nRF54 kernel ticks are 31250 Hz; LED phase is always 32768 Hz. */
	uint64_t ticks = k_uptime_ticks();
	return (uint32_t)((ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC) * LED_SYNC_HZ
		+ (ticks % CONFIG_SYS_CLOCK_TICKS_PER_SEC) * LED_SYNC_HZ
			/ CONFIG_SYS_CLOCK_TICKS_PER_SEC);
}
#endif

static void led_thread(void)
{
#if !LED_EXISTS && !LED_STRIP_EXISTS && !PWM_LED_EXISTS
	LOG_WRN("LED GPIO does not exist");
	return;
#else
	enum sys_led_pattern current = SYS_LED_PATTERN_OFF;
	int owner = -1;
	uint32_t generation = 0;
	int state = 0;
	bool powered = false;
	int last_value = -1;
	int last_brightness = -1;
	enum sys_led_color last_color = SYS_LED_COLOR_DEFAULT;
	int64_t due = 0;
	unsigned steady_attempts = 0;
	uint32_t local_origin = 0;
	struct led_sync_start active = {0};
#if CONFIG_LED_NETWORK_SYNC
	uint32_t held_offset = 0;
	bool have_offset = false;
#endif
	for (;;) {
		k_spinlock_key_t key = k_spin_lock(&led_request_lock);
		bool shutting_down = shutdown_pending;
		k_spin_unlock(&led_request_lock, key);
		if (shutting_down) {
			if (powered) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
#if LED_STRIP_EXISTS
				/* Electrical margin, not a driver completion acknowledgment. */
				k_msleep(1);
#endif
			}
			led_suspend();
			powered = false;
			last_value = -1;
			key = k_spin_lock(&led_request_lock);
			led_patterns[SYS_LED_PRIORITY_HIGHEST] = SYS_LED_PATTERN_OFF_FORCE;
			led_generations[SYS_LED_PRIORITY_HIGHEST]++;
			shutdown_pending = false;
			k_spin_unlock(&led_request_lock, key);
			k_sem_give(&led_quiesced);
		}
		struct led_request request = led_request_snapshot();
		int64_t now = k_uptime_ticks();
		if (request.pattern != current ||
		    (request.owner == owner && request.generation != generation)) {
			current = request.pattern;
			state = 0;
			due = now;
			steady_attempts = 0;
			local_origin = led_local_ticks();
			active = (struct led_sync_start){.entered = local_origin};
			/* A new effect starts with a fresh first frame, not the previous
			 * effect's fractional strip brightness. Keep hardware powered. */
#if LED_STRIP_EXISTS
			led_strip_fade_reset(&led_fade);
#endif
			last_value = -1;
		}
		owner = request.owner;
		generation = request.generation;
		if (current <= SYS_LED_PATTERN_OFF) {
			if (powered) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
#if LED_STRIP_EXISTS
				k_msleep(1);
#endif
				led_suspend();
				powered = false;
			}
			k_sem_take(&led_changed, K_FOREVER);
			continue;
		}
		if (!powered) {
			led_resume();
			powered = true;
			last_value = -1;
			now = k_uptime_ticks();
		}
		if (now < due) {
			k_sem_take(&led_changed, due == INT64_MAX ? K_FOREVER : K_TICKS(due - now));
			continue;
		}
		uint32_t wait_us = 0;
		bool forever = false;
		int brightness = 10000;
		int value = 0;
		enum sys_led_color color = SYS_LED_COLOR_DEFAULT;
		enum sys_led_pattern completed = SYS_LED_PATTERN_OFF;
		bool complete = false;
		switch (current) {
		case SYS_LED_PATTERN_ON:
		case SYS_LED_PATTERN_ON_PERSIST:
			color = current == SYS_LED_PATTERN_ON ? SYS_LED_COLOR_DEFAULT : SYS_LED_COLOR_SUCCESS;
			brightness = current == SYS_LED_PATTERN_ON ? 10000 : 2000;
			value = 10000;
			forever = true;
			break;
		case SYS_LED_PATTERN_SHORT:
		case SYS_LED_PATTERN_LONG:
		case SYS_LED_PATTERN_FLASH:
		case SYS_LED_PATTERN_DFU:
			state = (state + 1) % 2;
			value = state * 10000;
			color = current == SYS_LED_PATTERN_SHORT ? SYS_LED_COLOR_PAIRING :
				current == SYS_LED_PATTERN_DFU ? SYS_LED_COLOR_CHARGING : SYS_LED_COLOR_DEFAULT;
			wait_us = current == SYS_LED_PATTERN_SHORT ? (state ? 100000 : 900000) :
				current == SYS_LED_PATTERN_LONG ? 500000 :
				current == SYS_LED_PATTERN_FLASH ? 200000 : 100000;
			break;
		case SYS_LED_PATTERN_ONESHOT_POWERON:
		case SYS_LED_PATTERN_ONESHOT_PROGRESS:
		case SYS_LED_PATTERN_ONESHOT_COMPLETE:
		case SYS_LED_PATTERN_ONESHOT_PING:
			state++;
			color = current == SYS_LED_PATTERN_ONESHOT_PROGRESS ||
				current == SYS_LED_PATTERN_ONESHOT_COMPLETE ? SYS_LED_COLOR_SUCCESS : SYS_LED_COLOR_DEFAULT;
			value = (current == SYS_LED_PATTERN_ONESHOT_PING ? state % 2 : !(state % 2)) * 10000;
			complete = state == (current == SYS_LED_PATTERN_ONESHOT_POWERON ? 7 :
				current == SYS_LED_PATTERN_ONESHOT_PROGRESS ? 5 :
				current == SYS_LED_PATTERN_ONESHOT_COMPLETE ? 9 : 20);
			wait_us = 200000;
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			state++;
			brightness = state == 1 ? 10000 : (202 - state) * 50;
			value = state == 1 || state == 202 ? 0 : 10000;
			wait_us = state == 1 ? 250000 : 5000;
			complete = state == 202;
			completed = SYS_LED_PATTERN_OFF_FORCE;
			break;
		case SYS_LED_PATTERN_LONG_PERSIST:
		case SYS_LED_PATTERN_PULSE_PERSIST:
		case SYS_LED_PATTERN_ACTIVE_PERSIST: {
			uint32_t local = led_local_ticks();
			uint32_t clock = local - local_origin;
#if CONFIG_LED_NETWORK_SYNC
			uint32_t network;
			if (esb_get_status_clock(&local, &network)) {
				if (!have_offset && !active.started) {
					active.joined = false;
				}
				held_offset = network - local;
				have_offset = true;
			}
			clock = have_offset ? local + held_offset : local - local_origin;
#endif
			uint32_t distance;
			if (current == SYS_LED_PATTERN_PULSE_PERSIST) {
				color = SYS_LED_COLOR_CHARGING;
				value = led_sync_pulse(led_sync_phase(clock, 5U * LED_SYNC_HZ));
				distance = 164U;
			} else if (current == SYS_LED_PATTERN_LONG_PERSIST) {
				color = SYS_LED_COLOR_CHARGING;
				brightness = 2000;
				uint32_t phase = led_sync_phase(clock, LED_SYNC_HZ);
				value = phase < LED_SYNC_HZ / 2U ? 10000 : 0;
				distance = LED_SYNC_HZ / 2U - phase % (LED_SYNC_HZ / 2U);
			} else {
				uint32_t phase = led_sync_phase(clock, LED_SYNC_ACTIVE_PERIOD);
				value = led_sync_active(&active, local, phase) ? 10000 : 0;
				distance = phase < LED_SYNC_ACTIVE_OFF ?
					LED_SYNC_ACTIVE_OFF - phase : LED_SYNC_ACTIVE_PERIOD - phase;
				if (!active.eligible) {
					uint32_t hold = LED_SYNC_ACTIVE_OFF - (local - active.entered);
					if (hold < distance) {
						distance = hold;
					}
				}
			}
			distance = led_sync_until(clock, distance);
#if CONFIG_LED_NETWORK_SYNC
			if (distance > LED_SYNC_HZ / 4U) {
				distance = LED_SYNC_HZ / 4U;
			}
#endif
			wait_us = (uint32_t)(((uint64_t)distance * 1000000U + LED_SYNC_HZ - 1U) / LED_SYNC_HZ);
			break;
		}
		case SYS_LED_PATTERN_ERROR_A:
		case SYS_LED_PATTERN_ERROR_B:
		case SYS_LED_PATTERN_ERROR_C:
		case SYS_LED_PATTERN_ERROR_D:
			color = SYS_LED_COLOR_ERROR;
			state = (state + 1) % (current == SYS_LED_PATTERN_ERROR_D ? 2 : 10);
			value = (state % 2 && (current == SYS_LED_PATTERN_ERROR_D ||
				state < 4 + 2 * ((int)current - SYS_LED_PATTERN_ERROR_A))) * 10000;
			wait_us = 500000;
			break;
		default:
			forever = true;
			break;
		}
		/* Fade frames retain sub-level dither; stable outputs need no
		 * transfer merely to check clock freshness. */
		bool rendered = true;
		if (value != last_value || brightness != last_brightness || color != last_color
		    || current == SYS_LED_PATTERN_PULSE_PERSIST
		    || current == SYS_LED_PATTERN_ONESHOT_POWEROFF) {
			rendered = led_pin_set(color, brightness, value);
			if (rendered) {
				last_value = value;
				last_brightness = brightness;
				last_color = color;
			} else {
				last_value = -1;
			}
		}
		if (complete) {
			if (!led_complete(request, completed)) {
				/* A new winning owner may inherit this final frame. A
				 * replacement in the same slot instead restarts above. */
				state--;
			} else {
				/* A lower-priority one-shot may now become visible. */
				state = 0;
			}
			due = k_uptime_ticks();
		} else if (forever) {
			/* Only indefinitely held frames retry. Timed effects keep their
			 * normal deadlines, including 5ms fades. A same-pattern wake
			 * cannot refill this budget or revive an exhausted output. */
			if (!rendered && ++steady_attempts < 3) {
				due = k_uptime_ticks() + k_us_to_ticks_ceil64(100000);
			} else {
				due = INT64_MAX;
				k_sem_take(&led_changed, K_FOREVER);
			}
		} else {
			due = now + k_us_to_ticks_ceil64(wait_us);
		}
	}
#endif
}

/* Display mode, global brightness and LED binding are implemented on
 * three-channel boards only. Other boards keep the compile-time color mapping
 * and the compile-time brightness, so these store the value without changing
 * rendering; the console commands stay usable everywhere. */

static enum led_display_mode led_mode_legacy = LED_MODE_DAILY;
/* Reports the compile-time brightness of this board; rendering still uses
 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT directly. */
static uint16_t led_brightness_pptt_legacy = CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT;

void set_led_mode(enum led_display_mode mode)
{
	led_mode_legacy = (mode == LED_MODE_DEBUG) ? LED_MODE_DEBUG : LED_MODE_DAILY;
}

enum led_display_mode get_led_mode(void)
{
	return led_mode_legacy;
}

void set_led_brightness(uint8_t percent)
{
	if (percent > 100) {
		percent = 100;
	}
	led_brightness_pptt_legacy = (uint16_t)(percent * 100);
}

uint8_t get_led_brightness(void)
{
	return (uint8_t)(led_brightness_pptt_legacy / 100);
}

bool set_led_binding(const uint8_t phys_colors[3])
{
	(void)phys_colors;
	return false; /* no runtime binding on single-arbitration boards */
}

void get_led_binding(uint8_t phys_colors[3])
{
	static const uint8_t identity[3] = {0, 1, 2}; /* R G B */

	memcpy(phys_colors, identity, 3);
}

void reset_led_binding(void)
{
}

bool led_uses_color_channels(void)
{
	return false;
}

#endif /* LED_TRI_RENDER */
