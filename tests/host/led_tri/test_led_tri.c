/*
 * Host test for the three-channel LED renderer (CONFIG_LED_TRI_COLOR boards).
 *
 * The production source is compiled in directly, with the devicetree, kernel and
 * driver interfaces replaced by the shims in this directory. The worker runs on a
 * fake millisecond clock: every blocking call advances it and longjmps out of the
 * (endless) worker loop once the run budget is spent, so each assertion sees the
 * real rendering path end to end. Events can be scheduled on the fake clock to
 * drive the worker from inside its own loop (which is how the shutdown barrier is
 * exercised without a second thread).
 */

#include <assert.h>
#include <errno.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../../src/system/led.c"

/* ---- fake platform ------------------------------------------------------- */

struct device mock_pwm_dev;
struct k_thread led_thread_mock;

static int64_t now_ms;
static int64_t stop_at_ms;
static jmp_buf stopped;
static bool in_led_thread;
static uint32_t quiesced_gives;
static unsigned suspend_count;
static int64_t ms_since_motion;
static bool ota_active;
static bool clock_synced;
static int64_t network_offset_ms;

struct write {
	int64_t at;
	uint8_t phys;
	uint32_t pulse;
};
static struct write writes[8192];
static unsigned write_count;

struct event {
	int64_t at;
	void (*fn)(void);
};
static struct event events[8];
static unsigned event_count;

static struct retained_data mock_retained;
struct retained_data *retained = &mock_retained;

void retained_update(void)
{
}

int64_t k_uptime_ticks(void)
{
	return (now_ms * CONFIG_SYS_CLOCK_TICKS_PER_SEC) / 1000;
}

uint32_t k_uptime_get_32(void)
{
	return (uint32_t)now_ms;
}

k_tid_t k_current_get(void)
{
	return in_led_thread ? led_thread_id : NULL;
}

void k_sem_give(struct k_sem *sem)
{
	if (sem == &led_quiesced) {
		quiesced_gives++;
	}
	if (sem->count < sem->limit) {
		sem->count++;
	}
}

static void record_write(uint8_t phys, uint32_t pulse)
{
	assert(write_count < sizeof(writes) / sizeof(writes[0]));
	writes[write_count++] = (struct write){now_ms, phys, pulse};
}

static void poll_events(void)
{
	if (event_count == 0 || now_ms < events[0].at) {
		return;
	}
	void (*fn)(void) = events[0].fn;
	memmove(&events[0], &events[1], (event_count - 1) * sizeof(events[0]));
	event_count--;
	fn();
	fn = NULL;
}

int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	if (sem == &led_quiesced) {
		/* led_shutdown()'s barrier: the worker's give is asserted instead of
		 * blocking, because the harness runs single-threaded. */
		return 0;
	}
	poll_events();
	if (timeout == K_FOREVER) {
		longjmp(stopped, 1);
	}
	now_ms += (int64_t)timeout;
	if (now_ms >= stop_at_ms) {
		longjmp(stopped, 1);
	}
	return -EAGAIN;
}

int32_t k_msleep(int32_t duration)
{
	now_ms += duration;
	poll_events();
	if (now_ms >= stop_at_ms) {
		longjmp(stopped, 1);
	}
	return 0;
}

int64_t sensor_ms_since_motion(void)
{
	return ms_since_motion;
}

bool esb_ota_is_active(void)
{
	return ota_active;
}

bool esb_get_status_clock(uint32_t *local_ticks, uint32_t *network_ticks)
{
	if (!clock_synced) {
		return false;
	}
	*local_ticks = (uint32_t)((now_ms * (int64_t)LED_SYNC_HZ) / 1000);
	*network_ticks = (uint32_t)(((now_ms + network_offset_ms) * (int64_t)LED_SYNC_HZ) / 1000);
	return true;
}

int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, int flags)
{
	(void)spec;
	(void)flags;
	return 0;
}

int gpio_pin_set_dt(const struct gpio_dt_spec *spec, int value)
{
	(void)spec;
	(void)value;
	return 0;
}

int pm_device_action_run(const struct device *dev, int action)
{
	/* Suspending the PWM device stops its output: model that as a black frame so
	 * "what the LEDs show" stays truthful after the worker parks. */
	if (action == PM_DEVICE_ACTION_SUSPEND && dev == &mock_pwm_dev) {
		suspend_count++;
		for (uint8_t phys = 0; phys < LED_CH_COUNT; phys++) {
			record_write(phys, 0);
		}
	}
	return 0;
}

int pwm_set_pulse_dt(const struct pwm_dt_spec *spec, uint32_t pulse)
{
	uint8_t phys = 255;

	if (spec == &pwm_led) {
		phys = 0;
	} else if (spec == &pwm_led1) {
		phys = 1;
	} else if (spec == &pwm_led2) {
		phys = 2;
	}
	record_write(phys, pulse);
	return 0;
}

/* ---- harness helpers ----------------------------------------------------- */

static void reset_harness(void)
{
	write_count = 0;
	event_count = 0;
	now_ms = 0;
	quiesced_gives = 0;
	suspend_count = 0;
	ms_since_motion = 0;
	ota_active = false;
	clock_synced = false;
	network_offset_ms = 0;
	shutdown_pending = false;

	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) {
		led_patterns[i] = SYS_LED_PATTERN_OFF;
		led_generations[i] = 0;
	}
	for (int i = 0; i < LED_CH_COUNT; i++) {
		chans[i].pattern = SYS_LED_PATTERN_OFF;
		chans[i].owner = -1;
		chans[i].generation = 0;
		chans[i].state = 0;
		chans[i].last_value = 0;
	}
	led_mode = LED_MODE_DAILY;
	led_brightness_pptt = 2500;
	led_phys_color[0] = LED_CH_R;
	led_phys_color[1] = LED_CH_G;
	led_phys_color[2] = LED_CH_B;
	led_cal_progress = 0;

	mock_retained.led_mode = 0;
	mock_retained.led_bright = 25;
	mock_retained.led_bind[0] = 0;
	mock_retained.led_bind[1] = 1;
	mock_retained.led_bind[2] = 2;
}

static void schedule(int64_t at, void (*fn)(void))
{
	assert(event_count < sizeof(events) / sizeof(events[0]));
	events[event_count++] = (struct event){at, fn};
}

/* Run the worker until the fake clock reaches "until_ms". */
static void run_frames(int64_t until_ms)
{
	stop_at_ms = until_ms;
	in_led_thread = true;
	if (setjmp(stopped) == 0) {
		led_thread();
	}
	in_led_thread = false;
}

static int value_at(uint8_t phys, int64_t at_ms)
{
	int found = -1;

	for (unsigned i = 0; i < write_count; i++) {
		if (writes[i].phys == phys && writes[i].at <= at_ms) {
			found = (int)writes[i].pulse;
		}
	}
	return found;
}

static int max_value(uint8_t phys, int64_t from_ms, int64_t to_ms)
{
	int best = -1;

	for (unsigned i = 0; i < write_count; i++) {
		if (writes[i].phys == phys && writes[i].at >= from_ms && writes[i].at <= to_ms) {
			if ((int)writes[i].pulse > best) {
				best = (int)writes[i].pulse;
			}
		}
	}
	return best;
}

/* ---- tests --------------------------------------------------------------- */

/* Physical PWM positions (pwm-led0/1/2). With the default identity binding they
 * carry red, green and blue respectively. */
#define POS0 0
#define POS1 1
#define POS2 2

#define PEAK_FULL 10000
#define BR_DEFAULT 25 /* percent; the board default brightness */

static void test_concurrent_channels_on_the_grid(void)
{
	reset_harness();
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	set_led(SYS_LED_PATTERN_CONNECT_HEARTBEAT, SYS_LED_PRIORITY_CONNECTION);
	run_frames(22000);

	/* 25% brightness: the green peak (10000) becomes 2500 at t = 1.2 s. */
	assert(value_at(POS1, 1200) == PEAK_FULL * BR_DEFAULT / 100);
	assert(value_at(POS1, 10000) == 0);
	/* Blue heartbeat is inverted by half the 20 s grid (peak at 11.2 s, 50%). */
	assert(value_at(POS2, 11200) == 5000 * BR_DEFAULT / 100);
	assert(value_at(POS2, 1200) == 0);
	/* Green peaks again in the next period: the grid does not drift. */
	assert(value_at(POS1, 21200) == PEAK_FULL * BR_DEFAULT / 100);
	/* The red channel is rendered (dark here) rather than left undriven. */
	assert(value_at(POS0, 5000) == 0);
}

static void test_unpaired_double_heartbeat(void)
{
	reset_harness();
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	run_frames(20000);

	/* Two swells at 10 s and 13 s, dark between them. */
	assert(max_value(POS2, 11200 - 60, 11200 + 60) == 5000 * BR_DEFAULT / 100);
	assert(max_value(POS2, 14200 - 60, 14200 + 60) == 5000 * BR_DEFAULT / 100);
	assert(max_value(POS2, 12600, 12800) == 0);
	assert(max_value(POS2, 2000, 9000) == 0);
	assert(max_value(POS1, 0, 20000) == 0);
}

static void test_debug_table_flashes(void)
{
	reset_harness();
	set_led_mode(LED_MODE_DEBUG);
	assert(get_led_mode() == LED_MODE_DEBUG);
	assert(mock_retained.led_mode == (uint8_t)LED_MODE_DEBUG);

	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	run_frames(2000);

	/* Debug family: a 300 ms blip every 10 s instead of the 20 s breathe. */
	assert(value_at(POS1, 200) == PEAK_FULL * BR_DEFAULT / 100);
	assert(value_at(POS1, 500) == 0);
	assert(value_at(POS1, 10200) == PEAK_FULL * BR_DEFAULT / 100);

	set_led_mode(LED_MODE_DAILY);
	assert(get_led_mode() == LED_MODE_DAILY);
	assert(mock_retained.led_mode == (uint8_t)LED_MODE_DAILY);
}

static void test_brightness_scales_everything(void)
{
	reset_harness();
	set_led_brightness(60);
	assert(get_led_brightness() == 60);
	assert(mock_retained.led_bright == 60);

	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	run_frames(1300);
	assert(value_at(POS1, 1200) == 6000);

	set_led_brightness(0);
	assert(get_led_brightness() == 0);
	run_frames(2700);
	assert(max_value(POS1, 1300, 2700) == 0);
	assert(max_value(POS2, 0, 2700) == 0);
}

static void test_binding_must_be_a_permutation(void)
{
	reset_harness();

	const uint8_t duplicate[3] = {0, 0, 2};
	const uint8_t out_of_range[3] = {0, 1, 3};
	assert(!set_led_binding(duplicate));
	assert(!set_led_binding(out_of_range));

	uint8_t cur[3];
	get_led_binding(cur);
	assert(cur[0] == LED_CH_R && cur[1] == LED_CH_G && cur[2] == LED_CH_B);

	const uint8_t swapped[3] = {LED_CH_B, LED_CH_G, LED_CH_R};
	assert(set_led_binding(swapped));
	get_led_binding(cur);
	assert(cur[0] == LED_CH_B && cur[1] == LED_CH_G && cur[2] == LED_CH_R);
	assert(mock_retained.led_bind[0] == LED_CH_B);

	/* Semantic red now drives physical position 2 (pwm-led2). */
	set_led(SYS_LED_PATTERN_PULSE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	run_frames(1000);
	assert(max_value(POS0, 0, 1000) == 0);
	assert(max_value(POS2, 0, 1000) > 0);

	reset_harness();
	reset_led_binding();
	get_led_binding(cur);
	assert(cur[0] == LED_CH_R && cur[1] == LED_CH_G && cur[2] == LED_CH_B);
}

static void test_off_force_clears_every_slot(void)
{
	reset_harness();
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	set_led(SYS_LED_PATTERN_CONNECT_HEARTBEAT, SYS_LED_PRIORITY_CONNECTION);

	set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_SYSTEM);
	assert(led_patterns[SYS_LED_PRIORITY_HIGHEST] == SYS_LED_PATTERN_OFF_FORCE);
	for (int i = 1; i < SYS_LED_PATTERN_DEPTH; i++) {
		assert(led_patterns[i] == SYS_LED_PATTERN_OFF);
	}

	run_frames(1000);
	/* Nothing is ever lit while OFF_FORCE owns the highest slot. */
	assert(max_value(POS0, 0, 1000) <= 0);
	assert(max_value(POS1, 0, 1000) <= 0);
	assert(max_value(POS2, 0, 1000) <= 0);
}

static void request_shutdown(void)
{
	led_shutdown();
}

static void test_shutdown_barrier_blacks_out_first(void)
{
	reset_harness();
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	schedule(1000, request_shutdown);
	run_frames(2000);

	/* The worker acknowledged the barrier and left all three channels black. */
	assert(quiesced_gives == 1);
	assert(suspend_count >= 1);
	assert(value_at(POS0, 2000) == 0);
	assert(value_at(POS1, 2000) == 0);
	assert(value_at(POS2, 2000) == 0);
	/* The lights were on before the request (the dark state is not a no-op). */
	assert(max_value(POS1, 0, 990) > 0);
}

static void test_one_shot_retires_its_own_slot(void)
{
	reset_harness();
	set_led(SYS_LED_PATTERN_ONESHOT_PING, SYS_LED_PRIORITY_HIGHEST);
	set_led(SYS_LED_PATTERN_CONNECT_HEARTBEAT, SYS_LED_PRIORITY_CONNECTION);
	run_frames(2000);

	assert(led_patterns[SYS_LED_PRIORITY_HIGHEST] == SYS_LED_PATTERN_OFF);
	assert(led_patterns[SYS_LED_PRIORITY_CONNECTION] == SYS_LED_PATTERN_CONNECT_HEARTBEAT);
}

static void test_resting_dim_and_ota_exemption(void)
{
	reset_harness();
	ms_since_motion = 180001;
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	run_frames(1300);
	assert(max_value(POS1, 1150, 1250) == PEAK_FULL * BR_DEFAULT / 100 / 2);

	reset_harness();
	ota_active = true;
	ms_since_motion = 180001;
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	run_frames(1300);
	assert(max_value(POS1, 1150, 1250) == PEAK_FULL * BR_DEFAULT / 100);
}

static void test_calibration_progress_bar(void)
{
	reset_harness();
	led_cal_progress = 10000; /* finished: green leg only */
	set_led(SYS_LED_PATTERN_CAL_PROGRESS, SYS_LED_PRIORITY_SENSOR);
	run_frames(1000);
	assert(max_value(POS1, 0, 1000) > 0);
	assert(max_value(POS0, 0, 1000) == 0);

	reset_harness();
	led_cal_progress = 0; /* start: red leg only */
	set_led(SYS_LED_PATTERN_CAL_PROGRESS, SYS_LED_PRIORITY_SENSOR);
	run_frames(1000);
	assert(max_value(POS0, 0, 1000) > 0);
	assert(max_value(POS1, 0, 1000) == 0);
}

static void test_error_mask_and_channel_yield(void)
{
	reset_harness();
	set_led(SYS_LED_PATTERN_CONNECT_HEARTBEAT, SYS_LED_PRIORITY_CONNECTION);
	set_led(SYS_LED_PATTERN_ERROR_B, SYS_LED_PRIORITY_STATUS);
	run_frames(2000);

	/* The heartbeat holds the blue channel (lower slot index) until status.c
	 * yields it on a connection error. */
	assert(led_patterns[SYS_LED_PRIORITY_CONNECTION] == SYS_LED_PATTERN_CONNECT_HEARTBEAT);

	reset_harness();
	set_led(SYS_LED_PATTERN_ERROR_B, SYS_LED_PRIORITY_STATUS);
	run_frames(1000);
	/* Errors own all three channels: only red breathes in the daily table. */
	assert(max_value(POS0, 0, 1000) > 0);
	assert(max_value(POS1, 0, 1000) == 0);
	assert(max_value(POS2, 0, 1000) == 0);

	/* With the link slot yielded, the red error shows on the blue channel too. */
	reset_harness();
	set_led(SYS_LED_PATTERN_ERROR_B, SYS_LED_PRIORITY_STATUS);
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CONNECTION);
	run_frames(1000);
	assert(max_value(POS2, 0, 1000) == 0);
}

static void test_first_sample_defaults(void)
{
	reset_harness();
	mock_retained.led_bright = 0xFF; /* not initialized */
	mock_retained.led_mode = 0xFF;
	mock_retained.led_bind[0] = 0xFF;
	mock_retained.led_bind[1] = 0xFF;
	mock_retained.led_bind[2] = 0xFF;
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	run_frames(1300);

	/* 0xFF is rejected everywhere: 25% brightness, daily table, identity binding. */
	assert(get_led_brightness() == 25);
	assert(get_led_mode() == LED_MODE_DAILY);
	assert(value_at(POS1, 1200) == 2500);
}

static void test_stored_preferences_are_restored(void)
{
	reset_harness();
	mock_retained.led_mode = (uint8_t)LED_MODE_DEBUG;
	mock_retained.led_bright = 50;
	mock_retained.led_bind[0] = LED_CH_B;
	mock_retained.led_bind[1] = LED_CH_G;
	mock_retained.led_bind[2] = LED_CH_R;
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	run_frames(1300);

	/* Debug table + 50% brightness. The green blip is routed by the binding:
	 * position 1 carries green, positions 0 (blue) and 2 (red) stay dark. */
	assert(value_at(POS1, 200) == 5000);
	assert(value_at(POS0, 200) == 0);
	assert(value_at(POS2, 200) == 0);

	uint8_t cur[3];
	get_led_binding(cur);
	assert(cur[0] == LED_CH_B && cur[2] == LED_CH_R);
}

#if CONFIG_LED_NETWORK_SYNC
static void test_grid_follows_receiver_time(void)
{
	/* Receiver time is 10 s ahead of the local clock: the green peak moves with
	 * the shared grid, not with local uptime. */
	reset_harness();
	clock_synced = true;
	network_offset_ms = 10000;
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	run_frames(12000);

	assert(max_value(POS1, 11200 - 200, 11200 + 200) == PEAK_FULL * BR_DEFAULT / 100);
	assert(max_value(POS1, 1200 - 200, 1200 + 200) == 0);

	/* Losing the receiver clock keeps extrapolating the last offset: the next
	 * peak stays on the same grid instead of jumping back to local time. */
	clock_synced = false;
	run_frames(32000);
	assert(max_value(POS1, 31200 - 400, 31200 + 400) == PEAK_FULL * BR_DEFAULT / 100);
}
#endif

int main(void)
{
	test_concurrent_channels_on_the_grid();
	test_unpaired_double_heartbeat();
	test_debug_table_flashes();
	test_brightness_scales_everything();
	test_binding_must_be_a_permutation();
	test_off_force_clears_every_slot();
	test_shutdown_barrier_blacks_out_first();
	test_one_shot_retires_its_own_slot();
	test_resting_dim_and_ota_exemption();
	test_calibration_progress_bar();
	test_error_mask_and_channel_yield();
	test_first_sample_defaults();
	test_stored_preferences_are_restored();
#if CONFIG_LED_NETWORK_SYNC
	test_grid_follows_receiver_time();
#endif
	printf("led_tri: all assertions passed\n");
	return 0;
}
