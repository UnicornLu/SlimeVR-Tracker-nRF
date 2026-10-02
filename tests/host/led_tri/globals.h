#ifndef LED_TRI_HOST_GLOBALS_H
#define LED_TRI_HOST_GLOBALS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <limits.h>

#define CONFIG_LED_THREAD_STACK_SIZE 512
#ifndef CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 32768
#endif
#ifndef CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
#define CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT 10000
#endif
#ifndef CONFIG_LED_NETWORK_SYNC
#define CONFIG_LED_NETWORK_SYNC 0
#endif
#define CONFIG_APPLICATION_INIT_PRIORITY 0
#define CONFIG_LED_STRIP_TIMING_LOG 0
#define CONFIG_LED_STRIP 0
#define CONFIG_LED_TRI_COLOR 1
#define LED_THREAD_PRIORITY 0

#define LOG_MODULE_REGISTER(...)
#define LOG_DBG(...)
#define LOG_WRN(...)
#define LOG_INF(...)
#define LOG_ERR(...)
#define LOG_LEVEL_INF 0

#ifndef ARG_UNUSED
#define ARG_UNUSED(x) (void)(x)
#endif

#define DT_PATH(x) x
#define DT_ALIAS(x) x
#define HOST_CAT_INNER(a, b) a##b
#define HOST_CAT(a, b) HOST_CAT_INNER(a, b)
/* No led-enable gate and no led-gpios property: LED_EXISTS comes from led0. */
#define DT_NODE_HAS_PROP(n, p) 0
#define DT_NODE_EXISTS(n) HOST_CAT(DT_EXISTS_, n)
#define DT_EXISTS_led0 1
#define DT_EXISTS_led1 0
#define DT_EXISTS_led2 0
#define DT_EXISTS_led3 0
#define DT_EXISTS_pwm_led0 1
#define DT_EXISTS_pwm_led1 1
#define DT_EXISTS_pwm_led2 1
#define GPIO_DT_SPEC_GET(n, p) {.pin = 1}
#define PWM_DT_SPEC_GET(n) {.dev = &mock_pwm_dev, .period = 10000}
#define DEVICE_DT_GET(n) (&mock_pwm_dev)
#define SYS_INIT(...)
#define APPLICATION 0
#define GPIO_OUTPUT 1
#define GPIO_DISCONNECTED 0
#define GPIO_ACTIVE_HIGH 1
#define PM_DEVICE_ACTION_SUSPEND 0
#define PM_DEVICE_ACTION_RESUME 1

struct device {
	int unused;
};
extern struct device mock_pwm_dev;
static inline bool device_is_ready(const struct device *dev)
{
	(void)dev;
	return true;
}

struct gpio_dt_spec {
	int pin;
};
struct pwm_dt_spec {
	const struct device *dev;
	uint32_t period;
};
struct k_thread {
	int unused;
};
typedef struct k_thread *k_tid_t;

struct k_spinlock {
	int unused;
};
typedef int k_spinlock_key_t;
typedef int64_t k_timeout_t;
struct k_sem {
	unsigned count;
	unsigned limit;
};
struct k_mutex {
	int unused;
};

#define K_SEM_DEFINE(n, initial, max) struct k_sem n = {initial, max}
#define K_MUTEX_DEFINE(n) struct k_mutex n
#define K_THREAD_DEFINE(name, ...) static const k_tid_t name = &led_thread_mock
extern struct k_thread led_thread_mock;

#define K_FOREVER INT64_MAX
#define K_TICKS(n) (n)
#define K_MSEC(n) ((k_timeout_t)(n)) /* fake clock is milliseconds */

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *s)
{
	(void)s;
	return 0;
}
static inline void k_spin_unlock(struct k_spinlock *s, k_spinlock_key_t k)
{
	(void)s;
	(void)k;
}
static inline void k_sem_reset(struct k_sem *s)
{
	s->count = 0;
}
static inline void k_mutex_lock(struct k_mutex *m, k_timeout_t timeout)
{
	(void)m;
	(void)timeout;
}
static inline void k_mutex_unlock(struct k_mutex *m)
{
	(void)m;
}
static inline uint64_t k_us_to_ticks_ceil64(uint64_t us)
{
	return (us * CONFIG_SYS_CLOCK_TICKS_PER_SEC + 999999) / 1000000;
}

/* Provided by the test: fake clock, recording drivers, mock retained data. */
int k_sem_take(struct k_sem *sem, k_timeout_t timeout);
void k_sem_give(struct k_sem *sem);
int32_t k_msleep(int32_t duration);
k_tid_t k_current_get(void);
uint32_t k_uptime_get_32(void);
int64_t k_uptime_ticks(void);
int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, int flags);
int gpio_pin_set_dt(const struct gpio_dt_spec *spec, int value);
int pwm_set_pulse_dt(const struct pwm_dt_spec *spec, uint32_t pulse);
int pm_device_action_run(const struct device *dev, int action);

/* Mock of the retained LED preference fields (see src/retained.h). */
struct retained_data {
	uint8_t led_mode;
	uint8_t led_bright;
	uint8_t led_bind[3];
};
extern struct retained_data *retained;
void retained_update(void);

#endif /* LED_TRI_HOST_GLOBALS_H */
