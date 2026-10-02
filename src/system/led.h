#ifndef SLIMENRF_SYSTEM_LED
#define SLIMENRF_SYSTEM_LED

#include <stdbool.h>
#include <stdint.h>

/*
LED priorities (0 is highest)
0: boot/power
1: sensor
2: connection (esb)
3: status
4: system (persist)
*/

#define SYS_LED_PRIORITY_HIGHEST 0
#define SYS_LED_PRIORITY_BOOT 0
#define SYS_LED_PRIORITY_SENSOR 1
#define SYS_LED_PRIORITY_CONNECTION 2
#define SYS_LED_PRIORITY_STATUS 3
#define SYS_LED_PRIORITY_SYSTEM 4
#define SYS_LED_PATTERN_DEPTH 5

// RGB
// Red, Green, Blue

// Tri-color
// Red/Amber, Green, YellowGreen/White

// RG
// Red, Green

// Dual color
// Red/Amber, YellowGreen/White

// TODO: these patterns are kinda funky
enum sys_led_pattern {
	SYS_LED_PATTERN_OFF_FORCE, // ignores lower priority patterns

	SYS_LED_PATTERN_OFF,   // yield to lower priority patterns
	SYS_LED_PATTERN_ON,    // Default | indicates busy
	SYS_LED_PATTERN_SHORT, // 100ms on 900ms off									// Pairing | indicates waiting
						   // (pairing)
	SYS_LED_PATTERN_LONG,  // 500ms on 500ms off										// Default | indicates waiting
	SYS_LED_PATTERN_FLASH, // 200ms on 200ms off									// Default | indicates readiness

	SYS_LED_PATTERN_ONESHOT_POWERON,  // 200ms on 200ms off, 3 times					// Default
	SYS_LED_PATTERN_ONESHOT_POWEROFF, // 250ms off, 1000ms fade to off				// Default
	SYS_LED_PATTERN_ONESHOT_PROGRESS, // 200ms on 200ms off, 2 times				// Success
	SYS_LED_PATTERN_ONESHOT_COMPLETE, // 200ms on 200ms off, 4 times				// Success
	SYS_LED_PATTERN_ONESHOT_PING,     // 200ms on 200ms off, 10 times				// Ping

	SYS_LED_PATTERN_ON_PERSIST,     // 20% duty cycle									// Success | indicates charged
	SYS_LED_PATTERN_LONG_PERSIST,   // 20% duty cycle, 500ms on 500ms off				// Charging| indicates low battery
	SYS_LED_PATTERN_PULSE_PERSIST,  // 5000ms pulsing								// Charging| indicates charging
	SYS_LED_PATTERN_ACTIVE_PERSIST, // 300ms on 9700ms off							// Default | indicates normal
									// operation

	SYS_LED_PATTERN_ERROR_A, // 500ms on 500ms off, 2 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_B, // 500ms on 500ms off, 3 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_C, // 500ms on 500ms off, 4 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_D, // 500ms on 500ms off (same as SYS_LED_PATTERN_LONG)	// Error
	SYS_LED_PATTERN_DFU,       // Fast yellow pulse								// DFU/OTA update mode

	/* Three-channel boards only (see CONFIG_LED_TRI_COLOR): each value below is
	 * rendered by per-channel semantics rather than a single color+value pair.
	 * Appending keeps every existing numeric value stable. */
	SYS_LED_PATTERN_CONNECT_HEARTBEAT, // Blue link heartbeat, set again after pairing
	SYS_LED_PATTERN_CAL_PROGRESS,      // Red->green by led_cal_progress (magnetometer calibration)
	SYS_LED_PATTERN_FAST_GREEN,        // Long-press zone preview: green only (unused so far)
	SYS_LED_PATTERN_FAST_BLUE,         // Long-press zone preview: blue only (unused so far)
	SYS_LED_PATTERN_FAST_RED,          // Long-press zone preview: red only (unused so far)
	SYS_LED_PATTERN_ONESHOT_X2,        // Double-flash confirmation (unused so far)
	SYS_LED_PATTERN_ONESHOT_X3,        // Triple-flash confirmation (unused so far)

	SYS_LED_PATTERN__COUNT,
};

enum sys_led_color {
	SYS_LED_COLOR_DEFAULT,
	SYS_LED_COLOR_SUCCESS,
	SYS_LED_COLOR_ERROR,
	SYS_LED_COLOR_CHARGING,
	SYS_LED_COLOR_PAIRING,
};

/* Assignment table for three-channel boards: daily (breathing family) or debug
 * (flashing family). Selected at runtime with the ledmode console command. */
enum led_display_mode {
	LED_MODE_DAILY = 0,
	LED_MODE_DEBUG = 1,
};

void set_led(enum sys_led_pattern led_pattern, int priority);

/* Mode and global brightness (three-channel boards; persisted in retained).
 * On other boards these only store the value so the console stays usable. */
void set_led_mode(enum led_display_mode mode);
enum led_display_mode get_led_mode(void);
void set_led_brightness(uint8_t percent); // 0-100, global multiplier (0 = all off)
uint8_t get_led_brightness(void);

/* LED binding (three-channel boards; persisted in retained): which semantic
 * color sits on each physical position LED1/2/3 (= devicetree pwm-led0/1/2).
 * color values: 0 = R, 1 = G, 2 = B. The setter requires a permutation of
 * R/G/B and rejects duplicates or out-of-range values. */
bool set_led_binding(const uint8_t phys_colors[3]);
void get_led_binding(uint8_t phys_colors[3]);
void reset_led_binding(void);

/* Magnetometer calibration progress (0-10000; written by the calibration
 * thread, read by the LED thread). Defined for every build so cal_mag.c links. */
extern volatile uint16_t led_cal_progress;

/* True when the three-channel renderer is active (CONFIG_LED_TRI_COLOR plus three
 * PWM LED aliases). Callers that set channel-specific patterns use this to keep
 * single-arbitration boards on their previous indication: those patterns have no
 * color-table equivalent there and would otherwise mask the ordinary status. */
bool led_uses_color_channels(void);

/* Blocking black/power-gate barrier for shutdown threads only (not ISR or
 * LED-worker context). Concurrent callers are serialized. */
void led_shutdown(void);

#endif
