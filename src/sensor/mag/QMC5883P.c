#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "QMC5883P.h"
#include "sensor/sensor_none.h"

// QMC5883P 3-Axis Magnetic Sensor, pin-to-pin successor of QMC5883L with an
// incompatible register map (closer to QMC6309).
// I2C address 0x2C, Chip ID register 0x00 = 0x80

#define QMC5883P_CHIP_ID 0x00 // 0x80, checked by the scanner

#define QMC5883P_OUTX_L_REG 0x01

// Status register {DRDY:1;OVFL:1;...}; DRDY clears when this register is read
#define QMC5883P_STAT_REG 0x09

#define STAT_DATA_RDY_MASK 0b01
#define STAT_OVERFLOW_MASK 0b10

// Control register 1 {OSR2:2;OSR1:2;ODR:2;MD:2;}
#define QMC5883P_CTRL_REG_1 0x0A

#define MD_SUSPEND 0b00
#define MD_NORMAL 0b01
#define MD_SINGLE 0b10
#define MD_CONTINUOUS 0b11
#define MD_MASK 0b11

#define ODR_10Hz 0b00
#define ODR_50Hz 0b01
#define ODR_100Hz 0b10
#define ODR_200Hz 0b11
#define ODR_MASK(odr) ((odr) << 2)

// Over sample ratio 1: 00=8 (lowest noise), 11=1
#define OSR1_8 0b00
#define OSR1_4 0b01
#define OSR1_2 0b10
#define OSR1_1 0b11
#define OSR1_MASK(osr1) ((osr1) << 4)

// Down sampling rate: 00=1, 11=8; the vendor setup examples use OSR2=8
#define OSR2_8 0b11
#define OSR2_MASK(osr2) ((osr2) << 6)

// Control register 2 {SOFT_RST:1;SELF_TEST:1;:2;RNG:2;SET/RESET:2;}
#define QMC5883P_CTRL_REG_2 0x0B

#define SOFT_RESET_MASK 0x80
#define SOFT_RESET_CLEAR 0x00

#define SELF_TEST_MASK 0x40

// 00=±30G (1000 LSB/G), 01=±12G (2500), 10=±8G (3750), 11=±2G (15000)
#define RNG_30G 0b00
#define RNG_12G 0b01
#define RNG_8G 0b10
#define RNG_2G 0b11
#define RNG_MASK(rng) ((rng) << 2)

// 00 = set and reset on (offset renewed each measurement); 10/11 keep it off
#define SET_RESET_ON 0b00

// Undocumented axis sign register; every datasheet setup example (Rev A,
// sections 7.1-7.3) starts by writing 0x06 to define the X Y Z sign convention.
#define QMC5883P_SIGN_REG 0x29
#define QMC5883P_SIGN_XYZ 0x06

static const float sensitivity = 1 / 3750.0f; // 3750 LSB/G at 8G range

static uint8_t last_state = 0xff;
static bool last_overflow = false;
static int64_t oneshot_trigger_ms = 0;
static bool oneshot_pending;
static bool oneshot_failed;
// Normal-mode data registers retain the last sample (Rev A, section 9.2.1).
// Byte equality is a duplicate heuristic, not proof of sample identity.
// The external-interface path omits the separate status read.
static uint8_t last_raw_sample[6];
static bool last_raw_sample_valid = false;
static int64_t last_mag_time_ms;
static int32_t mag_period_ms = 20; // default 50Hz

LOG_MODULE_REGISTER(QMC5883P, LOG_LEVEL_INF);

int qmc5883p_init(float period_s, float *actual_period_s)
{
	last_state = 0xff; // init state
	last_overflow = false;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;
	last_mag_time_ms = 0;

	// Soft reset restores all register defaults and enters suspend mode.
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_2, SOFT_RESET_MASK);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	k_msleep(2); // max 250us POR time, plus margin for NVM reload

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_SIGN_REG, QMC5883P_SIGN_XYZ);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	err = qmc5883p_update_odr(period_s, actual_period_s);
	return (err < 0 ? err : 0);
}

void qmc5883p_shutdown(void)
{
	last_state = 0xff;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;
	// Soft reset powers down to suspend mode with default registers.
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_2, SOFT_RESET_MASK);
	if (err) {
		LOG_ERR("Communication error");
	}
}

int qmc5883p_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Truncate fractional Hz before selecting a supported rate.
	uint8_t odr_code;
	uint8_t mode_code;

	if (period_s <= 0 || period_s == INFINITY) // power down mode or single measurement mode
	{
		mode_code = MD_SUSPEND; // oneshot will set SINGLE after
		requested_odr_hz = 0;
	} else {
		mode_code = MD_NORMAL;
		requested_odr_hz = 1 / period_s;
	}

	if (mode_code == MD_SUSPEND) {
		odr_code = ODR_200Hz; // for oneshot
		period_s = INFINITY;  // signal oneshot mode available
	} else if (requested_odr_hz > 100) {
		odr_code = ODR_200Hz;
		period_s = 1.f / 200;
	} else if (requested_odr_hz > 50) {
		odr_code = ODR_100Hz;
		period_s = 1.f / 100;
	} else if (requested_odr_hz > 10) {
		odr_code = ODR_50Hz;
		period_s = 1.f / 50;
	} else {
		odr_code = ODR_10Hz;
		period_s = 1.f / 10;
	}

	// Mode changes between normal/single/continuous must route through
	// suspend (Rev A, section 9.2.3); suspend itself is always reachable.
	uint8_t config_code = OSR2_MASK(OSR2_8) | OSR1_MASK(OSR1_8) | ODR_MASK(odr_code) | mode_code;
	if (last_state == config_code) {
		*actual_period_s = period_s;
		return 0; /* already configured — success for err|= callers */
	}

	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		QMC5883P_CTRL_REG_2,
		RNG_MASK(RNG_8G) | SET_RESET_ON
	);
	if (!err) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_1, config_code);
	}
	if (err) {
		LOG_ERR("Communication error");
		last_state = 0xff;
		return err;
	}

	last_state = config_code;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;

	if (mode_code != MD_SUSPEND) {
		mag_period_ms = (int32_t)(period_s * 1000);
	}

	*actual_period_s = period_s;
	return 0;
}

void qmc5883p_mag_oneshot(void)
{
	// Single mode takes one measurement and returns to suspend by itself.
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		QMC5883P_CTRL_REG_1,
		OSR2_MASK(OSR2_8) | OSR1_MASK(OSR1_8) | ODR_MASK(ODR_200Hz) | MD_SINGLE
	);
	last_state = 0xff;
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_trigger_ms = k_uptime_get();
	if (err) {
		LOG_ERR("Communication error");
	}
}

bool qmc5883p_mag_read(float m[3])
{
	bool was_oneshot = oneshot_pending;
	if (oneshot_pending) {
		if (oneshot_failed) {
			oneshot_pending = false;
			oneshot_failed = false;
			return false;
		}

		// Oneshot mode: wait for DRDY with timeout
		uint8_t status = 0;
		int64_t deadline_ms = oneshot_trigger_ms + 10; // 10ms timeout
		while ((status & STAT_DATA_RDY_MASK) == 0) {
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_STAT_REG, &status);
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (k_uptime_get() >= deadline_ms) {
				LOG_WRN("Data ready status timeout!");
				oneshot_pending = false;
				return false;
			}
		}
		oneshot_trigger_ms = 0;
		oneshot_pending = false;
		if (status & STAT_OVERFLOW_MASK) {
			if (!last_overflow) {
				LOG_INF("Magnetometer overflow");
			}
			last_overflow = true;
			return false;
		}
		last_overflow = false;
	}
	if (!was_oneshot && sensor_interface_get_spec(SENSOR_INTERFACE_DEV_MAG) != SENSOR_INTERFACE_SPEC_EXT) {
		uint8_t status = 0;
		int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_STAT_REG, &status);
		if (err) {
			LOG_ERR("Communication error");
			return false;
		}
		if (!(status & STAT_DATA_RDY_MASK) || (status & STAT_OVERFLOW_MASK)) {
			return false;
		}
	}
	uint8_t rawData[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, QMC5883P_OUTX_L_REG, rawData, 6);
	if (err) {
		LOG_ERR("Communication error");
		return false;
	}
	// Reject byte-identical readings until the cached period plus its integer
	// 10% margin expires. A new conversion can have identical values, so this
	// heuristic trades exact freshness detection for bounded duplicate suppression.
	int64_t now = k_uptime_get();
	if (last_raw_sample_valid && memcmp(rawData, last_raw_sample, 6) == 0) {
		if ((now - last_mag_time_ms) < (mag_period_ms + mag_period_ms / 10)) {
			return false;
		}
		// Advance one cached period; resynchronize to now only when far behind.
		last_mag_time_ms += mag_period_ms;
		if (now - last_mag_time_ms > mag_period_ms * 2) {
			last_mag_time_ms = now;
		}
	} else {
		last_mag_time_ms = now;
	}
	memcpy(last_raw_sample, rawData, 6);
	last_raw_sample_valid = true;
	qmc5883p_mag_process(rawData, m);
	return true;
}

void qmc5883p_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		uint16_t raw = ((uint16_t)raw_m[i * 2 + 1] << 8) | raw_m[i * 2];
		m[i] = (int16_t)raw;
		m[i] *= sensitivity; // Gauss
	}
}

const sensor_mag_t sensor_mag_qmc5883p
	= {qmc5883p_init,
	   qmc5883p_shutdown,

	   qmc5883p_update_odr,

	   qmc5883p_mag_oneshot,
	   qmc5883p_mag_read,
	   mag_none_temp_read,

	   qmc5883p_mag_process,
	   6,
	   6};
