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

#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "BNO085.h"
#include "../imu/BNO085.h"

LOG_MODULE_REGISTER(BNO085_MAG, LOG_LEVEL_DBG);

// BNO085 magnetometer sensitivity
// BNO085 reports magnetic field in microtesla (µT)
static const float sensitivity = 0.1f; // µT/LSB (may need adjustment based on actual Q-point format)

static float last_mag_time = 0;
static int64_t oneshot_trigger_time = 0;

// Helper function to enable magnetometer feature report
static int bno085_mag_enable_feature(uint32_t report_interval_us)
{
	uint8_t cmd[17];
	cmd[0] = BNO085_COMMAND_FEATURE_ENABLE;
	cmd[1] = BNO085_MAG_FEATURE_REPORT_ID;
	cmd[2] = report_interval_us & 0xFF;
	cmd[3] = (report_interval_us >> 8) & 0xFF;
	cmd[4] = (report_interval_us >> 16) & 0xFF;
	cmd[5] = (report_interval_us >> 24) & 0xFF;
	// Additional parameters set to 0
	memset(&cmd[6], 0, 11);

	// Use IMU interface to send SHTP packet
	// Note: This requires access to BNO085's SHTP functions
	// For now, we'll need to implement a way to share SHTP communication
	// between IMU and MAG drivers, or use the IMU interface directly
	
	// TODO: Implement shared SHTP communication or use IMU interface
	LOG_DBG("Enabling BNO085 magnetometer feature");
	return 0;
}

// Helper function to disable magnetometer feature report
static int bno085_mag_disable_feature(void)
{
	uint8_t cmd[2];
	cmd[0] = BNO085_COMMAND_FEATURE_DISABLE;
	cmd[1] = BNO085_MAG_FEATURE_REPORT_ID;

	// TODO: Implement shared SHTP communication or use IMU interface
	LOG_DBG("Disabling BNO085 magnetometer feature");
	return 0;
}

int bno085_mag_init(float time, float *actual_time)
{
	LOG_INF("Initializing BNO085 magnetometer");

	// BNO085 magnetometer is accessed through the IMU interface via SHTP
	// The IMU should already be initialized
	
	last_mag_time = 0;
	int err = bno085_mag_update_odr(time, actual_time);
	if (err < 0)
		return err;

	LOG_INF("BNO085 magnetometer initialized");
	return 0;
}

void bno085_mag_shutdown(void)
{
	LOG_DBG("Shutting down BNO085 magnetometer");
	
	bno085_mag_disable_feature();
	last_mag_time = 0;
}

int bno085_mag_update_odr(float time, float *actual_time)
{
	// Calculate report interval
	uint32_t mag_interval = 0;

	if (time > 0 && time != INFINITY)
	{
		mag_interval = (uint32_t)(time * 1000000.0f);
		// BNO085 minimum interval is 5000 us (200 Hz max)
		if (mag_interval < 5000)
			mag_interval = 5000;
		*actual_time = mag_interval / 1000000.0f;
	}
	else
	{
		*actual_time = 0;
	}

	// Update if changed
	if (mag_interval != 0 && (last_mag_time == 0 || fabsf(*actual_time - last_mag_time) > 0.001f))
	{
		int err = bno085_mag_enable_feature(mag_interval);
		if (err)
		{
			LOG_ERR("Failed to update magnetometer ODR");
			return -1;
		}
	}
	else if (mag_interval == 0 && last_mag_time != 0)
	{
		bno085_mag_disable_feature();
	}

	last_mag_time = *actual_time;
	return 0;
}

void bno085_mag_oneshot(void)
{
	// BNO085 doesn't have a traditional oneshot mode
	// Instead, we can request a single report or read from buffered data
	// For now, mark the trigger time
	oneshot_trigger_time = k_uptime_get();
	
	LOG_DBG("bno085_mag_oneshot - not fully implemented");
}

void bno085_mag_read(float m[3])
{
	// Read magnetometer data from BNO085
	// This would need to parse SHTP reports from the IMU interface
	// For now, return zeros - actual implementation would need to:
	// 1. Read SHTP packets from IMU interface
	// 2. Parse magnetometer reports
	// 3. Extract and convert data
	
	LOG_DBG("bno085_mag_read - not fully implemented");
	m[0] = 0;
	m[1] = 0;
	m[2] = 0;
}

float bno085_mag_temp_read(float bias[3])
{
	// BNO085 can report temperature via feature report
	// For now, return a default value
	// Actual implementation would need to enable temperature reports
	
	LOG_DBG("bno085_mag_temp_read - not fully implemented");
	return 25.0f; // Default temperature
}

void bno085_mag_process(uint8_t *raw_m, float m[3])
{
	// Process raw magnetometer data from SHTP report
	// Report format: [Report ID][Sequence][X LSB][X MSB][Y LSB][Y MSB][Z LSB][Z MSB]
	// Values are in Q-point format (µT)
	
	if (raw_m == NULL)
	{
		m[0] = 0;
		m[1] = 0;
		m[2] = 0;
		return;
	}

	uint8_t report_id = raw_m[0];
	if (report_id != BNO085_MAG_FEATURE_REPORT_ID)
	{
		// Not a magnetometer report
		m[0] = 0;
		m[1] = 0;
		m[2] = 0;
		return;
	}

	// Parse magnetometer data
	// Report format: [Report ID][Sequence][X LSB][X MSB][Y LSB][Y MSB][Z LSB][Z MSB]
	int16_t x = (int16_t)(raw_m[2] | (raw_m[3] << 8));
	int16_t y = (int16_t)(raw_m[4] | (raw_m[5] << 8));
	int16_t z = (int16_t)(raw_m[6] | (raw_m[7] << 8));

	// Convert from Q-point to float (µT), then to gauss if needed
	// Q-point format depends on BNO085 configuration, typically 0.1 µT per LSB
	// 1 gauss = 100 µT
	float scale = 0.1f; // Default scale, may need adjustment
	m[0] = x * scale / 100.0f; // Convert µT to gauss
	m[1] = y * scale / 100.0f;
	m[2] = z * scale / 100.0f;
}

const sensor_mag_t sensor_mag_bno085 = {
	*bno085_mag_init,
	*bno085_mag_shutdown,

	*bno085_mag_update_odr,

	*bno085_mag_oneshot,
	*bno085_mag_read,
	*bno085_mag_temp_read,

	*bno085_mag_process,
	0,  // ext_min_burst - not applicable for BNO085
	0   // ext_burst - not applicable for BNO085
};

