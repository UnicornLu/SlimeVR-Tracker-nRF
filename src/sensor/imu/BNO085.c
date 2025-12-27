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
#include <hal/nrf_gpio.h>

#include "BNO085.h"
#include "sensor/sensor_none.h"

LOG_MODULE_REGISTER(BNO085, LOG_LEVEL_DBG);

// BNO085 uses SHTP protocol, not traditional register-based communication
// This is a basic implementation framework

static float accel_sensitivity = 1.0f; // BNO085 reports in m/s^2, convert to g
static float gyro_sensitivity = 1.0f;  // BNO085 reports in rad/s, convert to deg/s

static float last_accel_time = 0;
static float last_gyro_time = 0;

// SHTP packet buffer (for temporary storage)
static uint8_t shtp_buffer[BNO085_PACKET_MAX_SIZE];

// Helper function to send SHTP packet
static int bno085_send_shtp_packet(uint8_t channel, uint8_t *data, uint16_t len)
{
	if (len > BNO085_PACKET_MAX_SIZE - BNO085_PACKET_HEADER_SIZE)
	{
		LOG_ERR("SHTP packet too large: %d", len);
		return -1;
	}

	// Build SHTP header
	uint8_t header[BNO085_PACKET_HEADER_SIZE];
	header[0] = len & 0xFF;           // Length LSB
	header[1] = (len >> 8) & 0xFF;   // Length MSB
	header[2] = channel;              // Channel ID
	header[3] = 0;                   // Sequence number (not used for now)

	// Send header
	int err = ssi_write(SENSOR_INTERFACE_DEV_IMU, header, BNO085_PACKET_HEADER_SIZE);
	if (err)
		return err;

	// Send data
	if (len > 0)
	{
		err = ssi_write(SENSOR_INTERFACE_DEV_IMU, data, len);
		if (err)
			return err;
	}

	return 0;
}

// Helper function to receive SHTP packet
static int bno085_receive_shtp_packet(uint8_t *channel, uint8_t *data, uint16_t *len)
{
	uint8_t header[BNO085_PACKET_HEADER_SIZE];
	
	// Read header
	int err = ssi_read(SENSOR_INTERFACE_DEV_IMU, header, BNO085_PACKET_HEADER_SIZE);
	if (err)
		return err;

	*len = header[0] | (header[1] << 8);
	*channel = header[2];

	if (*len > BNO085_PACKET_MAX_SIZE - BNO085_PACKET_HEADER_SIZE)
	{
		LOG_ERR("SHTP packet length too large: %d", *len);
		return -1;
	}

	if (*len > 0)
	{
		err = ssi_read(SENSOR_INTERFACE_DEV_IMU, data, *len);
		if (err)
			return err;
	}

	return 0;
}

// Request product ID
static int bno085_request_product_id(void)
{
	uint8_t cmd[4];
	cmd[0] = BNO085_COMMAND_PRODUCT_ID_REQUEST;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	cmd[3] = 0x00;

	return bno085_send_shtp_packet(BNO085_CHANNEL_COMMAND, cmd, 4);
}

// Enable feature report
static int bno085_enable_feature(uint8_t feature_id, uint32_t report_interval_us)
{
	uint8_t cmd[17];
	cmd[0] = BNO085_COMMAND_FEATURE_ENABLE;
	cmd[1] = feature_id;
	cmd[2] = report_interval_us & 0xFF;
	cmd[3] = (report_interval_us >> 8) & 0xFF;
	cmd[4] = (report_interval_us >> 16) & 0xFF;
	cmd[5] = (report_interval_us >> 24) & 0xFF;
	// Additional parameters set to 0
	memset(&cmd[6], 0, 11);

	return bno085_send_shtp_packet(BNO085_CHANNEL_COMMAND, cmd, 17);
}

// Disable feature report
static int bno085_disable_feature(uint8_t feature_id)
{
	uint8_t cmd[2];
	cmd[0] = BNO085_COMMAND_FEATURE_DISABLE;
	cmd[1] = feature_id;

	return bno085_send_shtp_packet(BNO085_CHANNEL_COMMAND, cmd, 2);
}

int bno085_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	LOG_INF("Initializing BNO085");

	// BNO085 uses I2C, configure interface
	// Note: BNO085 doesn't use traditional register interface, but we still need I2C configured
	// The actual communication will be done through SHTP protocol

	// Wait for device to be ready
	k_msleep(100);

	// Request product ID to verify communication
	int err = bno085_request_product_id();
	if (err)
	{
		LOG_ERR("Failed to request product ID");
		return -1;
	}

	// Wait for response
	k_msleep(50);
	uint8_t channel;
	uint16_t len;
	err = bno085_receive_shtp_packet(&channel, shtp_buffer, &len);
	if (err || channel != BNO085_CHANNEL_COMMAND)
	{
		LOG_ERR("Failed to receive product ID response");
		return -1;
	}

	LOG_INF("BNO085 product ID response received");

	// Initialize ODR
	last_accel_time = 0;
	last_gyro_time = 0;
	err = bno085_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	if (err < 0)
		return err;

	// Enable accelerometer and gyroscope reports
	uint32_t accel_interval = (uint32_t)(*accel_actual_time * 1000000.0f);
	uint32_t gyro_interval = (uint32_t)(*gyro_actual_time * 1000000.0f);

	err = bno085_enable_feature(BNO085_FEATURE_REPORT_ACCELEROMETER, accel_interval);
	if (err)
	{
		LOG_ERR("Failed to enable accelerometer");
		return -1;
	}

	err = bno085_enable_feature(BNO085_FEATURE_REPORT_GYROSCOPE, gyro_interval);
	if (err)
	{
		LOG_ERR("Failed to enable gyroscope");
		return -1;
	}

	// Set sensitivity values
	// BNO085 reports acceleration in m/s^2, convert to g (divide by 9.80665)
	accel_sensitivity = 1.0f / 9.80665f;
	// BNO085 reports angular velocity in rad/s, convert to deg/s (multiply by 180/π)
	gyro_sensitivity = 180.0f / M_PI;

	LOG_INF("BNO085 initialized");
	return 0;
}

void bno085_shutdown(void)
{
	LOG_DBG("Shutting down BNO085");

	// Disable all features
	bno085_disable_feature(BNO085_FEATURE_REPORT_ACCELEROMETER);
	bno085_disable_feature(BNO085_FEATURE_REPORT_GYROSCOPE);
	bno085_disable_feature(BNO085_FEATURE_REPORT_MAGNETOMETER);

	last_accel_time = 0;
	last_gyro_time = 0;
}

void bno085_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	// BNO085 has fixed ranges:
	// Accelerometer: ±16g (reported in m/s^2)
	// Gyroscope: ±2000 dps (reported in rad/s)
	*accel_actual_range = 16.0f;
	*gyro_actual_range = 2000.0f;
}

int bno085_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int err = 0;

	// Calculate report intervals
	uint32_t accel_interval = 0;
	uint32_t gyro_interval = 0;

	if (accel_time > 0 && accel_time != INFINITY)
	{
		accel_interval = (uint32_t)(accel_time * 1000000.0f);
		// BNO085 minimum interval is 5000 us (200 Hz max)
		if (accel_interval < 5000)
			accel_interval = 5000;
		*accel_actual_time = accel_interval / 1000000.0f;
	}
	else
	{
		*accel_actual_time = 0;
	}

	if (gyro_time > 0 && gyro_time != INFINITY)
	{
		gyro_interval = (uint32_t)(gyro_time * 1000000.0f);
		// BNO085 minimum interval is 5000 us (200 Hz max)
		if (gyro_interval < 5000)
			gyro_interval = 5000;
		*gyro_actual_time = gyro_interval / 1000000.0f;
	}
	else
	{
		*gyro_actual_time = 0;
	}

	// Update if changed
	if (accel_interval != 0 && (last_accel_time == 0 || fabsf(*accel_actual_time - last_accel_time) > 0.001f))
	{
		err = bno085_enable_feature(BNO085_FEATURE_REPORT_ACCELEROMETER, accel_interval);
		if (err)
			LOG_ERR("Failed to update accelerometer ODR");
	}

	if (gyro_interval != 0 && (last_gyro_time == 0 || fabsf(*gyro_actual_time - last_gyro_time) > 0.001f))
	{
		err = bno085_enable_feature(BNO085_FEATURE_REPORT_GYROSCOPE, gyro_interval);
		if (err)
			LOG_ERR("Failed to update gyroscope ODR");
	}

	last_accel_time = *accel_actual_time;
	last_gyro_time = *gyro_actual_time;

	return (err < 0 ? err : 0);
}

uint16_t bno085_fifo_read(uint8_t *data, uint16_t len)
{
	// BNO085 doesn't have a traditional FIFO
	// Instead, it sends SHTP reports asynchronously
	// This function should read available reports from the input sensor channel
	
	uint16_t total = 0;
	uint8_t channel;
	uint16_t packet_len;

	// Try to read available packets
	while (total < len && total < BNO085_PACKET_MAX_SIZE)
	{
		int err = bno085_receive_shtp_packet(&channel, &data[total], &packet_len);
		if (err)
			break;

		// Only process input sensor channel packets
		if (channel == BNO085_CHANNEL_INPUT_SENSOR)
		{
			total += packet_len;
		}
		else
		{
			// Handle other channels if needed
			LOG_DBG("Received packet on channel %d, length %d", channel, packet_len);
		}
	}

	return total;
}

int bno085_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	// Process SHTP report packet
	// Report format: [Report ID][Sequence][Data...]
	// Note: data buffer contains SHTP packets from fifo_read
	// Each packet starts with a 4-byte header: [Length LSB][Length MSB][Channel][Sequence]
	// Then follows the actual report data
	
	// For now, assume data is already parsed and contains report data
	// In a full implementation, you would need to parse the SHTP header first
	uint8_t *packet = &data[index];
	uint8_t report_id = packet[0];

	// Process accelerometer report
	if (report_id == BNO085_FEATURE_REPORT_ACCELEROMETER)
	{
		// Report format: [Report ID][Sequence][X LSB][X MSB][Y LSB][Y MSB][Z LSB][Z MSB]
		// Values are in Q-point format (m/s^2)
		int16_t x = (int16_t)(packet[2] | (packet[3] << 8));
		int16_t y = (int16_t)(packet[4] | (packet[5] << 8));
		int16_t z = (int16_t)(packet[6] | (packet[7] << 8));

		// Convert from Q-point to float (m/s^2), then to g
		// Q-point format depends on BNO085 configuration, typically 1/100 m/s^2 per LSB
		float scale = 0.01f; // Default scale, may need adjustment
		a[0] = x * scale * accel_sensitivity;
		a[1] = y * scale * accel_sensitivity;
		a[2] = z * scale * accel_sensitivity;
	}

	// Process gyroscope report
	if (report_id == BNO085_FEATURE_REPORT_GYROSCOPE)
	{
		// Report format: [Report ID][Sequence][X LSB][X MSB][Y LSB][Y MSB][Z LSB][Z MSB]
		// Values are in Q-point format (rad/s)
		int16_t x = (int16_t)(packet[2] | (packet[3] << 8));
		int16_t y = (int16_t)(packet[4] | (packet[5] << 8));
		int16_t z = (int16_t)(packet[6] | (packet[7] << 8));

		// Convert from Q-point to float (rad/s), then to deg/s
		// Q-point format depends on BNO085 configuration, typically 1/900 rad/s per LSB
		float scale = 1.0f / 900.0f; // Default scale, may need adjustment
		g[0] = x * scale * gyro_sensitivity;
		g[1] = y * scale * gyro_sensitivity;
		g[2] = z * scale * gyro_sensitivity;
	}

	return 0;
}

void bno085_accel_read(float a[3])
{
	// Request immediate accelerometer reading
	// This is a simplified implementation
	// In practice, you would need to request a one-shot report or read from buffered data
	
	// For now, return zeros - actual implementation would need to:
	// 1. Request one-shot report
	// 2. Wait for response
	// 3. Parse the report
	
	LOG_DBG("bno085_accel_read - not fully implemented");
	a[0] = 0;
	a[1] = 0;
	a[2] = 0;
}

void bno085_gyro_read(float g[3])
{
	// Request immediate gyroscope reading
	// This is a simplified implementation
	// In practice, you would need to request a one-shot report or read from buffered data
	
	LOG_DBG("bno085_gyro_read - not fully implemented");
	g[0] = 0;
	g[1] = 0;
	g[2] = 0;
}

float bno085_temp_read(void)
{
	// BNO085 can report temperature via feature report
	// For now, return a default value
	// Actual implementation would need to enable temperature reports
	
	LOG_DBG("bno085_temp_read - not fully implemented");
	return 25.0f; // Default temperature
}

uint8_t bno085_setup_DRDY(uint16_t threshold)
{
	// BNO085 uses INT pin to indicate data ready
	// The interrupt is triggered when new sensor reports are available
	// This is typically configured via the MEAP (Metaware Embedded Application Protocol) command
	
	LOG_DBG("bno085_setup_DRDY - not fully implemented");
	
	// Configure INT pin as active low, pull-up
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;
}

uint8_t bno085_setup_WOM(void)
{
	// BNO085 supports wake-on-motion via accelerometer
	// This would need to be configured via feature reports or MEAP commands
	
	LOG_DBG("bno085_setup_WOM - not fully implemented");
	
	// Configure INT pin as active low, pull-up
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;
}

int bno085_ext_setup(void)
{
	// BNO085 doesn't support external sensor passthrough in the traditional sense
	// It has its own internal sensors
	return -1;
}

int bno085_ext_passthrough(bool passthrough)
{
	// BNO085 doesn't support external sensor passthrough
	return -1;
}

const sensor_imu_t sensor_imu_bno085 = {
	*bno085_init,
	*bno085_shutdown,

	*bno085_update_fs,
	*bno085_update_odr,

	*bno085_fifo_read,
	*bno085_fifo_process,
	*bno085_accel_read,
	*bno085_gyro_read,
	*bno085_temp_read,

	*bno085_setup_DRDY,
	*bno085_setup_WOM,

	*bno085_ext_setup,
	*bno085_ext_passthrough
};

