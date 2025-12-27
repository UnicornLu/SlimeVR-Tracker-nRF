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

#ifndef BNO085_h
#define BNO085_h

#include "sensor/sensor.h"

// BNO085 uses SHTP (Sensor Hub Transport Protocol) over I2C/SPI
// I2C addresses: 0x4A (default), 0x4B (if ADR pin is high)

// SHTP Report IDs
#define BNO085_REPORT_ID_PRODUCT_ID_RESPONSE    0xF1
#define BNO085_REPORT_ID_GET_FEATURE_RESPONSE   0xF3
#define BNO085_REPORT_ID_COMMAND_RESPONSE       0xF1
#define BNO085_REPORT_ID_INPUT_SENSOR_REPORT    0xFC

// Feature Report IDs
#define BNO085_FEATURE_REPORT_ACCELEROMETER     0x01
#define BNO085_FEATURE_REPORT_GYROSCOPE         0x02
#define BNO085_FEATURE_REPORT_MAGNETOMETER      0x03
#define BNO085_FEATURE_REPORT_ROTATION_VECTOR   0x05
#define BNO085_FEATURE_REPORT_GAME_ROTATION_VECTOR 0x08
#define BNO085_FEATURE_REPORT_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define BNO085_FEATURE_REPORT_TEMPERATURE       0x13

// SHTP Commands
#define BNO085_COMMAND_PRODUCT_ID_REQUEST       0xF9
#define BNO085_COMMAND_GET_FEATURE_REQUEST      0xFE
#define BNO085_COMMAND_FEATURE_ENABLE           0xFD
#define BNO085_COMMAND_FEATURE_DISABLE           0xFC
#define BNO085_COMMAND_MEAP                     0x10

// SHTP Channel IDs
#define BNO085_CHANNEL_COMMAND                  0x00
#define BNO085_CHANNEL_EXECUTABLE               0x01
#define BNO085_CHANNEL_CONTROL                  0x02
#define BNO085_CHANNEL_INPUT_SENSOR             0x04
#define BNO085_CHANNEL_INPUT_GYRO               0x05

// SHTP Packet structure
#define BNO085_PACKET_HEADER_SIZE               4
#define BNO085_PACKET_MAX_SIZE                  512

// Default report intervals (in microseconds)
#define BNO085_REPORT_INTERVAL_ACCEL_DEFAULT    50000  // 20 Hz
#define BNO085_REPORT_INTERVAL_GYRO_DEFAULT     50000  // 20 Hz

int bno085_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);
void bno085_shutdown(void);

void bno085_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range);
int bno085_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);

uint16_t bno085_fifo_read(uint8_t *data, uint16_t len);
int bno085_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void bno085_accel_read(float a[3]);
void bno085_gyro_read(float g[3]);
float bno085_temp_read(void);

uint8_t bno085_setup_DRDY(uint16_t threshold);
uint8_t bno085_setup_WOM(void);

int bno085_ext_setup(void);
int bno085_ext_passthrough(bool passthrough);

extern const sensor_imu_t sensor_imu_bno085;

#endif

