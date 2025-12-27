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

#ifndef BNO085_MAG_h
#define BNO085_MAG_h

#include "sensor/sensor.h"

// BNO085 magnetometer uses SHTP protocol through the IMU interface
// Feature Report ID for magnetometer
#define BNO085_MAG_FEATURE_REPORT_ID 0x03

// Default report interval (in microseconds)
#define BNO085_MAG_REPORT_INTERVAL_DEFAULT 50000  // 20 Hz

int bno085_mag_init(float time, float *actual_time);
void bno085_mag_shutdown(void);

int bno085_mag_update_odr(float time, float *actual_time);

void bno085_mag_oneshot(void);
void bno085_mag_read(float m[3]);
float bno085_mag_temp_read(float bias[3]);

void bno085_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_bno085;

#endif

