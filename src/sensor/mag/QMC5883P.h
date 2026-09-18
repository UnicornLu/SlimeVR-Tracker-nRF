#ifndef QMC5883P_h
#define QMC5883P_h

#include "sensor/sensor.h"

int qmc5883p_init(float period_s, float *actual_period_s);
void qmc5883p_shutdown(void);

int qmc5883p_update_odr(float period_s, float *actual_period_s);

void qmc5883p_mag_oneshot(void);
bool qmc5883p_mag_read(float m[3]);

void qmc5883p_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_qmc5883p;

#endif
