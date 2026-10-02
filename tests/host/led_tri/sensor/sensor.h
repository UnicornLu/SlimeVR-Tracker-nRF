#ifndef LED_TRI_HOST_SENSOR_H
#define LED_TRI_HOST_SENSOR_H

#include <stdint.h>

/* Milliseconds since the last observation that was not quiet (resting dim). */
int64_t sensor_ms_since_motion(void);

#endif
