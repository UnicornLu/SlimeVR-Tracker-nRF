#ifndef LED_TRI_HOST_ESB_H
#define LED_TRI_HOST_ESB_H

#include <stdbool.h>
#include <stdint.h>

/* Receiver time snapshot (32768 Hz ticks). */
bool esb_get_status_clock(uint32_t *local_ticks, uint32_t *network_ticks);

#endif
