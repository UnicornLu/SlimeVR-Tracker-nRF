#ifndef LED_TRI_HOST_ESB_OTA_H
#define LED_TRI_HOST_ESB_OTA_H

#include <stdbool.h>

/* True while an OTA session owns the blue channel (exempts the resting dim). */
bool esb_ota_is_active(void);

#endif
