#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"
#include "esp_wifi.h"

void ota_main(void);

#ifdef __cplusplus
}
#endif
#endif // OTA_UPDATE_H