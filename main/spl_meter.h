#ifndef SPL_METER_H
#define SPL_METER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the sound pressure level meter.
 *
 * Initialises the I2S microphone and starts the sampling/filtering task and the
 * task that calculates the Leq value and publishes it via MQTT
 * (publish_sensor_string()). Call after mqtt_main().
 *
 * Based on "Display A-weighted sound level measured by I2S Microphone"
 * (c)2019 Ivan Kostoski, GPLv3.
 *
 * @return ESP_OK on success
 */
esp_err_t spl_meter_start(void);

#ifdef __cplusplus
}
#endif

#endif /* SPL_METER_H */
