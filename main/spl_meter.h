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

/**
 * @brief Get the current SPL MQTT publish threshold.
 * @return Threshold in dB
 */
double spl_threshold_get(void);

/**
 * @brief Set the SPL MQTT publish threshold.
 * @param threshold_db Threshold in dB (0-150), only publish readings above this level
 */
void spl_threshold_save(double threshold_db);

/**
 * @brief Get the current FFT spectrum reporting enabled state.
 * @return true if FFT spectrum is reported when threshold is exceeded, false otherwise
 */
bool fft_report_enabled_get(void);

/**
 * @brief Enable or disable FFT spectrum reporting when the SPL threshold is exceeded.
 * @param enabled true to enable, false to disable
 */
void fft_report_enabled_set(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* SPL_METER_H */
