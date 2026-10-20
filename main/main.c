#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_sensor_client.h"

#include "cJSON.h" 
#include "time.h"
#include "math.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "board.h"
#include "spl_meter.h"

#include <wifi_provisioning/manager.h>
#include "ota_update.h"
extern void provisioning_main(void);

static const char *TAG = "sound_scanner";
static bool new_device_seen = false;

/* GPIO hold detection */
#define HOLD_GPIO_PIN GPIO_NUM_9   // change to the pin you want to monitor
static volatile bool gpio_hold_detected = false;

#define LED_GPIO_PIN GPIO_NUM_3

static const TickType_t fast_led_blink_interval = pdMS_TO_TICKS(100);

enum {
    NORMAL_MODE,
    PROVISIONING_MODE
};

static int current_mode = NORMAL_MODE;


static void gpio_hold_monitor_task(void *arg)
{
    const int64_t required_hold_ms = 5000; // 5 seconds
    const int64_t poll_interval_ms = 100;  // check every 100ms
    int64_t low_start_ms = -1;

    for (;;) {
        int level = gpio_get_level(HOLD_GPIO_PIN);
        int64_t now_ms = esp_timer_get_time() / 1000;

        if (level == 0) {
            if (low_start_ms < 0) {
                low_start_ms = now_ms;
            } else if ((now_ms - low_start_ms) >= required_hold_ms) {
                if (!gpio_hold_detected) {
                    gpio_hold_detected = true;
                    ESP_LOGI(TAG, "GPIO %d held low for %lld ms -> gpio_hold_detected=true", HOLD_GPIO_PIN, required_hold_ms);
                    wifi_prov_mgr_reset_provisioning();
                    current_mode = PROVISIONING_MODE;
                    vTaskDelay(pdMS_TO_TICKS(5000)); // Short delay to allow provisioning mode to start
                    esp_restart(); // Restart to ensure we enter provisioning mode cleanly
                }
            }
        } else {
            if (low_start_ms >= 0) low_start_ms = -1;
            if (gpio_hold_detected) {
                gpio_hold_detected = false;
                ESP_LOGI(TAG, "GPIO %d released -> gpio_hold_detected=false", HOLD_GPIO_PIN);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
    }
}


static void led_blink_task(void *arg)
{
    int level = 0;
    for (;;) {
        if (current_mode == NORMAL_MODE) {
            board_led_operation(LED_G, LED_ON);
            vTaskDelay(pdMS_TO_TICKS(25));
            board_led_operation(LED_G, LED_OFF);
            vTaskDelay(pdMS_TO_TICKS(975));
        } else {
            level = !level;
            vTaskDelay(fast_led_blink_interval);
            board_led_operation(LED_G, LED_TOGGLE);
        }
    }
}

void set_log_levels() {
    esp_log_level_set("LSM6",ESP_LOG_INFO);
    esp_log_level_set("HDC1080",ESP_LOG_INFO);
    esp_log_level_set("VL53L_TASK",ESP_LOG_INFO);
    esp_log_level_set("BATTERY",ESP_LOG_DEBUG);
    esp_log_level_set("OTA_UPDATE",ESP_LOG_INFO);
    esp_log_level_set("BOARD",ESP_LOG_WARN);
}

void app_main(void) {
    // Initialize NVS (required for BLE)

    time_t ota_check_start_time;

    ESP_LOGI(TAG, "[APP] App Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());


    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    set_log_levels();
    board_init();



    // Configure GPIO for hold detection before starting the monitor task
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HOLD_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Start the GPIO hold monitor task
    xTaskCreatePinnedToCore(gpio_hold_monitor_task, "gpio_hold_mon", 2048, NULL, 5, NULL, tskNO_AFFINITY);    
        // Start LED blink task
    xTaskCreatePinnedToCore(led_blink_task, "led_blink", 2048, NULL, 5, NULL, tskNO_AFFINITY);

    current_mode = PROVISIONING_MODE;

    provisioning_main();

    current_mode = NORMAL_MODE;

    //At this point we should be connected to Wi-Fi and ready to start BLE scanning and MQTT client
    //check if there is any new firmware and update if needed before starting main application tasks

    //Perform OTA check if required
    if (CONFIG_OTA_UPDATE_ENABLE) {
        ota_main();
        ota_check_start_time = time(NULL);
    }


    ESP_LOGI(TAG, "Init MQTT client");
    mqtt_main();
    
    ESP_LOGI(TAG, "Starting Sound Scanner");
    
   
    // Start the sound pressure level meter (publishes Leq via MQTT)
    if (spl_meter_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SPL meter");
    }
    
    // Keep the task running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (CONFIG_OTA_UPDATE_ENABLE) {        
            // check for OTA updates every 12 hours
            if (difftime(time(NULL), ota_check_start_time) >= 12 * 3600) {
                ota_main();
                ota_check_start_time = time(NULL);
            }
        }        
    }
}