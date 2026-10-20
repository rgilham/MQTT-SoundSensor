/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"
#include "i2c_bus.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#define TAG "BOARD"

struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;
        }
        if (onoff == LED_TOGGLE) {
            onoff = !led_state[i].previous;
        }
        else if (onoff == led_state[i].previous) {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;
        return;
    }

    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(led_state[i].pin);
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_state[i].pin, LED_OFF);
        led_state[i].previous = LED_OFF;
    }
}


void board_cfg_io()
{
    // 2. Configure Pin for Wakeup
    // Note: External pullup is recommended for low-level trigger
    uint32_t pin_mask = (1ULL << HALL_PIN) | (1ULL << INT0_PIN) | (1ULL << BOOT_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Use internal pullup
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}


/* Configure GPIO2 to wake the chip from deep sleep when pulled LOW */
void board_configure_ext_wakeup_src(void)
{
    

    ESP_LOGI(TAG,"Hall level %d IMU_Level %d boot level %d",gpio_get_level(HALL_PIN),gpio_get_level(INT0_PIN),gpio_get_level(BOOT_PIN));
    vTaskDelay(10);
    uint32_t pin_mask = (1ULL << HALL_PIN) | (1ULL << INT0_PIN) ;
    /* Enable wakeup on low level (0 = wake on low) using ext0 (single RTC IO) */
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(pin_mask,ESP_EXT1_WAKEUP_ANY_LOW));
    //ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(pin_mask, ESP_GPIO_WAKEUP_GPIO_LOW));
    //ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup((1ULL << HALL_PIN), ESP_GPIO_WAKEUP_GPIO_LOW));    
    //ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup((1ULL << INT0_PIN), ESP_GPIO_WAKEUP_GPIO_LOW));    
}

int read_imu_wakeup_state()
{
    ESP_LOGI(TAG,"IMU level %d",gpio_get_level(INT0_PIN));
    return gpio_get_level(INT0_PIN)?0:1; //return 1 when interrupt is active (active low)
}

int read_hall_state()
{
    return gpio_get_level(HALL_PIN)?0:1; //return 1 when magnet
}

int read_boot_pin_state()
{
    return gpio_get_level(BOOT_PIN)?0:1; //return 1 when pressed
}

void board_init(void)
{
    board_led_init();
    i2c_init_master();
    board_cfg_io();
}
