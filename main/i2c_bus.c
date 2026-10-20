#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "i2c_bus.h"




SemaphoreHandle_t i2c0_mutex;
i2c_master_bus_handle_t bus_handle;

#define I2C_MASTER_FREQ_HZ   100000
#define PORT_NUMBER -1

void i2c_init_master(void) {

    i2c0_mutex = xSemaphoreCreateMutex();
    i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .sda_io_num = CONFIG_BLE_VIBSENSOR_I2C_MASTER_SDA,
        .scl_io_num = CONFIG_BLE_VIBSENSOR_I2C_MASTER_SCL
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));


}