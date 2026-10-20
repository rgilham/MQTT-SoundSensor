#ifndef I2C_BUS_H
#define I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

// Add your includes here
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"

// Add your macro definitions here

#define I2C_MASTER_FREQ_HZ   100000


// Add your type definitions here

extern SemaphoreHandle_t i2c0_mutex;
extern i2c_master_bus_handle_t bus_handle;

// Add your function declarations here
void i2c_init_master(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_BUS_H