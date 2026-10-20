#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include "freertos/FreeRTOS.h"

/**
 * @brief Initialize the serial command task
 * 
 * Creates and starts a FreeRTOS task that reads commands from the USB serial console.
 * The task processes commands to update MQTT credentials and broker URI, storing them in NVS.
 * 
 * Supported commands:
 * - username <value>  : Set MQTT username
 * - password <value>  : Set MQTT password  
 * - uri <value>       : Set MQTT broker URI
 * - status            : Display status
 * - help              : Display help
 * - clear             : Clear terminal
 * 
 * @param stack_size Stack size in bytes for the task (recommended: 8192 minimum)
 * @param priority Task priority (recommended: tskIDLE_PRIORITY + 1 or 2)
 * 
 * Example usage:
 *   serial_commands_init(4096, tskIDLE_PRIORITY + 1);
 */
void serial_commands_init(uint32_t stack_size, UBaseType_t priority);

#endif /* SERIAL_COMMANDS_H */
