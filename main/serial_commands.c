#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include <fcntl.h>
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/usb_serial_jtag.h"  
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_task_wdt.h"
#include "esp_mac.h"


static const char *TAG = "serial_cmd";

// Forward declaration of the NVS functions from mqtt_client.c
extern void mqtt_nvs_save_parameters(const char *parameter, const char *value);
extern char* mqtt_nvs_get_parameter(const char *parameter);

#define SERIAL_CMD_BUFFER_SIZE 256

/**
 * @brief Command handler for 'username' command
 */
static int cmd_username(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: username <value>\n");
        return 1;
    }

    // Join all arguments after command name (in case value contains spaces)
    char username_value[SERIAL_CMD_BUFFER_SIZE] = {0};
    for (int i = 1; i < argc; i++) {
        if (i > 1) strcat(username_value, " ");
        strcat(username_value, argv[i]);
    }

    mqtt_nvs_save_parameters("username", username_value);
    ESP_LOGI(TAG, "Username updated: %s", username_value);
    printf("✓ Username set to: %s\n", username_value);
    return 0;
}

/**
 * @brief Command handler for 'password' command
 */
static int cmd_password(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: password <value>\n");
        return 1;
    }

    // Join all arguments after command name (in case value contains spaces)
    char password_value[SERIAL_CMD_BUFFER_SIZE] = {0};
    for (int i = 1; i < argc; i++) {
        if (i > 1) strcat(password_value, " ");
        strcat(password_value, argv[i]);
    }

    mqtt_nvs_save_parameters("password", password_value);
    ESP_LOGI(TAG, "Password updated");
    printf("✓ Password set (length: %zu characters)\n", strlen(password_value));
    return 0;
}

/**
 * @brief Command handler for 'uri' command
 */
static int cmd_uri(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: uri <value>\n");
        return 1;
    }

    // Join all arguments after command name (in case value contains spaces)
    char uri_value[SERIAL_CMD_BUFFER_SIZE] = {0};
    for (int i = 1; i < argc; i++) {
        if (i > 1) strcat(uri_value, " ");
        strcat(uri_value, argv[i]);
    }

    mqtt_nvs_save_parameters("broker_uri", uri_value);
    ESP_LOGI(TAG, "MQTT broker URI updated: %s", uri_value);
    printf("✓ Broker URI set to: %s\n", uri_value);
    return 0;
}

/**
 * @brief Command handler for 'status' command
 */
static int cmd_status(int argc, char **argv)
{
    printf("\n=== MQTT Configuration Status ===\n");

 
    uint8_t mac[6];
    char mac_str[18] = {0};

    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf((char *)mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    printf("Device MAC: %s\n", mac_str);
    
    char *username = mqtt_nvs_get_parameter("username");
    printf("Username  : %s\n", username ? username : "(not set)");
    if (username) free(username);
    
    char *password = mqtt_nvs_get_parameter("password");
    printf("Password  : %s\n", password ? "(set - " : "(not set)");
    if (password) {
        printf("%zu characters)\n", strlen(password));
        free(password);
    }
    
    char *broker_uri = mqtt_nvs_get_parameter("broker_uri");
    printf("Broker URI: %s\n", broker_uri ? broker_uri : "(not set)");
    if (broker_uri) free(broker_uri);
    
    printf("\nNote: Changes are saved to NVS and will persist after reboot\n");
    printf("==================================\n\n");
    return 0;
}

/**
 * @brief Command handler for 'restart' command
 */
static int cmd_restart(int argc, char **argv)
{
    printf("Restarting device in 2 seconds...\n");
    ESP_LOGI(TAG, "Device restart initiated");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return 0;
}

/**
 * @brief Register all console commands
 */
static void register_commands(void)
{
    // Register 'username' command
    esp_console_cmd_t username_cmd = {
        .command = "username",
        .help = "Set MQTT username",
        .hint = "<value>",
        .func = cmd_username,
        .argtable = NULL,
    };
    esp_console_cmd_register(&username_cmd);

    // Register 'password' command
    esp_console_cmd_t password_cmd = {
        .command = "password",
        .help = "Set MQTT password",
        .hint = "<value>",
        .func = cmd_password,
        .argtable = NULL,
    };
    esp_console_cmd_register(&password_cmd);

    // Register 'uri' command
    esp_console_cmd_t uri_cmd = {
        .command = "uri",
        .help = "Set MQTT broker URI (e.g., mqtts://broker.example.com:8883)",
        .hint = "<value>",
        .func = cmd_uri,
        .argtable = NULL,
    };
    esp_console_cmd_register(&uri_cmd);

    // Register 'status' command
    esp_console_cmd_t status_cmd = {
        .command = "status",
        .help = "Display current MQTT configuration status",
        .hint = NULL,
        .func = cmd_status,
        .argtable = NULL,
    };
    esp_console_cmd_register(&status_cmd);

    // Register 'restart' command
    esp_console_cmd_t restart_cmd = {
        .command = "restart",
        .help = "Restart the device",
        .hint = NULL,
        .func = cmd_restart,
        .argtable = NULL,
    };
    esp_console_cmd_register(&restart_cmd);

    esp_console_register_help_command();
}

/**
 * @brief Initialize console with history support
 */
static void initialize_console(void)
{
    esp_console_config_t console_config = {
        .max_cmdline_length = SERIAL_CMD_BUFFER_SIZE,
        .max_cmdline_args = 16,
        .hint_color = -1,
        .hint_bold = 0,
    };

    ESP_ERROR_CHECK(esp_console_init(&console_config));

    // Setup linenoise for command line editing
    //linenoiseSetMultiLine(0);

    // Enable history
    //linenoiseHistorySetMaxLen(10);

    linenoiseSetCompletionCallback(&esp_console_get_completion);
    //linenoiseSetHintsCallback(&esp_console_get_hint);
    linenoiseHistorySetMaxLen(8);

    printf("\n=== MQTT Configuration Terminal ===\n");
    printf("Available commands: username, password, uri, status, restart, help\n");
    printf("Type 'help' for more information\n");
    printf("=====================================\n\n");
}

void init_usb_cdc_console() {
    /* 1. Initialize the USB Serial/JTAG VFS driver */
    // 1. Configure and install the hardware peripheral driver
    usb_serial_jtag_driver_config_t usb_config = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    // 2. Register the virtual file system node ("/dev/usbserjtag")
    // Note: IDF automatically invokes this on boot if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
    ESP_ERROR_CHECK(usb_serial_jtag_vfs_register());

    // 3. Bind standard I/O streams to use this newly installed driver
    usb_serial_jtag_vfs_use_driver();

    // 4. Set correct line endings for linenoise and common monitors (CR / CRLF)
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    // 5. CRITICAL: Redirect standard streams and switch stdin to BLOCKING mode
    // This stops linenoise from instantly returning EOF/0 and looping forever!
    freopen("/dev/usbserjtag", "r", stdin);
    freopen("/dev/usbserjtag", "w", stdout);
    fcntl(fileno(stdin), F_SETFL, 0); 
}

/**
 * @brief Task for reading and processing serial commands using ESP console
 */
static void serial_command_task(void *pvParameters)
{
    // Disable task watchdog for this task since it blocks on user input
    esp_task_wdt_delete(NULL);
    init_usb_cdc_console();

    initialize_console();
    register_commands();


    ESP_LOGI(TAG, "Serial command task started");

    // Start the main command loop
    const char *prompt = LOG_COLOR_I "mqtt> " LOG_RESET_COLOR;
    
    while (1) {
        // Read a line with linenoise (provides editing, history, etc.)
        char *line = linenoise(prompt);
        if (line == NULL) {
            continue;  // EOF or error, try again
        }

        // Only process line if it contains real text characters
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);

            // Let the ESP framework tokenize, parse, match, and run the command
            int cmd_ret;
            esp_err_t err = esp_console_run(line, &cmd_ret);
            
            if (err == ESP_ERR_NOT_FOUND) {
                printf("Unknown command. Type 'help' for a full list.\n");
            } else if (err == ESP_ERR_INVALID_ARG) {
                printf("Command formatting error.\n");
            } else if (err == ESP_OK && cmd_ret != 0) {
                printf("Command executed but returned non-zero error code: %d\n", cmd_ret);
            }
        }

        linenoiseFree(line);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Initialize and start the serial command task
 * 
 * This should be called from the main application to enable serial command processing.
 * Uses the ESP console component for robust serial/USB terminal support.
 * 
 * @param stack_size Stack size for the task (use 8192 or larger for console)
 * @param priority Task priority (typically tskIDLE_PRIORITY + 1 or 2)
 */
void serial_commands_init(uint32_t stack_size, UBaseType_t priority)
{
    BaseType_t result = xTaskCreate(
        serial_command_task,           // Task function
        "serial_cmd",                  // Task name
        stack_size,                    // Stack size in words
        NULL,                          // Task parameters
        priority,                      // Task priority
        NULL                           // Task handle (NULL = don't need it)
    );

    if (result == pdPASS) {
        ESP_LOGI(TAG, "Serial command task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create serial command task");
    }
}
