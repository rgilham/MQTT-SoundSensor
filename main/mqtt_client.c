#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_app_desc.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include <sys/param.h>

#include "mqtt_sensor_client.h"
#include "cJSON.h" 

static const char *TAG = "mqtts_example";


static char *mqtt_uri = NULL;
static char *mqtt_username = NULL;
static char *mqtt_password = NULL;
static uint8_t *mqtt_cert_pem = NULL;

extern const uint8_t default_iot_inventech_pem_start[]   asm("_binary_iot_inventech_co_za_pem_start");
extern const uint8_t default_iot_inventech_pem_end[]     asm("_binary_iot_inventech_co_za_pem_end");


const char *default_base_topic = CONFIG_BASE_TOPIC;
const char *SYSTOMQTT = "SYStoMQTT/Gateway";
const char *MQTTTOSYS = "MQTTtoSYS/Gateway";
char mac_str[18] = {0};

static esp_mqtt_client_handle_t client = NULL;


void mqtt_nvs_save_parameters(const char *parameter, const char *value);

#define MALLOC_SPRINTF(dest, format, ...) do { \
    int len = snprintf(NULL, 0, format, __VA_ARGS__); \
    if (len >= 0) { \
        (dest) = (char *)malloc(len + 1); \
        if (dest != NULL) { \
            sprintf((dest), format, __VA_ARGS__); \
        } \
    } \
} while (0)

// Usage example:
// char *str;
// MALLOC_SPRINTF(str, "Value: %d, Text: %s", 10, "hello");
// // ... use str ...
// free(str);



//
// Note: this function is for testing purposes only publishing part of the active partition
//       (to be checked against the original binary)
//
// static void send_binary(esp_mqtt_client_handle_t client)
// {
//     esp_partition_mmap_handle_t out_handle;
//     const void *binary_address;
//     const esp_partition_t *partition = esp_ota_get_running_partition();
//     esp_partition_mmap(partition, 0, partition->size, ESP_PARTITION_MMAP_DATA, &binary_address, &out_handle);
//     // sending only the configured portion of the partition (if it's less than the partition size)
//     int binary_size = MIN(CONFIG_BROKER_BIN_SIZE_TO_SEND, partition->size);
//     int msg_id = esp_mqtt_client_publish(client, "/topic/binary", binary_address, binary_size, 0, 0);
//     ESP_LOGI(TAG, "binary sent with msg_id=%d", msg_id);
// }

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */

void get_mac_address() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf((char *)mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

const esp_netif_ip_info_t* get_ip_info() {
    static esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);
    return &ip_info;
}

void publish_gateway_status(const char *devices_json)
{

    char *topic_publish_buffer = NULL;
    int msg_id;
    if (strlen(devices_json) == 0) {
        ESP_LOGW(TAG, "No devices to publish");
        return;
    }

    MALLOC_SPRINTF(topic_publish_buffer, "%s/%s/%s/status", default_base_topic, SYSTOMQTT, mac_str);
    

    msg_id = esp_mqtt_client_publish(client, topic_publish_buffer, devices_json, 0, 0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    free(topic_publish_buffer);
    
        
}


void publish_sensor_string(const char* sensor_mac, const char* sensor_json_value)
{
    char *topic_publish_buffer  = NULL;
    int msg_id;

    MALLOC_SPRINTF(topic_publish_buffer, "%s/Gateway/%s/BTtoMQTT/%s/sensor", default_base_topic, mac_str, sensor_mac);
    ESP_LOGD(TAG, "topic %s message %s", topic_publish_buffer, sensor_json_value);
    msg_id = esp_mqtt_client_publish(client, topic_publish_buffer, sensor_json_value, 0, 0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    free(topic_publish_buffer);
}


void handle_event_data(esp_mqtt_event_handle_t event) {
    // This function can be expanded to handle incoming MQTT messages as needed

    const cJSON *json = cJSON_ParseWithLength(event->data, event->data_len);
    if (json == NULL) {
        ESP_LOGW(TAG, "Failed to parse JSON");
        return; 
    }
    const cJSON *command = cJSON_GetObjectItem(json, "cmd");
    if (command && cJSON_IsString(command)) {
        ESP_LOGI(TAG, "Received command: %s", command->valuestring);
        // Handle the command as needed
        if (strcmp(command->valuestring, "reboot") == 0) {
            ESP_LOGI(TAG, "Reboot command received, restarting...");
            esp_restart();
        }
        if (strcmp(command->valuestring, "certupdate") == 0) {
            ESP_LOGI(TAG, "Cert update command received, updating certificate...");
            //retrieve the certifcate from the cmd_data field
                const cJSON *cmd_data = cJSON_GetObjectItem(json, "cmd_data");
                if (cmd_data && cJSON_IsString(cmd_data)) {
                    const char *new_cert = cmd_data->valuestring;
                    // Here you would add code to securely store the new certificate and update the MQTT client configuration
                    ESP_LOGI(TAG, "New certificate received: %s", new_cert);
                    // For example, you might write the new certificate to NVS or a file and then restart the MQTT client to use it
                    mqtt_nvs_save_parameters("cert_pem", new_cert);

                } else {
                    ESP_LOGW(TAG, "No 'cmd_data' field in JSON or it's not a string");
                }   
        }
        if (strcmp(command->valuestring, "credentials") == 0) {
            ESP_LOGI(TAG, "Credentials update command received, updating credentials...");
            //retrieve the credentials from the cmd_data field
                const cJSON *cmd_data = cJSON_GetObjectItem(json, "cmd_data");
                if (cmd_data && cJSON_IsObject(cmd_data)) {
                    const cJSON *username = cJSON_GetObjectItem(cmd_data, "username");
                    const cJSON *password = cJSON_GetObjectItem(cmd_data, "password");
                    const cJSON *mqtt_uri_json = cJSON_GetObjectItem(cmd_data, "mqtt_uri");
                     if (mqtt_uri_json && cJSON_IsString(mqtt_uri_json)) {
                        mqtt_nvs_save_parameters("mqtt_uri", mqtt_uri_json->valuestring);
                    } else {
                        ESP_LOGW(TAG, "Invalid 'mqtt_uri' format in 'cmd_data'");
                    }   
                    if (username && cJSON_IsString(username) && password && cJSON_IsString(password)) {
                        mqtt_nvs_save_parameters("username", username->valuestring);
                        mqtt_nvs_save_parameters("password", password->valuestring);
                    } else {
                        ESP_LOGW(TAG, "Invalid 'cmd_data' format");
                    }
                } else {
                    ESP_LOGW(TAG, "No 'cmd_data' field in JSON or it's not an object");
                }
        }   
    } else {
        ESP_LOGW(TAG, "No 'cmd' field in JSON or it's not a string");
    }
    cJSON_Delete(json);

}   

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char *topic_publish_buffer = NULL;
    char *json_buffer = NULL;
    static char *accumulated_message = NULL;
    const esp_app_desc_t *app_desc = esp_app_get_description();
    const esp_netif_ip_info_t* ip_info = get_ip_info();

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        MALLOC_SPRINTF(topic_publish_buffer, "%s/%s/%s/status", default_base_topic, SYSTOMQTT, mac_str);
        MALLOC_SPRINTF(json_buffer, "{\"project\": \"%s\", \"version\": \"%s\", \"idf_version\": \"%s\", \"mac_address\": \"%s\",\"date\": \"%s\",\"time\": \"%s\", \"ipaddr\": \"%d.%d.%d.%d\" }",
                 app_desc->project_name, app_desc->version, app_desc->idf_ver, mac_str, app_desc->date, app_desc->time, IP2STR(&ip_info->ip));


        msg_id = esp_mqtt_client_publish(client, topic_publish_buffer, json_buffer, 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        free(json_buffer);
        free(topic_publish_buffer);

        MALLOC_SPRINTF(topic_publish_buffer, "%s/%s/%s/cmd", default_base_topic, MQTTTOSYS, mac_str);
        msg_id = esp_mqtt_client_subscribe(client, topic_publish_buffer, 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        free(topic_publish_buffer);
        
        /*
        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        */
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d, return code=0x%02x ", event->msg_id, (uint8_t)*event->data);
        
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");

        ESP_LOGI(TAG, "Received MQTT message on topic: %.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "Message total length: %d", event->total_data_len);
        ESP_LOGI(TAG, "Message length: %d", event->data_len);
        ESP_LOGI(TAG, "Message data: %.*s", event->data_len, event->data);
        if (event->data_len == event->total_data_len) {
            ESP_LOGI(TAG, "Complete message received in one event");
            //free accumulated message if it exists
            if (accumulated_message) {
                free(accumulated_message);
                accumulated_message = NULL;
            }
            handle_event_data(event);
        } else {
            ESP_LOGI(TAG, "Message received in multiple parts, waiting for more data...");
            // You may want to implement a buffer to accumulate the message parts until the complete message is received
            static esp_mqtt_event_t complete_event; // Copy the original event

            if (event->current_data_offset == 0) {
                 // Start of a new message, allocate buffer for the complete message
                 if (accumulated_message) {
                    free(accumulated_message);
                    accumulated_message = NULL;
                }
                accumulated_message = malloc(event->total_data_len + 1); // +1 for null terminator
                if (complete_event.topic) {
                    free(complete_event.topic);
                    complete_event.topic = NULL;
                }
                complete_event = *event; // Copy the original event
                //the pointer to the topic in the original event may not be valid after this function returns, so we need to copy it to a new buffer
                complete_event.topic = malloc(event->topic_len);
                if (complete_event.topic) {
                    memcpy(complete_event.topic, event->topic, event->topic_len);
                    complete_event.topic_len = event->topic_len;
                } else {
                    ESP_LOGW(TAG, "Failed to allocate memory for complete message topic");
                }
            }
            
            if (accumulated_message != NULL) {
                memcpy(accumulated_message + event->current_data_offset, event->data, event->data_len);
                if (event->current_data_offset + event->data_len >= event->total_data_len) {
                    accumulated_message[event->total_data_len] = '\0'; // Null-terminate the complete message
                    ESP_LOGI(TAG, "Complete message received: %s", accumulated_message);
                    // Create a temporary event structure to pass the complete message to the handler
                    complete_event.data = accumulated_message; // Point to the complete message
                    complete_event.data_len = event->total_data_len; // Set the length to the total data length
                    handle_event_data(&complete_event); // Call the handler with the complete message
                    free(accumulated_message); // Free the accumulated message buffer
                    accumulated_message = NULL; // Reset for next message
                    free(complete_event.topic); // Free the topic buffer
                    complete_event.topic = NULL; // Reset topic pointer
                    complete_event.data = NULL; // Reset data pointer 
                    complete_event.data_len = 0; // Reset data length
                }
            }
        }   
        

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = mqtt_uri ? mqtt_uri : CONFIG_MQTT_PROXY_BROKER_URI,
            .verification.certificate = mqtt_cert_pem ? (const char *)mqtt_cert_pem : (const char *)default_iot_inventech_pem_start
        },
        .credentials  = {
                .username = mqtt_username ? mqtt_username : CONFIG_MQTT_PROXY_BROKER_USERNAME,
                .authentication.password = mqtt_password ? mqtt_password : CONFIG_MQTT_PROXY_BROKER_PASSWORD
        }
    };

    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}



void mqtt_nvs_save_parameters(const char *parameter, const char *value) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("mqtt", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        esp_err_t set_err = nvs_set_str(my_handle, parameter, value);
        if (set_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set NVS parameter '%s': %s", parameter, esp_err_to_name(set_err));
        } else {
            ESP_LOGI(TAG, "MQTT parameter '%s' saved to NVS", parameter);
        }        nvs_commit(my_handle);
        nvs_close(my_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS handle for writing");
    }
}


void mqtt_nvs_load_parameters() {
    // This function can be implemented to load MQTT parameters from NVS if needed
    // For example, you could load the broker URI, username, and password from NVS instead of using compile-time configuration
    // Make sure to handle memory allocation and deallocation properly when loading strings from NVS
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("mqtt", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        size_t required_size;
        esp_err_t err;
        err = nvs_get_str(my_handle, "broker_uri", NULL, &required_size);
        if (err == ESP_OK) {
            // first erase memory if already allocated
            if (mqtt_uri) {
                free(mqtt_uri);
                mqtt_uri = NULL;
            }
            mqtt_uri = malloc(required_size);
            nvs_get_str(my_handle, "broker_uri", mqtt_uri, &required_size);
            ESP_LOGI(TAG, "Loaded broker URI from NVS: %s", mqtt_uri);
        } else {
            //allocate default value if not found in NVS
            if (mqtt_uri) {
                free(mqtt_uri);
                mqtt_uri = NULL;
            }
        }

        err = nvs_get_str(my_handle, "username", NULL, &required_size);
        if (err == ESP_OK) {
            if (mqtt_username) {
                free(mqtt_username);
                mqtt_username = NULL;
            }
            mqtt_username = malloc(required_size);
            nvs_get_str(my_handle, "username", mqtt_username, &required_size);
            ESP_LOGI(TAG, "Loaded username from NVS: %s", mqtt_username);
            
        } else {
            if (mqtt_username) {
                free(mqtt_username);
                mqtt_username = NULL;
            }
            ESP_LOGI(TAG, "Using default username");
        }
        
        err = nvs_get_str(my_handle, "password", NULL, &required_size);
        if (err == ESP_OK) {
            if (mqtt_password) {
                free(mqtt_password);
                mqtt_password = NULL;
            }
            mqtt_password = malloc(required_size);
            nvs_get_str(my_handle, "password", mqtt_password, &required_size);
            ESP_LOGI(TAG, "Loaded password from NVS: %s", mqtt_password);
        } else {
            if (mqtt_password) {
                free(mqtt_password);
                mqtt_password = NULL;
            }
            ESP_LOGI(TAG, "Using default password");
        }

        err = nvs_get_str(my_handle, "cert_pem", NULL, &required_size);
        //err = -1;
        if (err == ESP_OK) {
            if (mqtt_cert_pem) {
                free(mqtt_cert_pem);
                mqtt_cert_pem = NULL;
            }
            mqtt_cert_pem = malloc(required_size+1);
            nvs_get_str(my_handle, "cert_pem", (char *)mqtt_cert_pem, &required_size);
            mqtt_cert_pem[required_size] = '\0'; // Null-terminate the string
            ESP_LOGI(TAG, "Loaded certificate from NVS");
        } else {
            if (mqtt_cert_pem) {
                free(mqtt_cert_pem);
                mqtt_cert_pem = NULL;
            }
            ESP_LOGI(TAG, "Using default certificate");}
        nvs_close(my_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS handle %d", err);
    }
}

char* mqtt_nvs_get_parameter(const char *parameter) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("mqtt", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        return NULL;
    }
    
    size_t required_size;
    err = nvs_get_str(my_handle, parameter, NULL, &required_size);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return NULL;
    }
    
    char *value = malloc(required_size);
    if (value != NULL) {
        nvs_get_str(my_handle, parameter, value, &required_size);
    }
    nvs_close(my_handle);
    return value;
}

void mqtt_main(void)
{
    //Requires that wifi is already provisioned and connected. 

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_INFO);
    esp_log_level_set("mqtt_example", ESP_LOG_INFO);
    esp_log_level_set("transport_base", ESP_LOG_INFO);
    esp_log_level_set("transport", ESP_LOG_INFO);
    esp_log_level_set("outbox", ESP_LOG_INFO);

    get_mac_address();
    mqtt_nvs_load_parameters();


    mqtt_app_start();
}
