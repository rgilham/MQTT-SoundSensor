idf_component_register(SRCS 
                    "mqtt_client.c" 
                    "main.c" 
                    "provisioning_main.c" 
                    "mqtt_client.c"
                    "ota_update.c"
                    "board.c"
                    "i2c_bus.c"
                    "spl_meter.c"
                    PRIV_REQUIRES esp_wifi wifi_provisioning mqtt
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash bt esp_partition esp_netif esp_app_format esp_timer json esp_driver_gpio app_update esp_http_client driver esp_driver_i2s) 
