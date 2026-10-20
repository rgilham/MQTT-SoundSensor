#ifndef MQTT_SENSOR_CLIENT_H
#define MQTT_SENSOR_CLIENT_H

void mqtt_main(void);
void publish_sensor_string(const char* sensor_mac, const char* sensor_json_value);
void publish_gateway_status(const char *devices_json);


#endif /* MQTT_SENSOR_CLIENT_H */
