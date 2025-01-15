//
// Created by miguel on 14/01/25.
//

#include <mqtt_client.h>

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

void mqtt_task(void *param, void *callback(void *));