//
// Created by miguel on 11/01/25.
//

#ifndef FUNCTION_GENERATOR_OW_DEVICE_H
#define FUNCTION_GENERATOR_OW_DEVICE_H

#include <stdlib.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

typedef struct OW_Device *OWD_t;

void ow_read(OWD_t device, uint8_t *data, uint8_t size);
void ow_write(OWD_t device, uint8_t *data, uint8_t size);
OWD_t init_ow_device(uint8_t pin);
uint8_t get_ow_address(OWD_t device);

esp_err_t ow_bus_init(uint8_t pin);
esp_err_t ow_bus_read(uint8_t pin, uint8_t *data, uint8_t size);
esp_err_t ow_bus_write(uint8_t pin, const uint8_t *data, uint8_t size);
esp_err_t ow_bus_reset(uint8_t pin);

#endif //FUNCTION_GENERATOR_OW_DEVICE_H
