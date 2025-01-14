//
// Created by miguel on 11/01/25.
//

#include <stdlib.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

typedef struct OW_Device *OWD_t;

void ow_read(OWD_t device, uint8_t *data, uint8_t size);
void ow_write(OWD_t device, uint8_t *data, uint8_t size);
OWD_t init_ow_device(uint8_t pin);

uint8_t get_ow_pin(OWD_t device);

esp_err_t ow_bus_init(uint8_t pin);
esp_err_t ow_bus_read(uint8_t pin, uint8_t *data, uint8_t size);
esp_err_t ow_bus_write(uint8_t pin, const uint8_t *data, uint8_t size);
esp_err_t ow_bus_reset(uint8_t pin);
