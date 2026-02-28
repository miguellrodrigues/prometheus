//
// Created by miguel on 11/01/25.
//

#include <stdlib.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

typedef struct OW_Device *OWD_t;

void owRead(OWD_t device, uint8_t *data, uint8_t size);
void owWrite(OWD_t device, uint8_t *data, uint8_t size);
OWD_t initOwDevice(uint8_t pin);

uint8_t getOwPin(OWD_t device);

esp_err_t owBusInit(uint8_t pin);
esp_err_t owBusRead(uint8_t pin, uint8_t *data, uint8_t size);
esp_err_t owBusWrite(uint8_t pin, const uint8_t *data, uint8_t size);
esp_err_t owBusReset(uint8_t pin);
