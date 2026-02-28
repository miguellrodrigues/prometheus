//
// Created by miguel on 14/08/24.
//

#include <memory.h>
#include <stdlib.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

static const uint8_t MISO_PIN = GPIO_NUM_5;
static const uint8_t MOSI_PIN = GPIO_NUM_6;
static const uint8_t SCLK_PIN = GPIO_NUM_7;

typedef struct SPI_Device *SPID_t;

void spiRead(SPID_t device, uint8_t reg, uint8_t *data, uint8_t size);

void spiWrite(SPID_t device, const uint8_t *data, uint8_t size);

SPID_t initSpiDevice(spi_host_device_t host, uint8_t chip_select, uint8_t flags);
