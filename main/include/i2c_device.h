//
// Created by miguel on 09/01/25.
//

#include <stdlib.h>
#include <driver/i2c_master.h>
#include <esp_log.h>

typedef struct I2C_Device *I2CD_t;

void i2CRead(I2CD_t device, uint8_t *data, uint8_t size);

void i2CWrite(I2CD_t device, uint8_t *data, uint8_t size);

void i2CWriteReceive(I2CD_t device, uint8_t *data, uint8_t size, uint8_t *receive, uint8_t receive_size);

I2CD_t initI2CDevice(uint8_t address, i2c_master_bus_handle_t masterBus);

uint8_t getI2CDeviceAddress(I2CD_t device);
