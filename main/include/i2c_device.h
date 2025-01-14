//
// Created by miguel on 09/01/25.
//

#include <stdlib.h>
#include <driver/i2c_master.h>
#include <esp_log.h>

typedef struct I2C_Device *I2CD_t;

void i2c_read(I2CD_t device, uint8_t *data, uint8_t size);

void i2c_write(I2CD_t device, uint8_t *data, uint8_t size);

void i2c_write_receive(I2CD_t device, uint8_t *data, uint8_t size, uint8_t *receive, uint8_t receive_size);

I2CD_t init_i2c_device(uint8_t address, i2c_master_bus_handle_t master_bus);

uint8_t get_i2c_device_address(I2CD_t device);
