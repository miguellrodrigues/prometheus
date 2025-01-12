//
// Created by miguel on 09/01/25.
//

#include <i2c_device.h>

#define TAG "I2C_DEVICE"

struct I2C_Device {
    uint8_t address;
    i2c_master_bus_handle_t master_bus;
    i2c_master_dev_handle_t dev_handle;
};

/**
 * @brief Initialize an I2C device
 * @param address The i2c address of the slave
 * @param master_bus The master_bus previously initialized
 * @return The I2C device
 */
I2CD_t init_device(uint8_t address, i2c_master_bus_handle_t master_bus) {
    if (master_bus == NULL) {
        ESP_LOGE(TAG, "Master bus must not be NULL");
        return NULL;
    }

    I2CD_t device = calloc(1, sizeof(I2CD_t));

    i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = address,
            .scl_speed_hz    = 100000,
    };

    i2c_master_dev_handle_t handle;
    esp_err_t add_device2bus_err = i2c_master_bus_add_device(master_bus, &config, &handle);

    if (add_device2bus_err != ESP_OK) {
        free(device);

        ESP_LOGE(TAG, "Failed to add device to I2C bus");
        return NULL;
    }

    device->master_bus = master_bus;
    device->address    = address;
    device->dev_handle = handle;

    ESP_LOGV(TAG, "I2C Device %d initialized", address);

    return device;
}

/**
 * @brief Deinitialize an I2C device
 * @param device The i2c device
 * @param data Buffer to store the read data
 * @param size Size of the read data
 */
void i2c_read(I2CD_t device, uint8_t *data, uint8_t size) {
    if (size == 0) {
        ESP_LOGE(TAG, "Size must be greater than 0");
        return;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Data must not be NULL");
        return;
    }

    if (device == NULL) {
        ESP_LOGE(TAG, "Device must not be NULL");
        return;
    }

    esp_err_t i2c_read_err = (i2c_master_receive(device->dev_handle, data, size, -1));

    if (i2c_read_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from I2C device, %d", device->address);
    }
}

/**
 * @brief Write to an I2C device
 * @param device The i2c device
 * @param data Data to be written to the device
 * @param size Size of the data
 */
void i2c_write(I2CD_t device, uint8_t *data, uint8_t size) {
    if (size == 0) {
        ESP_LOGE(TAG, "Size must be greater than 0");
        return;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Data must not be NULL");
        return;
    }

    if (device == NULL) {
        ESP_LOGE(TAG, "Device must not be NULL");
        return;
    }

    esp_err_t i2c_write_err = (i2c_master_transmit(device->dev_handle, data, size, -1));

    if (i2c_write_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to I2C device, %d", device->address);
    }
}

/**
 * @brief Write to an I2C device and receive data
 * @param device The i2c device
 * @param data The transmit data
 * @param size Size of transmit data
 * @param receive Buffer to receive data
 * @param receive_size Size of receive buffer
 */
void i2c_write_receive(I2CD_t device, uint8_t *data, uint8_t size, uint8_t *receive, uint8_t receive_size) {
    if (size == 0) {
        ESP_LOGE(TAG, "Size must be greater than 0");
        return;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Data must not be NULL");
        return;
    }

    if (device == NULL) {
        ESP_LOGE(TAG, "Device must not be NULL");
        return;
    }

    if (receive == NULL) {
        ESP_LOGE(TAG, "Receive buffer must not be NULL");
        return;
    }

    if (receive_size == 0) {
        ESP_LOGE(TAG, "Receive size must be greater than 0");
        return;
    }

    esp_err_t i2c_read_write_err = (i2c_master_transmit_receive(device->dev_handle, data, size, receive, receive_size, -1));

    if (i2c_read_write_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read/write to I2C device, %d", device->address);
    }
}

/**
 * @brief Deinitialize an I2C device
 * @param device The i2c device
 * @return
 */
uint8_t get_address(I2CD_t device) {
    return device->address;
}