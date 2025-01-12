#include <spi_device.h>

#define TAG "SPI_DEVICE"

struct SPI_Device {
    spi_device_handle_t spi_device;
    spi_host_device_t   host;
    uint8_t             chip_select;
    uint8_t             flags;
};

/**
 * @brief Initialize an SPI device
 * @param host The spi host (2 or 3)
 * @param chip_select Device CS
 * @param flags SPI Device flags (half-duplex, etc)
 * @return The SPI Device
 */
SPID_t init_spi_device(spi_host_device_t host, uint8_t chip_select, uint8_t flags) {
    SPID_t device = calloc(1, sizeof(struct SPI_Device));

    if (device == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for device");
        return NULL;
    }

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 5 * 1000 * 1000, // 5 MHz
        .mode           = 0,
        .spics_io_num   = chip_select,
        .queue_size     = 7,
        .flags          = flags
    };

    spi_device_handle_t deviceHandle;

    esp_err_t addDeviceErr = spi_bus_add_device(host, &dev_config, &deviceHandle);
    if (addDeviceErr != ESP_OK) {
        ESP_LOGE(TAG, "Error installing device on bus, %d", chip_select);
        free(device);
        return NULL;
    }

    device->spi_device  = deviceHandle;
    device->host        = host;
    device->chip_select = chip_select;
    device->flags       = flags;

    ESP_LOGI(TAG, "SPI Device %d initialized", chip_select);

    return device;
}

/**
 * @brief Read an register of the spi device
 * @param device The spi device to be read
 * @param reg The register to read (all bits)
 * @param data The buffer to receive the data
 * @param size The size of the buffer to receive data
 */
void spi_read(SPID_t device, uint8_t reg, uint8_t *data, uint8_t size) {
    if (device == NULL) {
        ESP_LOGE(TAG, "Device cannot be null!");
        return;
    }

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

    // verify device flags
    if (device->flags & SPI_DEVICE_HALFDUPLEX) {
        // Transmission only
        transaction.flags        = SPI_TRANS_USE_TXDATA;
        transaction.length       = 8;
        transaction.tx_data[0]   = reg;
        transaction.rx_buffer    = NULL;

        ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_transmit(device->spi_device, &transaction));

        memset(&transaction, 0, sizeof(transaction));

        // Reception only
        transaction.flags      = SPI_TRANS_USE_RXDATA;
        transaction.rxlength   = 8 * size;

        ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_transmit(device->spi_device, &transaction));

        memcpy(data, transaction.rx_data, size);
    } else {
        // Full Duplex Transaction

        transaction.flags        = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        transaction.tx_data[0]   = reg;
        transaction.length       = 8*(size+1); // 8 bits per byte (size + 1 byte for the register)

        // Perform the SPI transaction in full-duplex mode
        ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_transmit(device->spi_device, &transaction));

        // Copy the received data to the buffer
        memcpy(data, transaction.rx_data + 1, size);
    }
}

/**
 * @brief Write data to the spi device
 * @param device The spi device to write
 * @param data The data to write (data[0] must be the register)
 * @param size Size of the data to write
 */
void spi_write(SPID_t device, const uint8_t *data, uint8_t size) {
    if (device == NULL) {
        ESP_LOGE(TAG, "Device cannot be null!");
        return;
    }

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

    transaction.length    = 8 * size;
    transaction.tx_buffer = data;

    esp_err_t err = spi_device_transmit(device->spi_device, &transaction);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing data to device %d", device->chip_select);
    }
}
