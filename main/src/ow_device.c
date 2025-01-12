//
// Created by miguel on 11/01/25.
//

#include <ow_device.h>

#define TAG "OW_DEVICE"
#define OW_DELAY_US 6

static void ow_write_bit(uint8_t pin, uint8_t bit) {
    gpio_set_level(pin, 0);
    esp_rom_delay_us(bit ? OW_DELAY_US : 60);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(bit ? 60 : OW_DELAY_US);
}

static uint8_t ow_read_bit(uint8_t pin) {
    gpio_set_level(pin, 0);
    esp_rom_delay_us(6);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(9);
    uint8_t bit = gpio_get_level(pin);
    esp_rom_delay_us(55);
    return bit;
}

esp_err_t ow_bus_init(uint8_t pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&io_conf);
}

esp_err_t ow_bus_read(uint8_t pin, uint8_t *data, uint8_t size) {
    if (data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t i = 0; i < size; i++) {
        data[i] = 0;
        for (uint8_t j = 0; j < 8; j++) {
            data[i] |= (ow_read_bit(pin) << j);
        }
    }
    return ESP_OK;
}

esp_err_t ow_bus_write(uint8_t pin, const uint8_t *data, uint8_t size) {
    if (data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t i = 0; i < size; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            ow_write_bit(pin, (data[i] >> j) & 0x01);
        }
    }
    return ESP_OK;
}

esp_err_t ow_bus_reset(uint8_t pin) {
    gpio_set_level(pin, 0);
    esp_rom_delay_us(480);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(70);
    uint8_t presence = gpio_get_level(pin);
    esp_rom_delay_us(410);
    return presence == 0 ? ESP_OK : ESP_FAIL;
}

struct OW_Device {
    uint8_t pin;
    uint8_t address;
};

OWD_t init_ow_device(uint8_t pin) {
    OWD_t device = calloc(1, sizeof(struct OW_Device));

    if (device == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for OW device");
        return NULL;
    }

    device->pin = pin;

    if (ow_bus_init(pin) != ESP_OK) {
        free(device);
        ESP_LOGE(TAG, "Failed to initialize OW bus on pin %d", pin);
        return NULL;
    }

    ESP_LOGI(TAG, "OW Device initialized on pin %d", pin);
    return device;
}

void ow_read(OWD_t device, uint8_t *data, uint8_t size) {
    if (size == 0 || data == NULL || device == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for OW read");
        return;
    }

    if (ow_bus_read(device->pin, data, size) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from OW device on pin %d", device->pin);
    }
}

void ow_write(OWD_t device, uint8_t *data, uint8_t size) {
    if (size == 0 || data == NULL || device == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for OW write");
        return;
    }

    if (ow_bus_write(device->pin, data, size) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to OW device on pin %d", device->pin);
    }
}

uint8_t get_ow_address(OWD_t device) {
    return device->address;
}