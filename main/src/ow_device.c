//
// Created by miguel on 11/01/25.
//

#include <ow_device.h>

#define TAG "OW_DEVICE"

struct OW_Device {
        uint8_t pin;
};

static portMUX_TYPE ow_mux = portMUX_INITIALIZER_UNLOCKED;

// Timing constants in microseconds
#define OW_RESET_TIME     480   // Reset pulse duration
#define OW_PRESENCE_WAIT  70    // Wait time before sampling presence
#define OW_PRESENCE_MAX   240   // Maximum time to wait for presence pulse
#define OW_WRITE_1_LOW    6     // Write 1 low time
#define OW_WRITE_1_HIGH   64    // Write 1 high time
#define OW_WRITE_0_LOW    60    // Write 0 low time
#define OW_WRITE_0_HIGH   10    // Write 0 high time
#define OW_READ_LOW       6     // Read low time
#define OW_READ_WAIT      9     // Wait before sampling
#define OW_READ_TOTAL     70    // Total read timeslot duration
#define OW_RECOVERY       5     // Recovery time between operations

static void ow_write_bit(uint8_t pin, uint8_t bit) {
//    gpio_set_direction(pin, GPIO_MODE_OUTPUT);

    if (bit) {
        // Write 1: 6µs low, 64µs high
        gpio_set_level(pin, 0);
        esp_rom_delay_us(OW_WRITE_1_LOW);
        gpio_set_level(pin, 1);
        esp_rom_delay_us(OW_WRITE_1_HIGH);
    } else {
        // Write 0: 60µs low, 10µs high
        gpio_set_level(pin, 0);
        esp_rom_delay_us(OW_WRITE_0_LOW);
        gpio_set_level(pin, 1);
        esp_rom_delay_us(OW_WRITE_0_HIGH);
    }
    esp_rom_delay_us(OW_RECOVERY);
}

static uint8_t ow_read_bit(uint8_t pin) {
    uint8_t bit;

//    gpio_set_direction(pin, GPIO_MODE_OUTPUT);

    // Drive bus low for 6µs
    gpio_set_level(pin, 0);
    esp_rom_delay_us(OW_READ_LOW);

    // Release bus and wait before sampling
    gpio_set_level(pin, 1);
    esp_rom_delay_us(OW_READ_WAIT);

//    gpio_set_direction(pin, GPIO_MODE_INPUT);

    // Sample the bit
    bit = gpio_get_level(pin);

    // Wait for timeslot to complete
    esp_rom_delay_us(OW_READ_TOTAL - OW_READ_LOW - OW_READ_WAIT);
    esp_rom_delay_us(OW_RECOVERY);

    return bit;
}

esp_err_t owBusReset(uint8_t pin) {
    uint8_t presence = 1;
    uint16_t wait_time = 0;

//    gpio_set_direction(pin, GPIO_MODE_OUTPUT);

    // Send reset pulse
    gpio_set_level(pin, 0);
    esp_rom_delay_us(OW_RESET_TIME);
    gpio_set_level(pin, 1);

    // Wait before checking for presence
    esp_rom_delay_us(OW_PRESENCE_WAIT);

//    gpio_set_direction(pin, GPIO_MODE_INPUT);

    // Wait for presence pulse or timeout
    while (presence && wait_time < OW_PRESENCE_MAX) {
        presence = gpio_get_level(pin);
        esp_rom_delay_us(1);
        wait_time++;
    }

    // Complete the reset sequence timing
    esp_rom_delay_us(OW_RESET_TIME - OW_PRESENCE_WAIT - wait_time);

    if (wait_time >= OW_PRESENCE_MAX) {
//        ESP_LOGW(TAG, "No presence pulse detected");
        return ESP_ERR_TIMEOUT;
    } else {
//        ESP_LOGW(TAG, "Device presence detected after %d µs", wait_time);
        return ESP_OK;
    }
}

esp_err_t owBusInit(uint8_t pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO pin %d", pin);
        return ret;
    }

    // Initialize bus in released (high) state
//    gpio_set_level(pin, 1);
//    esp_rom_delay_us(OW_RECOVERY);

    return ESP_OK;
}

esp_err_t owBusWrite(uint8_t pin, const uint8_t *data, uint8_t size) {
    if (data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&ow_mux);  // Disable interrupts during timing-critical section
    for (uint8_t i = 0; i < size; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            ow_write_bit(pin, (data[i] >> j) & 0x01);
        }
    }
    taskEXIT_CRITICAL(&ow_mux);

    return ESP_OK;
}

esp_err_t owBusRead(uint8_t pin, uint8_t *data, uint8_t size) {
    if (data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&ow_mux);  // Disable interrupts during timing-critical section
    for (uint8_t i = 0; i < size; i++) {
        data[i] = 0;
        for (uint8_t j = 0; j < 8; j++) {
            data[i] |= (ow_read_bit(pin) << j);
        }
    }
    taskEXIT_CRITICAL(&ow_mux);

    return ESP_OK;
}

OWD_t initOwDevice(uint8_t pin) {
    OWD_t device = calloc(1, sizeof(struct OW_Device));
    if (device == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for device");
        return NULL;
    }

    device->pin = pin;

    esp_err_t init_err = owBusInit(pin);
    if (init_err != ESP_OK) {
        free(device);
        return NULL;
    }

    ESP_LOGI(TAG, "OneWire device initialized on pin %d", pin);

    return device;
}

uint8_t getOwPin(OWD_t device) {
    return device->pin;
}