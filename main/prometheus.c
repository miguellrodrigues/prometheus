/* Includes */

#include <file_tools.h>
#include <i2c_device.h>
#include <keypad.h>
#include <lcd.h>
#include <ow_device.h>
#include <spi_device.h>
#include <driver/gpio.h>
#include <esp_spiffs.h>

/* Definitions */

#define TAG "PROMETHEUS"

#define I2C_SCL_IO GPIO_NUM_4
#define I2C_SDA_IO GPIO_NUM_5

#define SPI_MOSI_IO GPIO_NUM_23
#define SPI_MISO_IO GPIO_NUM_19
#define SPI_SCLK_IO GPIO_NUM_18

typedef enum { LCD, KEYPAD } i2c_device_type_t;
typedef enum { DS18B20 } one_wire_device_type_t;

I2CD_t *i2c_devices;
OWD_t *one_wire_devices;

/* Function Prototypes */

I2CD_t *setup_i2c(uint8_t *addresses, uint8_t num_devices);
SPID_t *setup_spi(uint8_t *cs, uint8_t num_devices);
OWD_t *setup_one_wire(uint8_t *pins, uint8_t num_devices);

void setup_spiffs();

/* Function Implementations */

I2CD_t *setup_i2c(uint8_t *addresses, uint8_t num_devices) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_SCL_IO,
        .sda_io_num = I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags = {
                .enable_internal_pullup = true
        }
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C bus created");

    if (addresses != NULL) {
        I2CD_t *devices = calloc(num_devices, sizeof(I2CD_t));

        for (uint8_t i = 0; i < num_devices; i++) {
            *(devices + i) = init_device(addresses[i], bus_handle);
            ESP_LOGI(TAG, "I2C device 0x%02X initialized", addresses[i]);
        }

        return devices;
    }

    return NULL;
}
SPID_t *setup_spi(uint8_t *cs, uint8_t num_devices) {
    spi_bus_config_t bus_config = {
        .mosi_io_num = SPI_MOSI_IO,
        .miso_io_num = SPI_MISO_IO,
        .sclk_io_num = SPI_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    uint8_t host = SPI3_HOST;

    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, 0));
    ESP_LOGI(TAG, "SPI bus created");

    if (cs != NULL) {
        SPID_t *devices = calloc(num_devices, sizeof(SPID_t));

        for (uint8_t i = 0; i < num_devices; i++) {
            *(devices + i) = init_spi_device(host, cs[i], 0);
            ESP_LOGI(TAG, "SPI device 0x%02X initialized", cs[i]);
        }

        return devices;
    }

    return NULL;
}
OWD_t *setup_one_wire(uint8_t *pins, uint8_t num_devices) {
    if (pins != NULL) {
        OWD_t *devices = calloc(num_devices, sizeof(OWD_t));

        for (uint8_t i = 0; i < num_devices; i++) {
            *(devices + i) = init_ow_device(pins[i]);
            ESP_LOGI(TAG, "OneWire device initialized on pin %d", pins[i]);
        }

        return devices;
    }

    return NULL;
}

void setup_spiffs() {
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&config));
}

void app_main(void)
{

}
