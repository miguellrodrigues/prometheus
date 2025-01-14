#include <file_tools.h>
#include <i2c_device.h>
#include <keypad.h>
#include <lcd.h>
#include <ow_device.h>
#include <spi_device.h>
#include <driver/gpio.h>
#include <esp_spiffs.h>
#include <esp_timer.h>
#include <freertos/queue.h>

/* Definitions */

#define TAG "PROMETHEUS"

/* I2C Pins */
#define I2C_SCL_IO GPIO_NUM_4
#define I2C_SDA_IO GPIO_NUM_5

/* Sampling Period in MS */
#define SAMPLING_INTERVAL_MS 7500

/**/
#define SPI_MOSI_IO 0
#define SPI_MISO_IO 0
#define SPI_SCLK_IO 0 // spi pins are not defined
/**/

/* Temperature Sensor Resolution in Bits */
#define DS18B20_RESOLUTION 10

/* Controller Order */
#define CONTROLLER_ORDER 2

/* Devices / Addresses / Pins */

typedef enum { LCD, KEYPAD } i2c_device_id_t;
typedef enum { DS18B20 } one_wire_device_id_t;

uint8_t one_wire_devices_pins[1] = {GPIO_NUM_48};
uint8_t i2c_devices_address[2] = {0x00, 0x01};

I2CD_t *i2c_devices = NULL;
OWD_t *one_wire_devices = NULL;

/* Sampling Used Variables */

typedef struct {
    int64_t timestamp;
    float temperature;
} sampling_event_t;

QueueHandle_t temperature_queue;

/* Spiffs & Files */

void setup_spiffs();

/* Communications Protocol Setup */

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
            *(devices + i) = init_i2c_device(addresses[i], bus_handle);
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

/* DS18B20 Functions */

float ds18b20_read_temperature(OWD_t device) {
    uint8_t data[2] = {0};

    uint8_t pin = get_ow_pin(device);

    ow_bus_reset(pin);
    ow_bus_write(pin, (uint8_t[]){0xCC, 0x44}, 2);

    vTaskDelay(750 / portTICK_PERIOD_MS);

    ow_bus_reset(pin);
    ow_bus_write(pin, (uint8_t[]){0xCC, 0xBE}, 2);
    ow_bus_read(pin, data, 2);

    return (float)((data[1] << 8 | data[0]) / 16.0);
}

void ds18b20_set_resolution(OWD_t device, uint8_t resolution) {
    // resolution can be 9, 10, 11, or 12 bits

    uint8_t pin = get_ow_pin(device);

    static uint8_t RES_9_BIT  = 0x1F,
            RES_10_BIT = 0x3F,
            RES_11_BIT = 0x5F,
            RES_12_BIT = 0x7F;

    uint8_t res = RES_9_BIT;

    switch (resolution) {
        case 9:
            res = RES_9_BIT;
            break;
        case 10:
            res = RES_10_BIT;
            break;
        case 11:
            res = RES_11_BIT;
            break;
        case 12:
            res = RES_12_BIT;
            break;
    }

    ow_bus_reset(pin);
    ow_bus_write(pin, (uint8_t[]){0xCC, 0x4E, res}, 2);
    ow_bus_reset(pin);

    ow_bus_write(pin, (uint8_t[]) {0xCC, 0x48}, 2);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

void read_temperature(void *arg) {
    OWD_t device = one_wire_devices[DS18B20];

    float temp = ds18b20_read_temperature(device);

    sampling_event_t samplingEvent = {
            .temperature = temp,
            .timestamp = esp_timer_get_time()
    };

    xQueueSend(temperature_queue, &samplingEvent, 0);
}

/* Closed Loop Functions & Sampling */

void compute_control_signal(
        float y,
        float *y_buffer,
        float *u_buffer,

        const float *controller_num,
        const float *controller_den,

        uint8_t n
) {
    // compute control signal

    float u = controller_num[0] * y;

    for (uint8_t i = 1; i <= n; i++) {
        u += controller_num[i] * y_buffer[i - 1];
        u -= controller_den[i] * u_buffer[i - 1];
    }

    // update buffers
    for (uint8_t i = n - 1; i > 0; i--) {
        y_buffer[i] = y_buffer[i - 1];
        u_buffer[i] = u_buffer[i - 1];
    }

    y_buffer[0] = y;
    u_buffer[0] = u;
}

_Noreturn void closed_loop_task(void *arg) {
    static int64_t last_timestamp;
    static float y_buffer[CONTROLLER_ORDER], u_buffer[CONTROLLER_ORDER];

    static float controller_num[CONTROLLER_ORDER + 1] = {0};
    static float controller_den[CONTROLLER_ORDER + 1] = {0};

    while (1) {
        sampling_event_t event;

        if (xQueueReceive(temperature_queue, &event, portMAX_DELAY)) {
            float temp = event.temperature;
            int64_t timestamp = event.timestamp;

            ESP_LOGI(TAG, "Delta time: %lld", timestamp - last_timestamp);
            last_timestamp = timestamp;

            ESP_LOGI(TAG, "Temperature: %.2f", temp);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup_sampling_timer() {
    esp_timer_handle_t timerHandle;

    const esp_timer_create_args_t timerArgs = {
            .callback = &read_temperature,
            .name = "temperature_sampling",
    };

    ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &timerHandle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timerHandle, SAMPLING_INTERVAL_MS * 1000));
}

/* Main */

_Noreturn void app_main(void)
{
    one_wire_devices = setup_one_wire(one_wire_devices_pins, 1);
    i2c_devices = setup_i2c(i2c_devices_address, 2);

    ds18b20_set_resolution(one_wire_devices[DS18B20], DS18B20_RESOLUTION);

    temperature_queue = xQueueCreate(10, sizeof(sampling_event_t));

    setup_sampling_timer();
    xTaskCreatePinnedToCore(closed_loop_task, "closed_loop_task", 4096, NULL, 5, NULL, 1);

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}