#include <file_tools.h>
#include <i2c_device.h>
#include <ow_device.h>
#include <spi_device.h>
#include <entityx.h>

#include <esp_spiffs.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <freertos/queue.h>
#include <mqtt_client.h>
#include <nvs_flash.h>

/* Definitions */

#define TAG "BDC_CONTROLLER"

/* I2C Pins */
#define I2C_SCL_IO GPIO_NUM_4
#define I2C_SDA_IO GPIO_NUM_5

/* Sampling Period in MS */
#define SAMPLING_INTERVAL_MS 7500

/* SPI */
#define SPI_MOSI_IO 0
#define SPI_MISO_IO 0
#define SPI_SCLK_IO 0 // spi pins are not defined

/* Temperature Sensor Resolution in Bits */
#define DS18B20_RESOLUTION 10

/* Controller Order */
#define CONTROLLER_ORDER 2

/* Devices / Addresses / Pins */
typedef enum {
    LCD, KEYPAD
} i2c_device_id_t;

typedef enum {
    DS18B20
} one_wire_device_id_t;

uint8_t one_wire_devices_pins[1] = {GPIO_NUM_48};
uint8_t i2c_devices_address[2] = {0x00, 0x01};

I2CD_t *i2c_devices = NULL;
OWD_t *one_wire_devices = NULL;

/* WIFI */
#define ST_WIFI_SSID "RODRIGUES 2.4"
#define ST_WIFI_PASS "@16@17@30"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define MAXIMUM_RETRY  5

#define AP_WIFI_SSID "BDC_AP"
#define AP_WIFI_PASS "41639549"

int s_retry_num = 0;
EventGroupHandle_t s_wifi_event_group;

typedef enum {
    WIFI_MODE_AP_ONLY, WIFI_MODE_STA_ONLY
} wifi_operation_mode_t;

/* MQTT */

#define MQTT_USERNAME "ESP32"
#define MQTT_TOPIC "/streaming/data"
#define MQTT_URI "mqtt://192.168.4.2:1883"
#define MQTT_PACKET_SIZE 16

bool mqtt_connected = false;

/* Sampling Variables & Queues */

typedef struct {
    int64_t timestamp;
    float temperature;
} sampling_event_t;

typedef struct {
    float temperature;
    float control_signal;
    int64_t timestamp;
} control_event_t;

QueueHandle_t mqtt_queue;
QueueHandle_t temperature_queue;

/* Communications Protocol Setup */

I2CD_t *setup_i2c(uint8_t *addresses, uint8_t num_devices) {
    i2c_master_bus_config_t bus_config = {.clk_source = I2C_CLK_SRC_DEFAULT, .i2c_port = -1, .scl_io_num = I2C_SCL_IO, .sda_io_num = I2C_SDA_IO, .glitch_ignore_cnt = 7, .flags = {.enable_internal_pullup = true}};

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
    spi_bus_config_t bus_config = {.mosi_io_num = SPI_MOSI_IO, .miso_io_num = SPI_MISO_IO, .sclk_io_num = SPI_SCLK_IO, .quadwp_io_num = -1, .quadhd_io_num = -1,};

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

        ESP_LOGI(TAG, "One Wire devices initialized");

        return devices;
    }

    return NULL;
}

/* Spiffs */

void setup_spiffs() {
    esp_vfs_spiffs_conf_t config = {.base_path = "/spiffs", .partition_label = NULL, .max_files = 5, .format_if_mount_failed = true};

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&config));
    ESP_LOGI(TAG, "SPIFFS mounted");
}

/* DS18B20 Functions */

float ds18b20_read_temperature(OWD_t device) {
    uint8_t data[2] = {0};

    uint8_t pin = get_ow_pin(device);

    ow_bus_reset(pin);
    ow_bus_write(pin, (uint8_t[]) {0xCC, 0x44}, 2);

    vTaskDelay(200 / portTICK_PERIOD_MS); // 200ms max for 10 bits

    ow_bus_reset(pin);
    ow_bus_write(pin, (uint8_t[]) {0xCC, 0xBE}, 2);
    ow_bus_read(pin, data, 2);

    return (float) ((data[1] << 8 | data[0]) / 16.0);
}

void ds18b20_set_resolution(OWD_t device, uint8_t resolution) {
    // resolution can be 9, 10, 11, or 12 bits

    // 9 bits: 0.5°C
    // 10 bits: 0.25°C
    // 11 bits: 0.125°C
    // 12 bits: 0.0625°C


    static uint8_t RES_9_BIT = 0x1F, RES_10_BIT = 0x3F, RES_11_BIT = 0x5F, RES_12_BIT = 0x7F;

    uint8_t res = RES_9_BIT;
    uint8_t pin = get_ow_pin(device);

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
        default:
            break;
    }

    ow_bus_reset(pin);
    ow_bus_write(pin, (uint8_t[]) {0xCC, 0x4E, res}, 2);

    ow_bus_reset(pin);
    ow_bus_write(pin, (uint8_t[]) {0xCC, 0x48}, 2);

    vTaskDelay(20 / portTICK_PERIOD_MS);
}

/* Closed Loop Functions */

void compute_control_signal(float y, float *y_buffer, float *u_buffer, const float *controller_num, const float *controller_den, uint8_t n) {
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
    int64_t last_timestamp;
    float y_buffer[CONTROLLER_ORDER], u_buffer[CONTROLLER_ORDER];

    float controller_num[CONTROLLER_ORDER + 1];
    float controller_den[CONTROLLER_ORDER + 1];

    while (1) {
        sampling_event_t event;

        if (xQueueReceive(temperature_queue, &event, portMAX_DELAY)) {
            float temp = event.temperature;
            int64_t timestamp = event.timestamp;

            if (mqtt_connected) {
                control_event_t controlEvent = {.temperature = temp, .timestamp = timestamp, .control_signal = 0};
                xQueueSend(mqtt_queue, &controlEvent, 0);
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/* Sampling */

void sample_temperature(void *arg) {
    OWD_t device = one_wire_devices[DS18B20];

    float temp = ds18b20_read_temperature(device);

    sampling_event_t samplingEvent = {.temperature = temp, .timestamp = esp_timer_get_time()};

    xQueueSend(temperature_queue, &samplingEvent, 0);
}

void setup_sampling_timer() {
    esp_timer_handle_t timerHandle;

    const esp_timer_create_args_t timerArgs = {.callback = &sample_temperature, .name = "temperature_sampling",};

    ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &timerHandle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timerHandle, SAMPLING_INTERVAL_MS * 1000));
}

/* WIFI */

void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Error connecting to the AP, retrying...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connection Failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void setup_wifi(wifi_operation_mode_t mode) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
                                                        &instance_got_ip));

    if (mode == WIFI_MODE_AP_ONLY) {
        esp_netif_create_default_wifi_ap();

        wifi_config_t ap_config = {.ap = {.ssid = AP_WIFI_SSID, .ssid_len = strlen(
                AP_WIFI_SSID), .password = AP_WIFI_PASS, .channel = 1, .authmode = WIFI_AUTH_WPA2_PSK, .max_connection = 2,},};

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "Starting AP");
        ESP_LOGI(TAG, "AP SSID: %s, Password: %s", ap_config.ap.ssid, ap_config.ap.password);
    } else if (mode == WIFI_MODE_STA_ONLY) {
        esp_netif_create_default_wifi_sta();

        wifi_config_t sta_config = {.sta = {.ssid = ST_WIFI_SSID, .password = ST_WIFI_PASS, .threshold.authmode = WIFI_AUTH_WPA2_PSK,},};

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "wifi_init_sta finished.");

        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
                                               portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connecting to ap SSID:%s password:%s", ST_WIFI_SSID, ST_WIFI_PASS);
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Error connecting to SSID:%s, password:%s", ST_WIFI_SSID, ST_WIFI_PASS);
        } else {
            ESP_LOGE(TAG, "Unexpected event");
        }
    }
}

/* MQTT */

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data) {
    esp_mqtt_event_handle_t event = data;

    switch (id) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(event->client, MQTT_TOPIC, 0);
            mqtt_connected = true;

            ESP_LOGI(TAG, "Connected to MQTT broker");
            break;
        default:
            break;
    }
}

_Noreturn void mqtt_task(void *arg) {
    esp_mqtt_client_config_t mqttConfig = {.broker.address.uri = MQTT_URI, .credentials = {.username = MQTT_USERNAME, .client_id = TAG,}};

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, MQTT_EVENT_CONNECTED, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    char buff[MQTT_PACKET_SIZE] = {0};

    while (1) {
        control_event_t event;

        if (xQueueReceive(mqtt_queue, &event, portMAX_DELAY)) {
            memcpy(buff, &event, sizeof(control_event_t));
            esp_mqtt_client_publish(client, MQTT_TOPIC, buff, MQTT_PACKET_SIZE, 1, 1);
            memset(buff, 0, MQTT_PACKET_SIZE);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/* Main */

_Noreturn void app_main(void) {
    /* Init Devices */
    one_wire_devices = setup_one_wire(one_wire_devices_pins, 1);
    i2c_devices = setup_i2c(i2c_devices_address, 2);

    ds18b20_set_resolution(one_wire_devices[DS18B20], DS18B20_RESOLUTION);

    /* WIFI */
    setup_wifi(WIFI_MODE_AP_ONLY);

    /* Sampling & MQTT */
    temperature_queue = xQueueCreate(10, sizeof(sampling_event_t));
    mqtt_queue = xQueueCreate(10, sizeof(control_event_t));

    setup_sampling_timer();

    xTaskCreatePinnedToCore(closed_loop_task, "closed_loop_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL, 1);

    /* Spiffs */
    setup_spiffs();

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}