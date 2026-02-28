#include <file_tools.h>
#include <ow_device.h>
#include <entityx.h>
#include <bdc_config.h>
#include <pid.h>

#include <esp_spiffs.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <mqtt_client.h>
#include <nvs_flash.h>
#include <math.h>
#include <driver/mcpwm_prelude.h>
#include <freertos/queue.h>


#define TAG "BDC_CONTROLLER"
/*#define I2C_SCL_IO GPIO_NUM_4
#define I2C_SDA_IO GPIO_NUM_5
#define SPI_MOSI_IO 0
#define SPI_MISO_IO 0
#define SPI_SCLK_IO 0 // spi pins are not defined*/

/* Temperature Sensor Resolution in Bits */
#define DS18B20_RESOLUTION 10

#define ST_WIFI_SSID "HW"
#define ST_WIFI_PASS "41639549"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define MAXIMUM_RETRY  5

/* WIFI */
#define AP_WIFI_SSID "GREA_HEAT_PUMP"
#define AP_WIFI_PASS "41639549"

/* MQTT */
#define MQTT_USERNAME "ESP32S3_HEATPUMP_CONTROLLER"
#define MQTT_PASSWORD "94593164"
#define TRANSMIT_INFO_TOPIC "/streaming/data"
#define UPDATE_CONFIG_TOPIC "/control/update_config"
#define UPDATE_SETPOINT_TOPIC "/control/update_setpoint"
#define MQTT_PACKET_SIZE sizeof(control_event_t)
char *MQTT_URI = "mqtt://greamqtt.broker:1883";

/* PINS */
#define ACTUATE_GPIO GPIO_NUM_2
#define IN1_GPIO GPIO_NUM_13
#define IN2_GPIO GPIO_NUM_1
#define EN_GPIO GPIO_NUM_38


/* Enums */
typedef enum {
    LCD, KEYPAD __attribute__((unused))
} i2c_device_id_t;

typedef enum {
    DS18B20
} one_wire_device_id_t;

typedef enum {
    CLOSED_LOOP, OPEN_LOOP
} control_state;

typedef enum {
    WIFI_MODE_AP_ONLY, WIFI_MODE_STA_ONLY
} wifi_operation_mode_t;

/* Structs */

typedef struct {
    uint64_t timestamp;
    double temperature;
} sampling_event_t;

typedef struct {
    uint64_t timestamp;
    double temperature;
    double control_signal;
    double set_point;
} control_event_t;

typedef struct {
    double set_point;
} update_setpoint_action_t;

/* Parameters & Holders */
uint16_t samplingIntervalMs; /* Sampling Period in MS */
uint8_t oneWireDevicesPins[1] = {GPIO_NUM_47}; /* One Wire Devices */
//uint8_t i2CDevicesAddress[2] = {}; /* I2C Devices */ // not used

uint8_t sRetryNum = 0; /* WIFI Retry Number */
EventGroupHandle_t sWifiEventGroup; /* WIFI Event Group */

bdc_config_t *bdcConfig; /* BDC CONFIG */

bool mqttConnected = false; /* MQTT Connection Status */
bool hasSwitchedState = false; /* State Switching Status */

/* Devices */
//I2CD_t *i2cDevices = NULL; /* I2C Devices */
OWD_t *oneWireDevices = NULL; /* One Wire Devices */

/* Sampling Variables & Queues & Control */
esp_timer_handle_t samplingTimerHandle = NULL;

QueueHandle_t mqttQueue; /* MQTT Queue */
QueueHandle_t temperatureQueue; /* Temperature Queue */

/* Controller */
uint8_t filterOrder; /* Filter Order */

double *filterNum;
double *filterDen;
double setPoint;
double openLoopControlSignal;
double calibrationAngularTerm;
double calibrationLinerTerm;

PID_t *pid;

control_state controlState = OPEN_LOOP;
mcpwm_cmpr_handle_t pwmComparator;

/* mqtt */
esp_mqtt_client_handle_t mqttClient;

/* Prototypes */

void updateParametersFromConfig(bdc_config_t *config);

/* Communications Protocol Setup */

/*I2CD_t *setupI2C(uint8_t *addresses, uint8_t num_devices) {
    i2c_master_bus_config_t busConfig = {
            .clk_source = I2C_CLK_SRC_DEFAULT, .i2c_port = -1, .scl_io_num = I2C_SCL_IO, .sda_io_num = I2C_SDA_IO,
            .glitch_ignore_cnt = 7, .flags = {.enable_internal_pullup = true}
    };

    i2c_master_bus_handle_t busHandle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&busConfig, &busHandle));
    ESP_LOGI(TAG, "I2C bus created");

    if (addresses != NULL) {
        I2CD_t *devices = calloc(num_devices, sizeof(I2CD_t));

        for (uint8_t i = 0; i < num_devices; i++) {
            *(devices + i) = initI2CDevice(addresses[i], busHandle);
        }

        return devices;
    }

    return NULL;
}

SPID_t *setupSPI(uint8_t *cs, uint8_t num_devices) {
    spi_bus_config_t busConfig = {
            .mosi_io_num = SPI_MOSI_IO, .miso_io_num = SPI_MISO_IO, .sclk_io_num = SPI_SCLK_IO, .quadwp_io_num = -1,
            .quadhd_io_num = -1,
    };

    uint8_t host = SPI3_HOST;

    ESP_ERROR_CHECK(spi_bus_initialize(host, &busConfig, 0));
    ESP_LOGI(TAG, "SPI bus created");

    if (cs != NULL) {
        SPID_t *devices = calloc(num_devices, sizeof(SPID_t));

        for (uint8_t i = 0; i < num_devices; i++) {
            *(devices + i) = initSpiDevice(host, cs[i], 0);
        }

        return devices;
    }

    return NULL;
} not used */

OWD_t *setupOneWire(uint8_t *pins, uint8_t num_devices) {
    if (pins != NULL) {
        OWD_t *devices = calloc(num_devices, sizeof(OWD_t));

        for (uint8_t i = 0; i < num_devices; i++) {
            *(devices + i) = initOwDevice(pins[i]);
        }

        ESP_LOGI(TAG, "One Wire devices initialized");

        return devices;
    }

    return malloc(sizeof(OWD_t));
}

/* Spiffs */

void setupSpiffs() {
    esp_vfs_spiffs_conf_t config = {
            .base_path = "/spiffs", .partition_label = NULL, .max_files = 5, .format_if_mount_failed = true
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&config));
    ESP_LOGI(TAG, "SPIFFS mounted");
}

/* DS18B20 Functions */

double ds18B20ReadTemperature(OWD_t device) {
    uint8_t data[2] = {0};

    uint8_t pin = getOwPin(device);

    owBusReset(pin);
    owBusWrite(pin, (uint8_t[]) {0xCC, 0x44}, 2);

    vTaskDelay(200 / portTICK_PERIOD_MS); // 200ms max for 10 bits

    owBusReset(pin);
    owBusWrite(pin, (uint8_t[]) {0xCC, 0xBE}, 2);
    owBusRead(pin, data, 2);

    return ((data[1] << 8 | data[0]) / 16.0);
}

void ds18B20SetResolution(OWD_t device, uint8_t resolution) {
    if (device == NULL) {
        ESP_LOGE(TAG, "Device is NULL");
        return;
    }

    // resolution can be 9, 10, 11, or 12 bits

    // 9 bits: 0.5°C
    // 10 bits: 0.25°C
    // 11 bits: 0.125°C
    // 12 bits: 0.0625°C

    static uint8_t RES_9_BIT = 0x1F, RES_10_BIT = 0x3F, RES_11_BIT = 0x5F, RES_12_BIT = 0x7F;

    uint8_t res = RES_9_BIT;
    uint8_t pin = getOwPin(device);

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

    owBusReset(pin);
    owBusWrite(pin, (uint8_t[]) {0xCC, 0x4E, res}, 3);

    owBusReset(pin);
    owBusWrite(pin, (uint8_t[]) {0xCC, 0x48}, 2);

    vTaskDelay(20 / portTICK_PERIOD_MS);
}

/* PWM */

void setupPwm() {
    mcpwm_timer_handle_t timerHandle;

    mcpwm_timer_config_t timerConfig = {
            .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, .resolution_hz =
            80 * 1000 * 1000,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP, .period_ticks = (1 << 16) - 1
    };

    // PWM_FREQ = 1 / (65534 * 1 / (80_000_000)) = 80_000_000/65534 = 1220.7 Hz

    ESP_ERROR_CHECK(mcpwm_new_timer(&timerConfig, &timerHandle));

    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t operConfig = {
            .group_id = 0 // Use MCPWM group 0
    };

    ESP_ERROR_CHECK(mcpwm_new_operator(&operConfig, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timerHandle));

    mcpwm_comparator_config_t cmprConfig = {
            .flags.update_cmp_on_tez = true // Update at timer zero event
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmprConfig, &pwmComparator));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(pwmComparator, 1 << 14));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t gen_config = {.gen_gpio_num = ACTUATE_GPIO};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &generator));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                           MCPWM_TIMER_EVENT_EMPTY,
                                                                                           MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                               pwmComparator,
                                                                                               MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timerHandle));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerHandle, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "PWM initialized");
}

/* Closed Loop Functions */

bool isNear(double a, double b, double epsilon) {
    return fabs(a - b) < epsilon;
}

double kahanSum(const double *values1, const double *values2, size_t count) {
    double sum = 0.0;
    double c = 0.0;  // Running compensation for lost low-order bits

    for (size_t i = 0; i < count; i++) {
        double product = values1[i] * values2[i];
        double y = product - c;
        double t = sum + y;
        c = (t - sum) - y;
        sum = t;
    }

    return sum;
}

double inner_product(const double *first1, const double *last1, const double *first2, double init) {
    size_t count = (size_t) (last1 - first1);
    if (count == 0) return init;

    return init + kahanSum(first1, first2, count);
}

void calculateTransferFunction(
        double input,
        double *input_buffer,
        double *output_buffer,
        double *numerator,
        double *denominator,
        size_t order
) {
    double u = 0;

    u += inner_product(numerator + 1, numerator + order + 1, input_buffer, numerator[0] * input);
    u -= inner_product(denominator + 1, denominator + order + 1, output_buffer, 0.0);

    u /= denominator[0];

    if (order) {
        memmove(input_buffer + 1, input_buffer, (order - 1) * sizeof(double));
        memmove(output_buffer + 1, output_buffer, (order - 1) * sizeof(double));
    }

    input_buffer[0] = input;
    output_buffer[0] = u;
}

double computeControlSignal(double y, double *inBuffer, double *outBuffer) {
    // Calculate the filter transfer function
    calculateTransferFunction(setPoint, inBuffer, outBuffer, filterNum, filterDen, filterOrder); // pre filter

    // Calculate the PID control signal
    return updatePID(pid, outBuffer[0], y);
}

double calibrationCurve(double x) {
    return calibrationAngularTerm * x + calibrationLinerTerm;
}

void actuate(double control_signal) {
    double voltage = calibrationCurve(control_signal + openLoopControlSignal);

    uint16_t dutyCycle = (uint16_t) ((clip(voltage, 0, 10.8f) / 10.8f) * ((1 << 16) - 1));
    mcpwm_comparator_set_compare_value(pwmComparator, dutyCycle);
}

_Noreturn void controlLoopTask(void *arg) {
    double filterInbuffer[filterOrder + 1], filterOutbuffer[filterOrder + 1];

    memset(filterInbuffer, 0, sizeof(filterInbuffer));
    memset(filterOutbuffer, 0, sizeof(filterOutbuffer));

    while (1) {
        sampling_event_t event;

        if (xQueueReceive(temperatureQueue, &event, portMAX_DELAY)) {
            double temp = event.temperature;
            uint64_t timestamp = event.timestamp;

            if (!hasSwitchedState && isNear(temp, setPoint, 5.0f)) {
                controlState = CLOSED_LOOP;
                hasSwitchedState = true;

                ESP_LOGW(TAG, "Switching to closed loop control");
            }

            control_event_t controlEvent = {.temperature = temp, .timestamp = timestamp, .set_point = setPoint};

            double controlOutput = 0.0f;

            switch (controlState) {
                case OPEN_LOOP:
                    break;
                case CLOSED_LOOP:
                    controlOutput = computeControlSignal(temp, filterInbuffer, filterOutbuffer);
                    break;
                default:
                    break;
            }

            controlEvent.control_signal = controlOutput;

            actuate(controlOutput);

            if (mqttConnected) {
                xQueueSend(mqttQueue, &controlEvent, 0);
            }
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

/* Sampling */

static TaskHandle_t samplingTaskHandle = NULL;

/* Timer callback: must not block — just wake the sampling task */
void samplingTimerCallback(void *arg) {
    if (samplingTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(samplingTaskHandle, NULL);
    }
}

_Noreturn void samplingTask(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        double temp = ds18B20ReadTemperature(oneWireDevices[DS18B20]);

        sampling_event_t samplingEvent = {.temperature = temp, .timestamp = esp_timer_get_time()};

        xQueueSend(temperatureQueue, &samplingEvent, 0);
    }
}

void setupSamplingTimer() {
    bool isActive = esp_timer_is_active(samplingTimerHandle);

    if (isActive) {
        ESP_ERROR_CHECK(esp_timer_stop(samplingTimerHandle));
        ESP_ERROR_CHECK(esp_timer_delete(samplingTimerHandle));
    }

    const esp_timer_create_args_t timerArgs = {.callback = &samplingTimerCallback, .name = "temperature_sampling",};

    ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &samplingTimerHandle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(samplingTimerHandle, samplingIntervalMs * 1000));
}

/* WIFI */

void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (sRetryNum < MAXIMUM_RETRY) {
            esp_wifi_connect();
            sRetryNum++;
            ESP_LOGI(TAG, "Error connecting to the AP, retrying...");
        } else {
            xEventGroupSetBits(sWifiEventGroup, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connection Failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "IP:" IPSTR, IP2STR(&event->ip_info.ip));
        sRetryNum = 0;
        xEventGroupSetBits(sWifiEventGroup, WIFI_CONNECTED_BIT);
    }
}


static void start_ap_mode() {
    esp_netif_create_default_wifi_ap();

    wifi_config_t apConfig = {
            .ap = {
                    .ssid = AP_WIFI_SSID,
                    .ssid_len = strlen(AP_WIFI_SSID),
                    .password = AP_WIFI_PASS,
                    .channel = 1,
                    .authmode = WIFI_AUTH_WPA2_PSK,
                    .max_connection = 4,
                    .ssid_hidden = 1
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apConfig));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Starting AP");
    ESP_LOGI(TAG, "AP SSID: %s, Password: %s", apConfig.ap.ssid, apConfig.ap.password);
}

static bool start_station_mode() {
    esp_netif_create_default_wifi_sta();

    wifi_config_t staConfig = {
            .sta = {
                    .ssid = ST_WIFI_SSID,
                    .password = ST_WIFI_PASS,
                    .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &staConfig));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(sWifiEventGroup,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID: %s password: %s",
                 ST_WIFI_SSID, ST_WIFI_PASS);
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s, password: %s",
                 ST_WIFI_SSID, ST_WIFI_PASS);
        return false;
    } else {
        ESP_LOGE(TAG, "Unexpected event");
        return false;
    }
}

void setup_wifi(wifi_operation_mode_t mode) {
    sWifiEventGroup = xEventGroupCreate();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instanceAnyId;
    esp_event_handler_instance_t instanceGotIp;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, NULL,
                                                        &instanceAnyId));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, NULL,
                                                        &instanceGotIp));

    if (mode == WIFI_MODE_AP_ONLY) {
        start_ap_mode();
    } else if (mode == WIFI_MODE_STA_ONLY) {
        if (!start_station_mode()) {
            ESP_LOGI(TAG, "Station mode failed, switching to AP mode");

            MQTT_URI = "mqtt://192.168.4.2:1883"; // the second connected device must be the broker

            esp_wifi_stop();
            start_ap_mode();
        }
    }
}

/* MQTT */

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data) {
    esp_mqtt_event_handle_t event = data;

    switch (id) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(event->client, UPDATE_CONFIG_TOPIC, 0);
            esp_mqtt_client_subscribe(event->client, UPDATE_SETPOINT_TOPIC, 0);
            mqttConnected = true;

            ESP_LOGI(TAG, "Connected to MQTT broker");
            break;
        case MQTT_EVENT_DATA:
            if (strcmp(event->topic, UPDATE_CONFIG_TOPIC) == 0) {
                bdc_config_t updateConfigAction;
                memcpy(&updateConfigAction, event->data, sizeof(updateConfigAction));

                bdcConfigSetSetPoint(bdcConfig, updateConfigAction.setPoint);
                bdcConfigSetOpenLoopControlSignal(bdcConfig, updateConfigAction.openLoopControlSignal);
                bdcConfigSetCalibrationAngularTerm(bdcConfig, updateConfigAction.calibrationAngularTerm);
                bdcConfigSetCalibrationLinearTerm(bdcConfig, updateConfigAction.calibrationLinearTerm);
                bdcConfigSetFilterOrder(bdcConfig, updateConfigAction.filterOrder);
                bdcConfigSetFilterNumCoeffs(bdcConfig, updateConfigAction.filterNum);
                bdcConfigSetFilterDenCoeffs(bdcConfig, updateConfigAction.filterDen);
                bdcConfigSetSamplingIntervalMs(bdcConfig, updateConfigAction.samplingIntervalMs);

                bdcConfigSetKp(bdcConfig, updateConfigAction.kp);
                bdcConfigSetKi(bdcConfig, updateConfigAction.ki);
                bdcConfigSetKd(bdcConfig, updateConfigAction.kd);
                bdcConfigSetTf(bdcConfig, updateConfigAction.tf);
                bdcConfigSetKsi(bdcConfig, updateConfigAction.ksi);
                bdcConfigSetSatUp(bdcConfig, updateConfigAction.satUp);
                bdcConfigSetSatDown(bdcConfig, updateConfigAction.satDown);

                bool configSampler = updateConfigAction.samplingIntervalMs != samplingIntervalMs;
                ESP_LOGI(TAG, "Configuration updated");

                updateParametersFromConfig(&updateConfigAction);

                if (configSampler) {
                    setupSamplingTimer();
                }

                bdcConfigSave(bdcConfig);
            } else if (strcmp(event->topic, UPDATE_SETPOINT_TOPIC) == 0) {
                update_setpoint_action_t updateSetpointAction;
                memcpy(&updateSetpointAction, event->data, sizeof(updateSetpointAction));

                setPoint = updateSetpointAction.set_point;

                ESP_LOGI(TAG, "Switching Set-Point: %.2f", setPoint);
            } else {
                // nothing
            }
            break;
        default:
            break;
    }
}

_Noreturn void mqtt_task(void *arg) {
    esp_mqtt_client_config_t mqttConfig = {
            .broker.address.uri = MQTT_URI,
            .credentials = {
                    .username = MQTT_USERNAME,
                    .authentication.password = MQTT_PASSWORD,
                    .client_id = TAG
            }
    };

    mqttClient = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_CONNECTED, mqtt_event_handler, mqttClient);
    esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_DATA, mqtt_event_handler, mqttClient);
    esp_mqtt_client_start(mqttClient);

    char buff[MQTT_PACKET_SIZE] = {0};

    while (1) {
        control_event_t event;

        if (xQueueReceive(mqttQueue, &event, portMAX_DELAY)) {
            memcpy(buff, &event, sizeof(control_event_t));
            esp_mqtt_client_publish(mqttClient, TRANSMIT_INFO_TOPIC, buff, MQTT_PACKET_SIZE, 1, 1);
            memset(buff, 0, MQTT_PACKET_SIZE);
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup_pulldown_pin(uint8_t pin, gpio_mode_t mode) {
    gpio_config_t config = {
            .pin_bit_mask = 1ULL << pin,
            .mode = mode,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&config);
}

void updateParametersFromConfig(bdc_config_t *config) {
    setPoint = bdcConfigGetSetPoint(config);
    openLoopControlSignal = bdcConfigGetOpenLoopControlSignal(config);
    calibrationAngularTerm = bdcConfigGetCalibrationAngularTerm(config);
    calibrationLinerTerm = bdcConfigGetCalibrationLinearTerm(config);
    samplingIntervalMs = bdcConfigGetSamplingIntervalMs(config);

    filterOrder = bdcConfigGetFilterOrder(config);

    free(filterNum);
    free(filterDen);

    filterNum = calloc(filterOrder + 1, sizeof(double));
    filterDen = calloc(filterOrder + 1, sizeof(double));

    bdcConfigGetFilterNumCoeffs(config, filterNum);
    bdcConfigGetFilterDenCoeffs(config, filterDen);

    ESP_LOGI(TAG, "Set Point: %.2f", setPoint);
    ESP_LOGI(TAG, "Open Loop Control Signal: %.2f", openLoopControlSignal);
    ESP_LOGI(TAG, "Calibration Angular Term: %.2f", calibrationAngularTerm);
    ESP_LOGI(TAG, "Calibration Linear Term: %.2f", calibrationLinerTerm);
    ESP_LOGI(TAG, "Sampling Interval: %d", samplingIntervalMs);
    ESP_LOGI(TAG, "Filter Order: %d", filterOrder);

    ESP_LOGI(TAG, "Filter Coefficients: ");
    for (uint8_t i = 0; i < filterOrder + 1; i++) {
        ESP_LOGI(TAG, "Num[%d]: %.12f, Den[%d]: %.12f", i, filterNum[i], i, filterDen[i]);
    }

    ESP_LOGI(TAG, "Controller Parameters: ");
    ESP_LOGI(TAG, "Kp: %.2f, Ki: %.2f, Kd: %.2f, Tf: %.2f, Ksi: %.2f, SatUp: %.2f, SatDown: %.2f",
             bdcConfigGetKp(config), bdcConfigGetKi(config), bdcConfigGetKd(config), bdcConfigGetTf(config),
             bdcConfigGetKsi(config), bdcConfigGetSatUp(config), bdcConfigGetSatDown(config));
}

/* Main */
void app_main(void) {
    /* Init Devices */
    oneWireDevices = setupOneWire(oneWireDevicesPins, 1);
    ds18B20SetResolution(oneWireDevices[DS18B20], DS18B20_RESOLUTION);

    /* Pins */
    setup_pulldown_pin(IN1_GPIO, GPIO_MODE_OUTPUT);
    setup_pulldown_pin(IN2_GPIO, GPIO_MODE_OUTPUT);
    setup_pulldown_pin(EN_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_level(IN2_GPIO, 1);
    gpio_set_level(EN_GPIO, 1);

    setup_pulldown_pin(GPIO_NUM_14, GPIO_MODE_INPUT);
    setup_pulldown_pin(GPIO_NUM_21, GPIO_MODE_INPUT);
    setup_pulldown_pin(GPIO_NUM_48, GPIO_MODE_INPUT);

    /* Spiffs/Config */
    setupSpiffs();
    bdcConfigInit();

    bdcConfig = bdcConfigGet();

    if (bdcConfig != NULL) {
        ESP_LOGI(TAG, "Config loaded");
        updateParametersFromConfig(bdcConfig);
    } else {
        ESP_LOGE(TAG, "Config not found");
        esp_restart();
    }

    /* WIFI */
    setup_wifi(WIFI_MODE_STA_ONLY);

    /* PWM */
    setupPwm();

    /* PID Controller */
    pid = initPID(
            bdcConfigGetKp(bdcConfig),
            bdcConfigGetKi(bdcConfig),
            bdcConfigGetKd(bdcConfig),
            bdcConfigGetTf(bdcConfig),
            bdcConfigGetKsi(bdcConfig),
            samplingIntervalMs / 1000.0,
            bdcConfigGetSatUp(bdcConfig),
            bdcConfigGetSatDown(bdcConfig)
    );

    /* Sampling & MQTT */
    temperatureQueue = xQueueCreate(10, sizeof(sampling_event_t));
    mqttQueue = xQueueCreate(10, sizeof(control_event_t));

    xTaskCreatePinnedToCore(samplingTask, "sampling_task", 4096, NULL, 5, &samplingTaskHandle, 0);
    setupSamplingTimer();

    xTaskCreatePinnedToCore(controlLoopTask, "control_loop_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL, 1);
}
