#include <bdc_config.h>
#include <file_tools.h>
#include <string.h>
#include <stdlib.h>

#define TAG "BDC_CONFIG"

void bdcConfigInit(void) {
    if (fileExists(BDC_CONFIG_PATH)) {
        ESP_LOGI(TAG, "Configuration file exists, skipping initialization");
        return;
    }

    ESP_LOGI(TAG, "Configuration file does not exist");

    bdc_config_t *config = (bdc_config_t *) calloc(1, sizeof(bdc_config_t));
    if (config == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for config");
        return;
    }

    // Default filter coefficients
    const double default_num[1] = {1.0};
    const double default_den[1] = {1.0};

    // Setup default values
    config->setPoint = 47.75;
    config->openLoopControlSignal = 0.5;
    config->calibrationAngularTerm = 2.705661;
    config->calibrationLinearTerm = 0.393678;
    config->samplingIntervalMs = 5000;
    config->filterOrder = 0;

    // Copy default coefficients
    memcpy(config->filterNum, default_num, 1 * sizeof(double));
    memcpy(config->filterDen, default_den, 1 * sizeof(double));

    config->kp = -95.0/822.0;
    config->ki = -21.0/52432.0;
    config->kd = -905.0/2372.0;
    config->tf = 1795.0/171.0;
    config->ksi = 10.0;
    config->satUp = 2.0;
    config->satDown = 0.0;

    createFile(BDC_CONFIG_PATH);

    if (!fileExists(BDC_CONFIG_PATH)) {
        ESP_LOGE(TAG, "Failed to create configuration file");
        free(config);
        return;
    }

    bdcConfigSave(config);
    ESP_LOGI(TAG, "Configuration file created");
    free(config);
}

bool bdcConfigSave(bdc_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "NULL config pointer");
        return false;
    }

    FILE *file = openFile(BDC_CONFIG_PATH, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Error opening file for writing");
        return false;
    }

    size_t written = fwrite(config, sizeof(bdc_config_t), 1, file);
    closeFile(file);

    if (written != 1) {
        ESP_LOGE(TAG, "Failed to write configuration");
        return false;
    }

    return true;
}

bdc_config_t *bdcConfigGet(void) {
    FILE *file = openFile(BDC_CONFIG_PATH, "rb");
    if (!file) {
        ESP_LOGE(TAG, "Error opening file for reading");
        return NULL;
    }

    bdc_config_t *config = calloc(1, sizeof(bdc_config_t));
    if (config == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for config");
        return NULL;
    }

    size_t read = fread(config, sizeof(bdc_config_t), 1, file);
    closeFile(file);

    if (read != 1) {
        ESP_LOGE(TAG, "Failed to read configuration %d", read);

        free(config);
        return NULL;
    }

    return config;
}

// Set Point
double bdcConfigGetSetPoint(bdc_config_t *config) {
    return config->setPoint;
}

void bdcConfigSetSetPoint(bdc_config_t *config, double value) {
    config->setPoint = value;
}

// Open Loop Control Signal
double bdcConfigGetOpenLoopControlSignal(bdc_config_t *config) {
    return config->openLoopControlSignal;
}

void bdcConfigSetOpenLoopControlSignal(bdc_config_t *config, double value) {
    config->openLoopControlSignal = value;
}

// Calibration Angular Term
double bdcConfigGetCalibrationAngularTerm(bdc_config_t *config) {
    return config->calibrationAngularTerm;
}

void bdcConfigSetCalibrationAngularTerm(bdc_config_t *config, double value) {
    config->calibrationAngularTerm = value;
}

// Calibration Linear Term
double bdcConfigGetCalibrationLinearTerm(bdc_config_t *config) {
    return config->calibrationLinearTerm;
}

void bdcConfigSetCalibrationLinearTerm(bdc_config_t *config, double value) {
    config->calibrationLinearTerm = value;
}

void bdcConfigGetFilterNumCoeffs(bdc_config_t *config, double *coeffs) {
    memcpy(coeffs, config->filterNum, (config->filterOrder + 1) * sizeof(double));
}

void bdcConfigSetFilterNumCoeffs(bdc_config_t *config, const double *coeffs) {
    memset(config->filterNum, 0, sizeof(config->filterNum));
    memcpy(config->filterNum, coeffs, (config->filterOrder + 1) * sizeof(double));
}

void bdcConfigGetFilterDenCoeffs(bdc_config_t *config, double *coeffs) {
    memcpy(coeffs, config->filterDen, (config->filterOrder + 1) * sizeof(double));
}

void bdcConfigSetFilterDenCoeffs(bdc_config_t *config, const double *coeffs) {
    memset(config->filterDen, 0, sizeof(config->filterDen));
    memcpy(config->filterDen, coeffs, (config->filterOrder + 1) * sizeof(double));
}

// PID Controller Parameters
double bdcConfigGetKp(bdc_config_t *config) {
    return config->kp;
}

void bdcConfigSetKp(bdc_config_t *config, double value) {
    config->kp = value;
}

double bdcConfigGetKi(bdc_config_t *config) {
    return config->ki;
}

void bdcConfigSetKi(bdc_config_t *config, double value) {
    config->ki = value;
}

double bdcConfigGetKd(bdc_config_t *config) {
    return config->kd;
}

void bdcConfigSetKd(bdc_config_t *config, double value) {
    config->kd = value;
}

double bdcConfigGetTf(bdc_config_t *config) {
    return config->tf;
}

void bdcConfigSetTf(bdc_config_t *config, double value) {
    config->tf = value;
}

double bdcConfigGetKsi(bdc_config_t *config) {
    return config->ksi;
}

void bdcConfigSetKsi(bdc_config_t *config, double value) {
    config->ksi = value;
}

// Saturation Limits
double bdcConfigGetSatUp(bdc_config_t *config) {
    return config->satUp;
}

void bdcConfigSetSatUp(bdc_config_t *config, double value) {
    config->satUp = value;
}

double bdcConfigGetSatDown(bdc_config_t *config) {
    return config->satDown;
}

void bdcConfigSetSatDown(bdc_config_t *config, double value) {
    config->satDown = value;
}

// Sampling Interval
uint16_t bdcConfigGetSamplingIntervalMs(bdc_config_t *config) {
    return config->samplingIntervalMs;
}

void bdcConfigSetSamplingIntervalMs(bdc_config_t *config, uint16_t value) {
    config->samplingIntervalMs = value;
}

// Filter Order
uint8_t bdcConfigGetFilterOrder(bdc_config_t *config) {
    return config->filterOrder;
}

void bdcConfigSetFilterOrder(bdc_config_t *config, uint8_t value) {
    config->filterOrder = value;
}