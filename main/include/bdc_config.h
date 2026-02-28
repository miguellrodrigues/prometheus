#include <stdbool.h>
#include <stdint.h>

#define BDC_CONFIG_PATH "/spiffs/bdc_config.bin"
#define MAX_FILTER_ORDER 9

typedef struct {
    double setPoint;
    double openLoopControlSignal;

    double calibrationAngularTerm;
    double calibrationLinearTerm;

    double kp;
    double ki;
    double kd;
    double tf;
    double ksi;
    double satUp;
    double satDown;

    double filterNum[MAX_FILTER_ORDER+1];
    double filterDen[MAX_FILTER_ORDER+1];

    uint16_t samplingIntervalMs;
    uint8_t filterOrder;
} bdc_config_t;

// Initialization and persistence
void bdcConfigInit(void);

bool bdcConfigSave(bdc_config_t *config);

bdc_config_t *bdcConfigGet(void);

// Set Point
double bdcConfigGetSetPoint(bdc_config_t *config);

void bdcConfigSetSetPoint(bdc_config_t *config, double value);

// Open Loop Control Signal
double bdcConfigGetOpenLoopControlSignal(bdc_config_t *config);

void bdcConfigSetOpenLoopControlSignal(bdc_config_t *config, double value);

// Safety Control Signal
double bdcConfigGetSafetyControlSignal(bdc_config_t *config);

void bdcConfigSetSafetyControlSignal(bdc_config_t *config, double value);

// Calibration Terms
double bdcConfigGetCalibrationAngularTerm(bdc_config_t *config);

void bdcConfigSetCalibrationAngularTerm(bdc_config_t *config, double value);

double bdcConfigGetCalibrationLinearTerm(bdc_config_t *config);

void bdcConfigSetCalibrationLinearTerm(bdc_config_t *config, double value);

// Filter Coefficients
void bdcConfigGetFilterNumCoeffs(bdc_config_t *config, double *coeffs);

void bdcConfigSetFilterNumCoeffs(bdc_config_t *config, const double *coeffs);

void bdcConfigGetFilterDenCoeffs(bdc_config_t *config, double *coeffs);

void bdcConfigSetFilterDenCoeffs(bdc_config_t *config, const double *coeffs);

// PID Controller Parameters
double bdcConfigGetKp(bdc_config_t *config);

void bdcConfigSetKp(bdc_config_t *config, double value);

double bdcConfigGetKi(bdc_config_t *config);

void bdcConfigSetKi(bdc_config_t *config, double value);

double bdcConfigGetKd(bdc_config_t *config);

void bdcConfigSetKd(bdc_config_t *config, double value);

double bdcConfigGetTf(bdc_config_t *config);

void bdcConfigSetTf(bdc_config_t *config, double value);

double bdcConfigGetKsi(bdc_config_t *config);

void bdcConfigSetKsi(bdc_config_t *config, double value);

// Saturation Limits
double bdcConfigGetSatUp(bdc_config_t *config);

void bdcConfigSetSatUp(bdc_config_t *config, double value);

double bdcConfigGetSatDown(bdc_config_t *config);

void bdcConfigSetSatDown(bdc_config_t *config, double value);

// Sampling and Filter Settings
uint16_t bdcConfigGetSamplingIntervalMs(bdc_config_t *config);

void bdcConfigSetSamplingIntervalMs(bdc_config_t *config, uint16_t value);

uint8_t bdcConfigGetFilterOrder(bdc_config_t *config);

void bdcConfigSetFilterOrder(bdc_config_t *config, uint8_t value);