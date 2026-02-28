//
// Created by miguel on 09/01/25.
//

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/unistd.h>
#include <i2c_device.h>

#define LCD_ADDRESS 0x27

void lcdSendCmd(I2CD_t lcd, uint8_t cmd);

void lcdSendData(I2CD_t lcd, uint8_t data);

void lcdSendString(I2CD_t lcd, char *str);

void lcdPutCur(I2CD_t lcd, int row, int col);

void lcdClear(I2CD_t lcd);

void lcdSendCustom(I2CD_t lcd, char *str, uint8_t posX, uint8_t posY);

void updateValue(I2CD_t lcd, char *string, float valor, uint8_t posX, uint8_t posY);

void lcdInit(I2CD_t ldc);

