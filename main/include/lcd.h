//
// Created by miguel on 09/01/25.
//

#ifndef FUNCTION_GENERATOR_LCD_H
#define FUNCTION_GENERATOR_LCD_H

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/unistd.h>
#include <i2c_device.h>

#define LCD_ADDRESS 0x27

void lcd_send_cmd(I2CD_t lcd, uint8_t cmd);

void lcd_send_data(I2CD_t lcd, uint8_t data);

void lcd_send_string(I2CD_t lcd, char *str);

void lcd_put_cur(I2CD_t lcd, int row, int col);

void lcd_clear(I2CD_t lcd);

void lcd_send_custom(I2CD_t lcd, char *str, uint8_t pos_x, uint8_t pos_y);

void update_value(I2CD_t lcd, char *string, float valor, uint8_t pos_x, uint8_t pos_y);

void lcd_init(I2CD_t ldc);

#endif //FUNCTION_GENERATOR_LCD_H
