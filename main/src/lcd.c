//
// Created by miguel on 09/01/25.
//

#include "lcd.h"

#define TAG "LCD"

// Hex Code | Command to LCD Instruction Register
// 0F       | Display on, cursor on, cursor blink on
// 01       | Clear display
// 02       | Return home
// 04       | Shift cursor to the left
// 06       | Shift cursor to the right
// 05       | Shift display to the right
// 07       | Shift display to the left
// 0E       | Display on, cursor blink
// 80       | Force cursor to the beginning ( 1st line)
// C0       | Force cursor to the beginning ( 2nd line)
// 38       | 8-bit mode, 2-line, 5x7 font
// 83       | Cursor line 1 position 3
// 3C       | Activate 2nd line
// 08       | Display off, cursor off
// C1       | Cursor line 2 position 1
// 0C       | Display on, cursor off
// C1       | Cursor line 2 position 1
// C2       | Cursor line 2 position 2

/**
 * @brief Send a command to the LCD
 * @param lcd The i2c device
 * @param cmd The command to send
 */
void lcd_send_cmd(I2CD_t lcd, uint8_t cmd) {
    uint8_t data_u, data_l;
    uint8_t data_t[4];

    data_u = cmd & 0xf0;
    data_l = (cmd << 4) & 0xf0;
    data_t[0] = data_u | 0x0C;  //en=1, rs=0
    data_t[1] = data_u | 0x08;  //en=0, rs=0
    data_t[2] = data_l | 0x0C;  //en=1, rs=0
    data_t[3] = data_l | 0x08;  //en=0, rs=0

    i2c_write(lcd, data_t, 4);
}

/**
 * @brief Send data to the LCD
 * @param lcd The i2c device
 * @param data The data to send
 */
void lcd_send_data(I2CD_t lcd, uint8_t data) {
    uint8_t data_u, data_l;
    uint8_t data_t[4];

    data_u = data & 0xf0;
    data_l = (data << 4) & 0xf0;

    data_t[0] = data_u | 0x0D;  //en=1, rs=0
    data_t[1] = data_u | 0x09;  //en=0, rs=0
    data_t[2] = data_l | 0x0D;  //en=1, rs=0
    data_t[3] = data_l | 0x09;  //en=0, rs=0

    i2c_write(lcd, data_t, 4);
}

/**
 * @brief Send a string to the LCD
 * @param lcd The i2c device
 * @param str The string to send
 */
void lcd_send_string(I2CD_t lcd, char *str) {
    while (*str) lcd_send_data(lcd, *str++);
}

/**
 * @brief Put the cursor in a specific position
 * @param lcd The i2c device
 * @param row The row (0 or 1)
 * @param col The col (0 to 15)
 */
void lcd_put_cur(I2CD_t lcd, int row, int col) {
    uint8_t command = (row == 0) ? (col | 0x80) : (col | 0xC0);
    lcd_send_cmd(lcd, command);
}

/**
 * @brief Clear the LCD
 * @param lcd The i2c device
 */
void lcd_clear(I2CD_t lcd) {
    lcd_send_cmd(lcd, 0x01);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/**
 * @brief Send a custom character to the LCD
 * @param lcd The i2c device
 * @param str The string to send
 * @param pos_x X position
 * @param pos_y Y position
 */
void lcd_send_custom(I2CD_t lcd, char *str, uint8_t pos_x, uint8_t pos_y) {
    lcd_send_cmd(lcd, 0x40);
    for (int i = 0; i < 8; ++i) {
        lcd_send_data(lcd, str[i]);
    }
    lcd_put_cur(lcd, pos_x, pos_y);
    lcd_send_data(lcd, 0);
    free(str);
}

/**
 * @brief Update a value in the LCD
 * @param lcd The i2c device
 * @param string The string to update
 * @param value The value to update
 * @param pos_x X position
 * @param pos_y Y position
 */
void update_value(I2CD_t lcd, char *string, float value, uint8_t pos_x, uint8_t pos_y) {
    sprintf(string, "%5.1f", value);
    lcd_put_cur(lcd, pos_x, pos_y);
    lcd_send_string(lcd, string);
}

/**
 * @brief Initialize the LCD
 * @param lcd The i2c device
 */
void lcd_init(I2CD_t lcd) {
    usleep(50000);
    lcd_send_cmd(lcd, 0x30);
    usleep(5000);
    lcd_send_cmd(lcd, 0x30);
    usleep(200);
    lcd_send_cmd(lcd, 0x30);
    usleep(10000);
    lcd_send_cmd(lcd, 0x20);
    usleep(10000);

    lcd_send_cmd(lcd, 0x28);
    usleep(1000);
    lcd_send_cmd(lcd, 0x08);
    usleep(1000);
    lcd_send_cmd(lcd, 0x01);
    usleep(1000);
    usleep(1000);
    lcd_send_cmd(lcd, 0x06);
    usleep(1000);
    lcd_send_cmd(lcd, 0x0C);
    usleep(1000);
}