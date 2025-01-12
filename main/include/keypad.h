//
// Created by miguel on 08/04/24.
//

// keypad uses the PCF8574 Remote 8-Bit I/O Expander for I2C Bus
// the keypad is a 4x3 matrix

#include <stdlib.h>
#include <i2c_device.h>

#define KEYPAD_ADDRESS 0x20

void keypad_init(I2CD_t device);

char keypad_get_key(I2CD_t device);
