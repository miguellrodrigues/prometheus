//
// Created by miguel on 08/04/24.
//

#include <keypad.h>

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 3

// PCF8574 connections:
// P0,P1,P2 -> Cols
// P3,P4,P5,P6 -> Rows
// P7 -> unused

static const uint8_t config_mask = 0b01111111;

const char keys[KEYPAD_ROWS][KEYPAD_COLS] = {{'#', '0', '*'},
                                             {'9', '8', '7'},
                                             {'6', '5', '4'},
                                             {'3', '2', '1'}};

/**
 * @brief Initialize the keypad
 * @param device The i2c device
 */
void keypad_init(I2CD_t device) {
    if (device == NULL) {
        return;
    }

    uint8_t *data = malloc(1);
    *data         = config_mask;

    // config first time
    i2c_write(device, data, 1);
    free(data);
}

/**
 * @brief Get the key pressed
 * @param device The i2c device
 * @return The pressed key if theres' one, otherwise '\0'
 */
char keypad_get_key(I2CD_t device) {
    if (device == NULL) {
        return '\0';
    }

    uint8_t *read_data = calloc(1, sizeof (uint8_t));

    // first, we gonna set each col bit to 1, in order to find the row
    for (uint8_t j = 0; j < KEYPAD_COLS; j++) {
        uint8_t write_data = config_mask & ~(1 << j); // we need to use this mask to not remove the the inputs config

        i2c_write(device, &write_data, 1);
        i2c_read (device, read_data, 1);

        // the return is 1 when not pressed, and 0 otherwise, because of the construction of the keypad
        // so first we discard the cols bits cause we know what col we are setting,
        // and then invert the bits
        uint8_t response = ~((*read_data) >> KEYPAD_COLS);
        response = response & 0x0F; // we only need the first 4 bits

        // clean read_data
        *read_data = 0;

        // verify if the response is not 0.
        if (!response) continue;

        // now we can find the row
        uint8_t i = response == 0 ? 0 : __builtin_ctz(response);

        return keys[i][j];
    }

    free(read_data);

    return '\0';
}