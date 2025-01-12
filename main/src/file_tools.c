//
// Created by miguel on 12/04/24.
//

#include <file_tools.h>

#define TAG "FILE_TOOLS"

/**
 * @brief
 * @param path The path to the file
 * @return
 */
uint8_t file_exists(const char *path) {
    FILE *file = fopen(path, "r");
    if (file) {
        fclose(file);
        return 1;
    }
    return 0;
}

/**
 * @brief Open a file if it exists
 * @param path The path of the file
 * @param mode The opening mode
 * @return
 */
FILE *open_file(const char *path, const char *mode) {
    FILE *file = fopen(path, mode);
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file: %s", path);
        return NULL;
    }
    return file;
}

/**
 * @broef Close a file
 * @param file The file
 */
void close_file(FILE *file) {
    fclose(file);
}

/**
 * @brief Dump a file in hexadecimal
 * @param file The file to dump
 */
void dump_file_hex(const char *path) {
    if (!path) {
        ESP_LOGE(TAG, "File path is NULL");
        return;
    }

    FILE *f = open_file(path, "r");

    // Get the file size
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    // Read the file
    char *buffer = malloc(size);

    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        close_file(f);
        return;
    }

    fread(buffer, 1, size, f);

    // Dump the file
    for (int i = 0; i < size; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");

    free(buffer);
    close_file(f);
}

/**
 * @brief Create a file
 * @param path Path to create the file
 */
void create_file(const char *path) {
    FILE *file = open_file(path, "w+");
    if (!file) {
        ESP_LOGE(TAG, "Failed to create file: %s", path);
        return;
    }
    close_file(file);
}