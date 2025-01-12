//
// Created by miguel on 12/04/24.
//

#include <stdio.h>
#include <esp_log.h>
#include <string.h>

void create_file(const char *path);

uint8_t file_exists(const char *path);

FILE *open_file(const char *path, const char *mode);

void dump_file_hex(const char *path);

void close_file(FILE *file);