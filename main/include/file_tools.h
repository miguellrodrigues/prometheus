//
// Created by miguel on 12/04/24.
//

#include <stdio.h>
#include <esp_log.h>
#include <string.h>

void createFile(const char *path);

uint8_t fileExists(const char *path);

FILE *openFile(const char *path, const char *mode);

void dumpFileHex(const char *path);

void closeFile(FILE *file);

void deleteFile(const char *path);