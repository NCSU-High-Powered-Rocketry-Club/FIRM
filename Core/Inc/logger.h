/*
 * logger.h
 *
 *  Created on: Sep 19, 2025
 *      Author: craig
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "fatfs.h"
#include "ff.h"
#include <stdint.h>

#include <string.h>

// #define BUFFER_SIZE 4096
// We use 8192 because it is the smallest sector size available when formatting SD card with right
// click menu on windows
#define BUFFER_SIZE 8192

#define TYPE_TIMESTAMP_SIZE 4

extern char* current_buffer;
extern UINT current_offset;

extern FIL log_file;

FRESULT logger_init();

// Ensure that there is enough space available in the current buffer. Swaps buffers if necessary
FRESULT logger_ensure_capacity(int capacity);

// Logs the type and timestamp. Advances the current offset
void logger_log_type_timestamp(char type);

#endif /* INC_LOGGER_H_ */
