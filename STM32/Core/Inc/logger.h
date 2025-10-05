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

// We use 8192 because it is the smallest sector size available when formatting SD card with right
// click menu on windows
#define BUFFER_SIZE 8192

/**
 * This is the size in bytes of the type of packet and the timestamp associated
 * with it.
 */
#define PACKET_METADATA_SIZE 4

// Initializes the SD card and creates the log file.
FRESULT logger_init();

// Ensure that there is enough space available in the current buffer. Swaps buffers if necessary
FRESULT logger_ensure_capacity(int capacity);

// Logs the type and timestamp. Advances the current offset
void logger_log_type_timestamp(char type);

FRESULT logger_write_entry(char type, const void* payload, size_t payload_size);

#endif /* INC_LOGGER_H_ */
