/*
 * logger.h
 *
 *  Created on: Sep 19, 2025
 *      Author: craig
 */

#pragma once

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
const int packet_metadata_size = 4;

// Initializes the SD card and creates the log file.
FRESULT logger_init();

// Ensure that there is enough space available in the current buffer. Swaps buffers if necessary
FRESULT logger_ensure_capacity(int capacity);

// Logs the type and timestamp. Advances the current offset
void logger_log_type_timestamp(char type);

/**
 * @brief allocates space for a sensor packet in the logger, not including the metadata info.
 * @note also ensures there is enough space available in the buffer. Swaps buffer if necessary.
 * 
 * @param capacity the size of the packet in bytes to allocate size for
 * @return void pointer to the allocated memory, or NULL if not enough memory
 */
void* logger_malloc_packet(size_t capacity);

FRESULT logger_write_entry(char type, const void* payload, size_t payload_size);
