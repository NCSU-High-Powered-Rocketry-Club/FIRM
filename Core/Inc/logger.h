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

#define BUFFER_SIZE 4096

extern char* current_buffer;
extern UINT current_offset;

extern FIL log_file;

FRESULT logger_init();
FRESULT logger_ensure_capacity(int capacity);

#endif /* INC_LOGGER_H_ */
