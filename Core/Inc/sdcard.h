/*
 * sdcard.h
 *
 *  Created on: Aug 19, 2025
 *      Author: harshil
 */
#pragma once

#ifndef INC_SDCARD_H_
#define INC_SDCARD_H_
#include <stdint.h>
#include "ff.h"
#include "fatfs.h"

#include <string.h>

FRESULT AppendToFile(char* path, size_t path_len, const char* msg, size_t msg_len);

extern FRESULT fres;
extern uint16_t raw_temp;
extern char log_path[];
extern char buf[];

FRESULT sdcard_init(void);
FRESULT sdcard_write(const char* temp);

#endif /* INC_SDCARD_H_ */
