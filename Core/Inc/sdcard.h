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

/**
 * @brief Initialize the SD card. Initialize the filesystem, mount it, and open the file and return the file object.
 * @param path: The path to the file we want to write.
 * @retval return the file object
 */
FRESULT sdCardInit(FIL *file_obj, char *path, size_t path_len);

/**
 * @brief adds data to file
 */
FRESULT AppendToFile(FIL *file_obj, const char *msg, size_t msg_len);

/**
 * @brief Save the file contents instantly
 * @param file_obj: Pointer to the file object from sdCardInit
 */
void sdCardSave(FIL *file_obj);

/**
 * @brief Close the file and sync it.
 * @param file_obj: the file object returned after initializing the sd card
 */
void sdCardClose(FIL *file_obj);

extern FRESULT fres;
extern uint16_t raw_temp;
extern char log_path[];
extern char buf[];

#endif /* INC_SDCARD_H_ */
