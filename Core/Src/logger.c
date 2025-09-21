/*
 * logger.c
 *
 *  Created on: Sep 19, 2025
 *      Author: craig
 */

#include "logger.h"
#include "fatfs.h"
#include "ff.h"
#include <stdint.h>
#include <stdio.h>

#include <string.h>

char buffer0[BUFFER_SIZE];
char buffer1[BUFFER_SIZE];

char* current_buffer = buffer0;
UINT current_offset = 0;

FIL log_file;
FATFS fs;

extern DMA_HandleTypeDef hdma_sdio_tx; // Link to the DMA handler to check if busy

TCHAR fileName[32] = {'\0'};

void logger_write() {
    if (HAL_DMA_GetState(&hdma_sdio_tx) != HAL_DMA_STATE_READY) {
        serialPrintStr("Full");
        return;
    }

    // Pad the file
    for (int i = current_offset; i < BUFFER_SIZE; i++) {
        current_buffer[current_offset] = 0;
    }

    uint32_t t1 = HAL_GetTick();
    f_sync(&log_file);
    uint32_t t2 = HAL_GetTick();
    // TODO: check sd card state
    UINT bytes_written = 0;

    //	sd_FastWriteFlag = 1;
    sd_FastWriteFlag = 1;
    //    FRESULT fr = f_write(&log_file, current_buffer, current_offset, &bytes_written);
    // TODO we might need to log exactly buffer size to get it to work.
    FRESULT fr = f_write(&log_file, current_buffer, BUFFER_SIZE, &bytes_written);
    uint32_t t3 = HAL_GetTick();
    sd_FastWriteFlag = 0;

    if (fr != FR_OK) {
        // TODO: Error
        serialPrintStr("ERR logger_write");
    } else {
        char out[32] = {'\0'};
        sprintf(out, "T: %i %i %s", t2 - t1, t3 - t2, fileName);
        serialPrintStr(out);
    }

    // TODO: fr error
}

void logger_swap_buffers() {
    if (current_buffer == buffer0) {
        current_buffer = buffer1;
    } else {
        current_buffer = buffer0;
    }

    current_offset = 0;
}

// FRESULT logger_write_header()
void logger_write_header() {
    if (current_offset != 0) {
        // TODO: error
        serialPrintStr("ERR logger_write");
    }

    const char* header = "FLOG v0.1\n";
    sprintf(current_buffer, header);
    current_offset = strlen(header);
    logger_write();
    logger_swap_buffers();

    // TODO error handling
    // return FR_OK;
}

FRESULT logger_init() {
    // Re-initialize SD
    if (BSP_SD_Init() != MSD_OK) {
        return FR_NOT_READY;
    }

    // Re-initialize FATFS
    if (FATFS_UnLinkDriver(SDPath) != 0) {
        return FR_NOT_READY;
    }
    if (FATFS_LinkDriver(&SD_Driver, SDPath) != 0) {
        return FR_NOT_READY;
    }

    // Mount file system
    FRESULT fr = f_mount(&fs, SDPath, 0);
    if (fr != FR_OK) {
        f_mount(NULL, SDPath, 0);
        return fr;
    }

    uint8_t fileIndex = 0;
    DIR dj;
    FILINFO fno;

    // Find the next available log file
    do {
        fileIndex++;
        sprintf(fileName, "log%i.txt", fileIndex);
        fr = f_findfirst(&dj, &fno, "/", fileName);
    } while (fr == FR_OK && fno.fname[0]);

    if (fr != FR_OK) {
        // TODO: error
        return fr;
    }

    serialPrintStr(fileName);

    fr = f_open(&log_file, fileName, FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK) {
        // TODO: error
        f_mount(0, SDPath, 0);
        return fr;
    }

#if 1
    {
        // Allocate a contiguous area to the file
        fr = f_truncate(&log_file);
        HAL_Delay(10);
        if (fr != FR_OK) {
            return fr;
        }
        // Create contiguous file for faster write transactions later
        fr = f_expand(&log_file, 50000000, 1);
        if (fr != FR_OK) {
            return fr;
        }
    }

    f_sync(&log_file);
#endif

    // TODO: truncate and expand for continuous file

    logger_write_header();

    f_sync(&log_file);

    return fr;
}

FRESULT logger_ensure_capacity(int capacity) {
    if (current_offset + capacity > BUFFER_SIZE) {
        logger_write();

        logger_swap_buffers();
    }

    // TODO error handling
    return FR_OK;
}
