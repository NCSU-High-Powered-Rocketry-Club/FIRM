/*
 * logger.c
 *
 *  Created on: Sep 19, 2025
 *      Author: craig
 */

#include "logger.h"
#include "fatfs.h"
#include "ff.h"
#include "usb_print_debug.h"
#include <stdint.h>
#include <stdio.h>

#include <string.h>

static char  buffer0[BUFFER_SIZE];
static char  buffer1[BUFFER_SIZE];
static char* current_buffer = buffer0;
static size_t current_offset = 0;
static FATFS fs;

FIL log_file;

extern DMA_HandleTypeDef hdma_sdio_tx; // Link to the DMA handler to check if busy

TCHAR file_name[32] = {'\0'};

FRESULT logger_write() {
    if (HAL_DMA_GetState(&hdma_sdio_tx) != HAL_DMA_STATE_READY) {
        serialPrintStr("Full");
        return FR_DISK_ERR;
    }

    // Pad the buffer
    for (int i = current_offset; i < BUFFER_SIZE; i++) {
        current_buffer[i] = 0;
    }

    UINT bytes_written = 0;

    // Set fast write so that it doesn't block on the dma request
    // See SD_write for fast implementation
    // We need to set it back because some of the internal SD card functions (like f_expand and
    // f_sync) need the old behavior
    sd_FastWriteFlag = 1;
    // We need to log exactly BUFFER_SIZE = sector size in order for this to work in future writes.
    FRESULT fr = f_write(&log_file, current_buffer, BUFFER_SIZE, &bytes_written);
    sd_FastWriteFlag = 0;

    if (fr != FR_OK) {
        serialPrintStr("ERR logger_write");
        return fr;
    } else {
        serialPrintStr(file_name);
    }

    return fr;
}

void logger_swap_buffers() {
    if (current_buffer == buffer0) {
        current_buffer = buffer1;
    } else {
        current_buffer = buffer0;
    }

    current_offset = 0;
}

// Writes the header to the log file
void logger_write_header() {
    // The length needs to be 4 byte aligned because the struts we are logging are 4 byte aligned
    // (they have floats).
    const char* header = "FIRM LOG v0.1\n";
    size_t len = strlen(header);
    int padded_len = ((len + 3) / 4) * 4;

    logger_ensure_capacity(padded_len);

    strcpy(current_buffer + current_offset, header);

    // Fill the remaining space with zeros
    for (int i = len; i < padded_len; i++) {
        current_buffer[current_offset + i] = 0;
    }

    current_offset += padded_len;
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

    uint8_t file_index = 0;
    DIR dj;
    FILINFO fno;

    // Find the next available log file
    do {
        file_index++;
        sprintf(file_name, "log%i.txt", file_index);
        fr = f_findfirst(&dj, &fno, "/", file_name);
    } while (fr == FR_OK && fno.fname[0]);

    if (fr != FR_OK) {
        return fr;
    }

    // Open the file
    fr = f_open(&log_file, file_name, FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK) {
        f_mount(0, SDPath, 0);
        return fr;
    }

    // Allocate a contiguous area to the file
    // We need to do this to avoid f_sync (slow!)
    {
        fr = f_truncate(&log_file);
        HAL_Delay(10);
        if (fr != FR_OK) {
            return fr;
        }

        // 2e8 bytes = (1 hour * ((8192 bytes * 4.4hz) * 1.5))
        fr = f_expand(&log_file, 2e8, 1);
        if (fr != FR_OK) {
            return fr;
        }
    }
    f_sync(&log_file);

    logger_write_header();

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

void logger_log_type_timestamp(char type) {
    // This should advance by TYPE_TIMESTAMP_SIZE
    current_buffer[current_offset++] = type;
    uint32_t current_time = HAL_GetTick();
    current_buffer[current_offset++] = (current_time >> 16) & 0xFF;
    current_buffer[current_offset++] = (current_time >> 8) & 0xFF;
    current_buffer[current_offset++] = current_time & 0xFF;
}

FRESULT logger_write_entry(char type, const void* payload, size_t payload_size) {
    // Make sure there is room for metadata + payload

    FRESULT fr = logger_ensure_capacity((int)(PACKET_METADATA_SIZE + payload_size));
    if (fr != FR_OK) return fr;

    logger_log_type_timestamp(type);

    // Copy payload bytes into the buffer
    memcpy(&current_buffer[current_offset], payload, payload_size);

    // Advance past the payload
    current_offset += payload_size;

    return fr;
}

void* logger_malloc_packet(size_t capacity) {
    if (logger_ensure_capacity(capacity + packet_metadata_size)) {
        return NULL;
    }
    return &current_buffer[current_offset + packet_metadata_size];
}