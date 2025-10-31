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

/**
 * @brief Ensure that there is enough space available in the current buffer.
 * @note Swaps buffers if necessary, when other buffer is full
 * 
 * @param capacity the number of bytes to ensure capacity of
 * @retval File Status error code, 0 on success.
 */
static FRESULT logger_ensure_capacity(int capacity);

/**
 * @brief Logs the type and clock cycle timestamp. This will be writen as 1 byte for the type
 *        and three bytes (uint24) for the clock cycle count. The clock cycle count will overflow
 *        every ~0.1 seconds.
 * @note advances the current offset variable for the buffer
 * 
 * @param type the character that signifies the type of packet being logged
 * @retval None
 */
static void logger_log_type_timestamp(char type);

/**
 * @brief writes a filled buffer to the Micro SD Card via DMA
 * 
 * @retval File Status error code, 0 on success.
 */
static FRESULT logger_write();

/**
 * @brief swaps the buffers for the DMA logging when one gets full
 */
static void logger_swap_buffers();
 
/**
 * This is the size in bytes of the type of packet and the timestamp associated
 * with it.
 */
static const int packet_metadata_size = 4;

static char  buffer0[SD_SECTOR_SIZE];
static char  buffer1[SD_SECTOR_SIZE];
static char* current_buffer = buffer0;
static size_t current_offset = 0;
static FATFS fs;
static FIL log_file;
static DMA_HandleTypeDef* hdma_sdio_tx; // Link to the DMA handler to check if busy

TCHAR file_name[32] = {'\0'};


FRESULT logger_init(DMA_HandleTypeDef* dma_sdio_tx_handle) {
    // set the dma handle to the static variable
    if (dma_sdio_tx_handle == NULL) {
        return FR_DENIED;
    }
    hdma_sdio_tx = dma_sdio_tx_handle;

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

    return fr;
}

FRESULT logger_write_header(HeaderFields* sensor_scale_factors) {
    const char* firm_log_header = "FIRM LOG v1.0\n";
    size_t header_len = strlen(firm_log_header);
    size_t scale_factor_len = sizeof(HeaderFields);
    size_t len = header_len + scale_factor_len;

    FRESULT error_status = logger_ensure_capacity(len);
    if (error_status) {
        return error_status;
    }

    // copy "FIRM LOG" text
    memcpy(current_buffer + current_offset, firm_log_header, header_len);

    // copy sensor scale factor struct
    memcpy(current_buffer + current_offset + header_len, sensor_scale_factors, scale_factor_len);

    current_offset += len;

    return error_status;
}

void* logger_malloc_packet(size_t capacity) {
    if (logger_ensure_capacity(capacity + packet_metadata_size)) {
        return NULL;
    }
    return &current_buffer[current_offset + packet_metadata_size];
}

void logger_write_entry(char type, size_t packet_size) {
    logger_log_type_timestamp(type);
    // advance current offset by packet size
    current_offset += packet_size;
}


static FRESULT logger_ensure_capacity(int capacity) {
    if (current_offset + capacity > SD_SECTOR_SIZE) {
        logger_write();

        logger_swap_buffers();
    }

    // TODO error handling
    return FR_OK;
}

static void logger_log_type_timestamp(char type) {
    current_buffer[current_offset++] = type;
    uint32_t current_time = DWT->CYCCNT;
    current_buffer[current_offset++] = (current_time >> 16) & 0xFF;
    current_buffer[current_offset++] = (current_time >> 8) & 0xFF;
    current_buffer[current_offset++] = current_time & 0xFF;
}


static FRESULT logger_write() {
    if (HAL_DMA_GetState(hdma_sdio_tx) != HAL_DMA_STATE_READY) {
        serialPrintStr("Full");
        return FR_DISK_ERR;
    }

    // Pad the buffer
    for (int i = current_offset; i < SD_SECTOR_SIZE; i++) {
        current_buffer[i] = 0;
    }

    UINT bytes_written = 0;

    // Set fast write so that it doesn't block on the dma request
    // See SD_write for fast implementation
    // We need to set it back because some of the internal SD card functions (like f_expand and
    // f_sync) need the old behavior
    sd_FastWriteFlag = 1;
    // We need to log exactly BUFFER_SIZE = sector size in order for this to work in future writes.
    FRESULT fr = f_write(&log_file, current_buffer, SD_SECTOR_SIZE, &bytes_written);
    sd_FastWriteFlag = 0;

    if (fr != FR_OK) {
        serialPrintStr("ERR logger_write");
        return fr;
    } else {
        //serialPrintStr(file_name);
    }

    return fr;
}

static void logger_swap_buffers() {
    if (current_buffer == buffer0) {
        current_buffer = buffer1;
    } else {
        current_buffer = buffer0;
    }

    current_offset = 0;
}
