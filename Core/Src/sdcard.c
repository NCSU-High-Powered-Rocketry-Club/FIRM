/*
 * sdcard.c
 *
 *  Created on: Aug 19, 2025
 *      Author: harshil
 */

#include "sdcard.h"
#include "usb_print_debug.h"

FATFS fs;
char log_path[] = "/TEMPLOG.TXT";
char buf[20];

FRESULT sdCardInit(FIL* file_obj, char* path, size_t path_len) {
    FRESULT stat;

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
    stat = f_mount(&fs, SDPath, 0);
    if (stat != FR_OK) {
        f_mount(NULL, SDPath, 0);
        return stat;
    }

    // Bounds check on strings
    if ((path[path_len] != 0)) {
        return FR_INVALID_NAME;
    }

    // Open file for appending
    stat = f_open(file_obj, path, FA_WRITE | FA_OPEN_APPEND);
    if (stat != FR_OK) {
        f_mount(0, SDPath, 0);
        return stat;
    }

    return stat;
}

FRESULT AppendToFile(FIL* file_obj, const char* msg, size_t msg_len) {
    UINT testByte;

    // Write message to end of file
    //  serialPrintStr("Before sd card write");
    FRESULT stat = f_write(file_obj, msg, msg_len, &testByte);
    if (stat != FR_OK) {
        serialPrintStr("failed write to sd card");
        //	  f_mount(NULL, SDPath, 0);
        return stat;
    }

    return stat;
}

void sdCardSave(FIL* file_obj) { f_sync(file_obj); }

void sdCardClose(FIL* file_obj) {
    // Sync, close file, unmount
    f_close(file_obj);
    f_mount(NULL, SDPath, 0);
}
