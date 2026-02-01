#include <string.h>
#include <stdio.h>
#include "unity.h"
#include "logger.h"
#include "settings.h"
#include "mock_ff.h"
#include "mock_fatfs.h"
#include "mock_stm32f4xx_hal.h"
#include "mock_usb_print_debug.h"
#include "mock_w25q128jv.h"

char SDPath[4];
Diskio_drvTypeDef SD_Driver;
DMA_HandleTypeDef dma_sdio_tx_handle;
volatile uint8_t sd_FastWriteFlag = 0;

static FRESULT f_findfirst_not_found_cb(DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern, int cmock_num_calls) {
    if (fno != NULL) fno->fname[0] = '\0';
    return FR_OK;
}

static FRESULT f_findfirst_log1_exists_cb(DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern, int cmock_num_calls) {
    if (cmock_num_calls == 0) {
        // First call: "log1.txt" exists
        strncpy((char*)fno->fname, "log1.txt", sizeof(fno->fname));
        return FR_OK;
    }
    // Second call: Nothing found (so log2.txt is free)
    fno->fname[0] = '\0';
    return FR_OK;
}

void setUp(void) {
    // Ceedling automagically initializes mocks before this runs
    memset(SDPath, 0, sizeof(SDPath));
    memset(&dma_sdio_tx_handle, 0, sizeof(dma_sdio_tx_handle));
}

void tearDown(void) {}

void test_logger_init_HandleIsNull(void) {
    TEST_ASSERT_EQUAL(FR_DENIED, logger_init(NULL));
}

void test_logger_init_SDInitFails(void) {
    BSP_SD_Init_ExpectAndReturn(MSD_ERROR);
    TEST_ASSERT_EQUAL(FR_NOT_READY, logger_init(&dma_sdio_tx_handle));
}

void test_logger_init_HandleMountFailure(void) {
    BSP_SD_Init_ExpectAndReturn(MSD_OK);
    FATFS_UnLinkDriver_ExpectAndReturn(SDPath, 0);
    FATFS_LinkDriver_ExpectAndReturn(&SD_Driver, SDPath, 0);

    // Expect fail, then cleanup
    f_mount_ExpectAndReturn(NULL, SDPath, 0, FR_DISK_ERR);
    f_mount_IgnoreArg_fs();
    
    // The cleanup call inside the error handler
    f_mount_ExpectAndReturn(NULL, SDPath, 0, FR_OK);
    f_mount_IgnoreArg_fs();

    TEST_ASSERT_EQUAL(FR_DISK_ERR, logger_init(&dma_sdio_tx_handle));
}

void test_logger_init_CreateLog1_When_NoLogsExist(void) {
    BSP_SD_Init_ExpectAndReturn(MSD_OK);
    FATFS_UnLinkDriver_ExpectAndReturn(SDPath, 0);
    FATFS_LinkDriver_ExpectAndReturn(&SD_Driver, SDPath, 0);
    f_mount_ExpectAndReturn(NULL, SDPath, 0, FR_OK);
    f_mount_IgnoreArg_fs();

    // No files exist
    f_findfirst_StubWithCallback(f_findfirst_not_found_cb);

    // Expect opening "log1.txt"
    f_open_ExpectAndReturn(NULL, "log1.txt", FA_CREATE_NEW | FA_WRITE, FR_OK);
    f_open_IgnoreArg_fp();
    
    f_truncate_ExpectAndReturn(NULL, FR_OK);
    f_truncate_IgnoreArg_fp();
    HAL_Delay_Expect(10);
    f_expand_ExpectAndReturn(NULL, (FSIZE_t)5e6, 1, FR_OK);
    f_expand_IgnoreArg_fp();
    f_sync_ExpectAndReturn(NULL, FR_OK);
    f_sync_IgnoreArg_fp();

    TEST_ASSERT_EQUAL(FR_OK, logger_init(&dma_sdio_tx_handle));
}

void test_logger_init_CreateLog2_When_Log1Exists(void) {
    BSP_SD_Init_ExpectAndReturn(MSD_OK);
    FATFS_UnLinkDriver_ExpectAndReturn(SDPath, 0);
    FATFS_LinkDriver_ExpectAndReturn(&SD_Driver, SDPath, 0);
    f_mount_ExpectAndReturn(NULL, SDPath, 0, FR_OK);
    f_mount_IgnoreArg_fs();

    // Use callback: log1 exists, next is free
    f_findfirst_StubWithCallback(f_findfirst_log1_exists_cb);

    // Expect opening "log2.txt"
    f_open_ExpectAndReturn(NULL, "log2.txt", FA_CREATE_NEW | FA_WRITE, FR_OK);
    f_open_IgnoreArg_fp();
    
    f_truncate_ExpectAndReturn(NULL, FR_OK);
    f_truncate_IgnoreArg_fp();
    HAL_Delay_Expect(10);
    f_expand_ExpectAndReturn(NULL, (FSIZE_t)5e6, 1, FR_OK);
    f_expand_IgnoreArg_fp();
    f_sync_ExpectAndReturn(NULL, FR_OK);
    f_sync_IgnoreArg_fp();

    TEST_ASSERT_EQUAL(FR_OK, logger_init(&dma_sdio_tx_handle));
}
