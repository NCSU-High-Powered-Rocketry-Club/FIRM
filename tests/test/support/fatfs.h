#ifndef FATFS_H
#define FATFS_H

#include "ff.h"

/* Return codes used by BSP */
#define MSD_OK 0x00
#define MSD_ERROR 0x01

typedef struct {
    uint8_t is_initialized;
} Diskio_drvTypeDef;

extern char SDPath[4]; 
extern Diskio_drvTypeDef SD_Driver;
extern volatile uint8_t sd_FastWriteFlag;

uint8_t FATFS_LinkDriver(Diskio_drvTypeDef *drv, char *path);
uint8_t FATFS_UnLinkDriver(char *path);
uint8_t BSP_SD_Init(void); 

#endif