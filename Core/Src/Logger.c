/*
 * spi_utils.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Praneeth, Ethan
 *
 */

#include "bmp581_spi.h"
#include <stdint.h>

typedef struct bmpType {
    uint16_t temp;
    uint16_t pres;
    struct bmpType* next;
} BMP;

typedef struct icmType {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    struct icmType* next;
} ICM;

BMP* addItem(BMP* head) {

    uint16_t t;
    uint16_t p;

    bmp_read(SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_channel, uint16_t cs_pin);

    BMP* newItem = (BMP*)malloc(sizeof(BMP)); // dynamically allocate memory
    if (newItem == NULL) {
        printf("Memory allocation failed.\n");
        return head;
    }

    // Copy data into the new flight
    strncpy(newItem->itemName, name, 50);
    newItem->quantity = q;
    newItem->price = p;
    newItem->next = head; // Insert at beginning

    return newItem;
}
