/*
 * icm45686.h
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#pragma once
#include "packets.h"
#include "usb_print_debug.h"

/**
 * @brief Indirect Register type enumeration
 */
typedef enum IREGMap { IMEM_SRAM, IPREG_BAR, IPREG_SYS1, IPREG_SYS2, IPREG_TOP1 } IREGMap_t;

/**
 * @brief initializes ICM45686 IMU settings
 *
 * @note disables the AUX1 channel, sets to 32g cap, 1600hz ODR, 4000 dps cap, and enables
 * 		 interrupt pin on data_ready. Interrupt is set to active-low, latching, and
 * push-pull.
 *
 * @param hspi specifies the SPI channel that the IMU is connected to.
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 if successful, 1 if unsuccessful due to a register being written to incorrectly
 *         or if a register did not output the expected value when read.
 */
int imu_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);

/**
 * @brief reads acceleration and gyroscope data from the IMU, if the data is ready.
 * @note this function should only be called when the interrupt pin is active. This function will
 *       reset the interrupt pin to it's inactive state.
 *
 * @param hspi specifies the SPI channel that the IMU is connected to.
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @param packet pointer to the IMU packet where the data will be stored
 * @retval 0 if successful, 1 if unsuccessful due to the data not being ready. In this case, the
 *         interrupt pin will still be reset to the inactive state, but no data will be collected.
 */
int imu_read(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin,
             IMUPacket_t* packet);

/**
 * @brief reads an indirect register from the IMU
 *
 * @note a minimum wait time of 4 microseconds is required between multiple ireg reads or writes.
 *
 * @param hspi specifies the SPI channel that the IMU is connected to.
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @param register_map the IREGMap enumeration value that the ireg register is located in.
 * @param ireg_addr the intended register address to read. From the datasheet, the register
 * 		  will either be a standard 8-bit register, or a 12-bit register. ireg_addr expects
 * 		  a 16-bit unsigned integer to account for this.
 * @param result a pointer to the variable that the result of the read will be stored in.
 * @retval None
 */
void spi_ireg_read(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin,
                   IREGMap_t register_map, uint16_t ireg_addr, uint8_t* result);

/**
 * @brief writes to an indirect register of the IMU
 *
 * @note a minimum wait time of 4 microseconds is required between multiple ireg reads or writes.
 *
 * @param hspi specifies the SPI channel that the IMU is connected to.
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @param register_map the IREGMap enumeration value that the ireg register is located in.
 * @param ireg_addr the intended register address to write to. From the datasheet, the register
 * 		  will either be a standard 8-bit register, or a 12-bit register. ireg_addr expects
 * 		  a 16-bit unsigned integer to account for this.
 * @param data the data to write to the indirect register address, as an 8-bit unsigned integer.
 * @retval None
 */
void spi_ireg_write(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin,
                    IREGMap_t register_map, uint16_t ireg_addr, uint8_t data);
