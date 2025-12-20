#include <bmp581.h>
#include <icm45686.h>
#include "logger.h"
#include <mmc5983ma.h>
#include "data_processing/preprocessor.h"
#include "usb_print_debug.h"
#include "usb_serializer.h"
#include "settings.h"
#include "firm.h"
#include "led.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>


volatile bool bmp581_has_new_data = false;
volatile bool icm45686_has_new_data = false;
volatile bool mmc5983ma_has_new_data = false;
static volatile bool uart_tx_done = true;

// check to verify if any new data has been collected, from any of the sensors
bool any_new_data_collected = false;
// instance of the calibrated data packet from the preprocessor to be reused
CalibratedDataPacket_t calibrated_packet = {0};

// instance of the serialized packet, will be reused
SerializedPacket_t serialized_packet = {0};

static UART_HandleTypeDef* firm_huart1;

int initialize_firm(SPIHandles* spi_handles_ptr, I2CHandles* i2c_handles_ptr, DMAHandles* dma_handles_ptr, UARTHandles* uart_handles_ptr) {
    firm_huart1 = uart_handles_ptr->huart1;

    // We use DWT (Data Watchpoint and Trace unit) to get a high resolution free-running timer
    // for our data packet timestamps. This allows us to use the clock cycle count instead of a
    // standard timestamp in milliseconds or similar, while not having any performance penalty.
    // Enables the trace and debug block in the core so that DWT registers become
    // accessible. This is required before enabling the DWT cycle counter.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Clear the DWT clock cycle counter to start counting from zero.
    DWT->CYCCNT = 0;

    // Enable the DWT cycle counter itself. Once active, it increments each CPU  
    // clock cycle so we can use clock cycles as data packet timestamps.
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Set the chip select pins to high, this means that they're not selected.
    // Note: We can't have these in the bmp581/imu/flash chip init functions, because those somehow
    // mess up with the initialization.
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // bmp581 cs pin
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // icm45686 cs pin
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // flash chip cs pin
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // mmc5983ma CS pin

    // Indicate that initialization is in progress:
    led_set_status(FIRM_UNINITIALIZED);

    HAL_Delay(500); // purely for debug purposes, allows time to connect to USB serial terminal

    if (icm45686_init(spi_handles_ptr->hspi2, GPIOB, GPIO_PIN_9)) {
        led_set_status(IMU_FAIL);
        return 1;
    }

    if (bmp581_init(spi_handles_ptr->hspi2, GPIOC, GPIO_PIN_2)) {
        led_set_status(BMP581_FAIL);
        return 1;
    }
    
    if (mmc5983ma_init(i2c_handles_ptr->hi2c1, 0x30)) {
        led_set_status(MMC5983MA_FAIL);
        return 1;
    }

    // set up settings module with flash chip
    if (settings_init(spi_handles_ptr->hspi1, GPIOC, GPIO_PIN_4)) {
        led_set_status(FLASH_CHIP_FAIL);
        return 1;
    }

    // Setup the SD card
    FRESULT res = logger_init(dma_handles_ptr->hdma_sdio_tx);
    if (res) {
        led_set_status(SD_CARD_FAIL);
        return 1;
    }
    
    // Indicate that all sensors initialized successfully
    led_set_status(FIRM_INITIALIZED);

    // get scale factor values for each sensor to put in header
    HeaderFields header_fields = {
        bmp581_get_temp_scale_factor(),
        bmp581_get_pressure_scale_factor(),
        icm45686_get_accel_scale_factor(),
        icm45686_get_gyro_scale_factor(),
        mmc5983ma_get_magnetic_field_scale_factor(),
    };


    // write the header to the log file
    logger_write_header(&header_fields);
    serializer_init_packet(&serialized_packet); // initializes the packet length and header bytes
    
    // the IMU runs into issues when the fifo is full at the very beginning, causing the interrupt
    // to be pulled back low too fast, and the ISR doesn't catch it for whatever reason. Doing
    // this initial read will prevent that.
    ICM45686Packet_t imu_packet;
    icm45686_read_data(&imu_packet);
    MMC5983MAPacket_t mag_packet;
    mmc5983ma_read_data(&mag_packet);
    BMP581Packet_t bmp_packet;
    bmp581_read_data(&bmp_packet);

    // Wait for interrupts to fire
    HAL_Delay(10);

    // Check if interrupts fired
    uint8_t interrupt_leds = 0b000;
    // Blink LEDs to indicate any failed interrupts
    for (int i = 0; i < 5; i++) {
        if (bmp581_has_new_data && icm45686_has_new_data && mmc5983ma_has_new_data) {
            break; // all interrupts fired successfully
        }
        if (!icm45686_has_new_data) {
            serialPrintStr("IMU didn't interrupt");
            interrupt_leds |= FAILED_INTERRUPT_IMU;
        }
        if (!bmp581_has_new_data) {
            serialPrintStr("BMP581 didn't interrupt");
            interrupt_leds |= FAILED_INTERRUPT_BMP;
        }
        if (!mmc5983ma_has_new_data) {
            serialPrintStr("MMC5983MA didn't interrupt");
            interrupt_leds |= FAILED_INTERRUPT_MAG;
        }
        led_set_status(interrupt_leds);
        HAL_Delay(500);
        led_set_status(FIRM_INITIALIZED);
        interrupt_leds = 0b000;
        HAL_Delay(500);
    }

    return 0;
};


void loop_firm(void) {
    if (bmp581_has_new_data) {
        BMP581Packet_t* bmp581_packet = logger_malloc_packet(sizeof(BMP581Packet_t));
        if (!bmp581_read_data(bmp581_packet)) {
            bmp581_has_new_data = false;
            logger_write_entry('B', sizeof(BMP581Packet_t));
            bmp581_convert_packet(bmp581_packet, &calibrated_packet);
            any_new_data_collected = true;
        }
    }

    if (icm45686_has_new_data) {
        ICM45686Packet_t* icm45686_packet = logger_malloc_packet(sizeof(ICM45686Packet_t)); 
        if (!icm45686_read_data(icm45686_packet)) {
            icm45686_has_new_data = false;
            logger_write_entry('I', sizeof(ICM45686Packet_t));
            icm45686_convert_packet(icm45686_packet, &calibrated_packet);
            any_new_data_collected = true;
        }
    }
    if (mmc5983ma_has_new_data) {
        MMC5983MAPacket_t* mmc5983ma_packet = logger_malloc_packet(sizeof(MMC5983MAPacket_t));
        if (!mmc5983ma_read_data(mmc5983ma_packet)) {
            mmc5983ma_has_new_data = false;
            logger_write_entry('M', sizeof(MMC5983MAPacket_t));
            mmc5983ma_convert_packet(mmc5983ma_packet, &calibrated_packet);
            any_new_data_collected = true;
        }
    }

    // If either USB or UART transfer setting is enabled and new data is collected,
    // serialize and transmit the data over the enabled interfaces.
    if (any_new_data_collected && (firmSettings.usb_transfer_enabled || firmSettings.uart_transfer_enabled)) {
        serialize_calibrated_packet(&calibrated_packet, &serialized_packet);
        if (firmSettings.usb_transfer_enabled) {
            usb_transmit_serialized_packet(&serialized_packet);
        }

        if (firmSettings.uart_transfer_enabled) {
            // Use DMA for UART transmission if the UART is ready. This avoids
            // blocking the CPU and allows high-throughput transfers.
            if (uart_tx_done) {
                uart_tx_done = false;
                HAL_UART_Transmit_DMA(firm_huart1, (uint8_t*)&serialized_packet, (uint16_t)sizeof(SerializedPacket_t));
            }
        }
        any_new_data_collected = false;
    }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == firm_huart1)
        uart_tx_done = true;
}
