#include <bmp581.h>
#include <icm45686.h>
#include "logger.h"
#include <mmc5983ma.h>
#include 
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






void loop_firm(void) {

    // If either USB or UART transfer setting is enabled and new data is collected,
    // serialize and transmit the data over the enabled interfaces.
    if (firmSettings.usb_transfer_enabled || firmSettings.uart_transfer_enabled) {
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
    }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == firm_huart1)
        uart_tx_done = true;
}
