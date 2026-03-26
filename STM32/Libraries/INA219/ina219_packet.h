#pragma once

#include <stdint.h>

/**
 * @brief magnetometer data packet structur for the INA219.
 */
typedef struct{
    uint16_t shunt_voltage;
    uint16_t bus_voltage;
    uint16_t power;
    uint16_t current;
}INA219Packet_t;