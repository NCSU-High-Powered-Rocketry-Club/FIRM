#pragma once

#include "settings.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define COMMAND_READ_CHUNK_SIZE_BYTES 256
#define COMMAND_PAYLOAD_MAX_LEN_BYTES 248

#define DEVICE_NAME_LENGTH 32
#define DEVICE_ID_LENGTH 8
#define FIRMWARE_VERSION_LENGTH 8
#define FREQUENCY_LENGTH 2

typedef enum {
  CMDID_GET_DEVICE_INFO = 0x0001,
  CMDID_GET_DEVICE_CONFIG = 0x0002,
  CMDID_SET_DEVICE_CONFIG = 0x0003,
  CMDID_REBOOT = 0x0004,
  CMDID_MOCK_REQUEST = 0x0005,
  CMDID_SET_MAG_CALIBRATON = 0x0006,
  CMDID_SET_IMU_CALIBRATON = 0x0007,
  CMDID_GET_CALIBRATION = 0x0009,
  CMDID_CANCEL_REQUEST = 0x00FF,
} CommandIdentifier;

typedef enum {
  PROTOCOL_USB = 0x01,
  PROTOCOL_UART = 0x02,
  PROTOCOL_I2C = 0x03,
  PROTOCOL_SPI = 0x04
} DeviceProtocol;

typedef struct {
  char name[DEVICE_NAME_LENGTH];
  uint16_t frequency;
  DeviceProtocol protocol;
} DeviceConfig;

typedef struct {
  uint64_t id;
  char firmware_version[FIRMWARE_VERSION_LENGTH];
} DeviceInfo;

typedef struct {
  bool b;
} CommandSuccess;

typedef union {
  DeviceConfig device_config;
  DeviceInfo device_info;
  CalibrationSettings_t calibration_settings;
  CommandSuccess success;
} ResponsePacket;

typedef void (*CommandSystemResetFn)(void *ctx);

void commands_register_system_reset(CommandSystemResetFn fn, void *ctx);

uint32_t execute_command(CommandIdentifier identifier, uint8_t *data, uint32_t data_len,
                         ResponsePacket *response_packet);

/**
 * Convenience wrapper for USB command handling:
 * - Derives the response header/id from the request header/id
 * - Executes the command to populate the response payload
 *
 * Pure logic (no RTOS/HAL).
 */
uint32_t commands_execute_to_response(uint16_t request_identifier, uint8_t *payload_bytes,
                                      uint32_t payload_len, uint16_t *out_response_header,
                                      uint16_t *out_response_identifier,
                                      ResponsePacket *out_response_payload);