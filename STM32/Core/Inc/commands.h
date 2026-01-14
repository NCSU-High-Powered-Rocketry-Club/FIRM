#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define COMMAND_READ_CHUNK_SIZE_BYTES 64
#define COMMAND_PAYLOAD_MAX_LEN_BYTES 56

// TODO clean these up
#define CMD_START_MARKER_1 0x55
#define CMD_START_MARKER_2 0xAA
#define CMD_PACKET_SIZE 64

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

typedef union {
  DeviceConfig device_config;
  DeviceInfo device_info;
  bool success;
} ResponsePacket;

uint32_t execute_command(uint32_t command_id, uint8_t *data, uint32_t data_len, ResponsePacket* response_packet);