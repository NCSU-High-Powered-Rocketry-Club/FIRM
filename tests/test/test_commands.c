#include "unity.h"
#include "commands.h"
#include "settings.h"
#include "utils.h"

#include <string.h>

// `commands.c` pulls in `settings.c`, which depends on flash + GPIO.
// Include the mocks so the unit-test link step succeeds.
#include "mock_w25q128jv.h"
#include "mock_stm32f4xx_hal.h"
#include "mock_usb_print_debug.h"

static int g_reset_called;
static void *g_reset_ctx_seen;

static void reset_cb(void *ctx) {
    g_reset_called++;
    g_reset_ctx_seen = ctx;
}


void setUp(void) {
    memset(&firmSettings, 0, sizeof(firmSettings));
    g_reset_called = 0;
    g_reset_ctx_seen = NULL;
}

void tearDown(void) {
    // no-op
}

void test_execute_command_get_device_info_populates_payload(void) {
    ResponsePacket response;
    memset(&response, 0, sizeof(response));

    firmSettings.device_uid = 0x1122334455667788ULL;
    strncpy(firmSettings.firmware_version, "v9.9.9", FIRMWARE_VERSION_LENGTH);

    uint32_t n = execute_command(CMDID_GET_DEVICE_INFO, NULL, 0, &response);

    TEST_ASSERT_EQUAL_UINT32(sizeof(response.device_info), n);
    TEST_ASSERT_EQUAL_UINT64(0x1122334455667788ULL, response.device_info.id);
    TEST_ASSERT_EQUAL_STRING_LEN("v9.9.9", response.device_info.firmware_version, FIRMWARE_VERSION_LENGTH);
}

void test_execute_command_get_device_config_selects_protocol_from_settings(void) {
    ResponsePacket response;
    memset(&response, 0, sizeof(response));

    strncpy(firmSettings.device_name, "MyDevice", DEVICE_NAME_LENGTH);
    firmSettings.frequency_hz = 123;

    firmSettings.uart_transfer_enabled = false;
    firmSettings.i2c_transfer_enabled = true;
    firmSettings.spi_transfer_enabled = true;

    uint32_t n = execute_command(CMDID_GET_DEVICE_CONFIG, NULL, 0, &response);

    TEST_ASSERT_EQUAL_UINT32(sizeof(response.device_config), n);
    TEST_ASSERT_EQUAL_STRING_LEN("MyDevice", response.device_config.name, DEVICE_NAME_LENGTH);
    TEST_ASSERT_EQUAL_UINT16(123, response.device_config.frequency);
    TEST_ASSERT_EQUAL(PROTOCOL_I2C, response.device_config.protocol);

    firmSettings.uart_transfer_enabled = true;
    memset(&response, 0, sizeof(response));
    (void)execute_command(CMDID_GET_DEVICE_CONFIG, NULL, 0, &response);
    TEST_ASSERT_EQUAL(PROTOCOL_UART, response.device_config.protocol);
}

void test_execute_command_reboot_calls_registered_reset(void) {
    void *ctx = (void*)0x1234;
    commands_register_system_reset(reset_cb, ctx);

    ResponsePacket response;
    memset(&response, 0, sizeof(response));

    uint32_t n = execute_command(CMDID_REBOOT, NULL, 0, &response);
    TEST_ASSERT_EQUAL_UINT32(0, n);
    TEST_ASSERT_EQUAL_INT(1, g_reset_called);
    TEST_ASSERT_EQUAL_PTR(ctx, g_reset_ctx_seen);
}
