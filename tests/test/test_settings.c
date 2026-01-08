#include <string.h>
#include "unity.h"
#include "settings.h"
#include "mock_w25q128jv_stubs.h"
#include "mock_hal_gpio.h"

static uint8_t mock_flash_chip_sector[SETTINGS_FLASH_BLOCK_SIZE_BYTES];
static uint64_t mock_uid = 0x1122334455667788ULL;

// Writes to our fake flash chip
static void mock_w25q128jv_write_sector(const uint8_t* buf, uint32_t sector, uint32_t offset, uint32_t len, int call_num) {
    (void)call_num;
    // We should only be writing to the first sector
    TEST_ASSERT_EQUAL_UINT32(0, sector);
    TEST_ASSERT_EQUAL_UINT32(SETTINGS_FLASH_BLOCK_SIZE_BYTES, len);
    memcpy(mock_flash_chip_sector, buf, len);
}

// Reads from our fake flash chip
static void mock_w25q128jv_read_sector_Stub(uint8_t* buf, uint32_t sector, uint32_t offset, uint32_t len, int call_num) {
    (void)call_num;
    TEST_ASSERT_EQUAL_UINT32(0, sector);
    TEST_ASSERT_TRUE(offset + len <= SETTINGS_FLASH_BLOCK_SIZE_BYTES);
    memcpy(buf, mock_flash_chip_sector + offset, len);
}

// Mocks reading the UID
static void mock_w25q128jv_read_UID(uint8_t* out, uint32_t len, int call_num) {
    (void)call_num;
    TEST_ASSERT_EQUAL_UINT32(8, len);
    memcpy(out, &mock_uid, len);
}

void setUp(void) {
    memset(mock_flash_chip_sector, 0, sizeof(mock_flash_chip_sector));
    memset(&firmSettings, 0, sizeof(firmSettings));
    memset(&calibrationSettings, 0, sizeof(calibrationSettings));

    w25q128jv_read_UID_Stub(mock_w25q128jv_read_UID);
    w25q128jv_read_sector_Stub(mock_w25q128jv_read_sector_Stub);
    w25q128jv_write_sector_Stub(mock_w25q128jv_write_sector);
}

/**
 * Test that settings_init writes default settings when the write-check pin is low.
 */
void test_settings_init(void) {
    w25q128jv_set_spi_settings_Expect(NULL, NULL, 0);
    w25q128jv_init_ExpectAndReturn(1);
    HAL_GPIO_ReadPin_ExpectAndReturn(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // This is called twice, once when writing calibration defaults, once when writing FIRM defaults
    w25q128jv_erase_sector_Expect(0);
    w25q128jv_erase_sector_Expect(0);

    TEST_ASSERT_EQUAL_INT(0, settings_init(NULL, NULL, 0));

    // Check FIRM Settings
    TEST_ASSERT_EQUAL_UINT64(mock_uid, firmSettings.device_uid);
    TEST_ASSERT_EQUAL_STRING("FIRM Device", firmSettings.device_name);
    TEST_ASSERT_TRUE(firmSettings.usb_transfer_enabled);
    TEST_ASSERT_FALSE(firmSettings.uart_transfer_enabled);
    TEST_ASSERT_FALSE(firmSettings.i2c_transfer_enabled);
    TEST_ASSERT_FALSE(firmSettings.spi_transfer_enabled);
    TEST_ASSERT_EQUAL_STRING("v1.0.0", firmSettings.firmware_version);
    TEST_ASSERT_EQUAL_UINT16(100, firmSettings.frequency_hz);

    // Check Calibration Settings
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, calibrationSettings.icm45686_accel.scale_multiplier[0]); // (0,0)
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, calibrationSettings.icm45686_accel.scale_multiplier[1]); // (0,1)
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, calibrationSettings.icm45686_accel.offset_gs[0]);
    
}
