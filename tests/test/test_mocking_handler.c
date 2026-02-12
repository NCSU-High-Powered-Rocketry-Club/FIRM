#include "unity.h"
#include <string.h>
#include "mocking_handler.h"
#include "mock_w25q128jv_stubs.h"
#include "mock_hal_gpio.h"

static int g_write_called;
static FIRMSettings_t g_firm_seen;
static CalibrationSettings_t g_cal_seen;

static bool write_mock_settings(void *ctx,
                               FIRMSettings_t *firm_settings,
                               CalibrationSettings_t *calibration_settings) {
    (void)ctx;
    g_write_called++;
    g_firm_seen = *firm_settings;
    g_cal_seen = *calibration_settings;
    return true;
}

void setUp(void) {
    g_write_called = 0;
    memset(&g_firm_seen, 0, sizeof(g_firm_seen));
    memset(&g_cal_seen, 0, sizeof(g_cal_seen));
}

void tearDown(void) {
    // no-op
}

void test_process_mock_packet_copies_sensor_payload(void) {
    uint8_t received[6] = {1, 2, 3, 4, 5, 6};
    uint8_t out[6] = {0};

    MockPacketID id = process_mock_packet((uint16_t)MOCKID_BMP581, sizeof(received), received, out);

    TEST_ASSERT_EQUAL_UINT16(MOCKID_BMP581, id);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(received, out, sizeof(received));
}

void test_process_mock_packet_settings_returns_without_copy(void) {
    uint8_t received[4] = {9, 9, 9, 9};
    uint8_t out[4] = {1, 2, 3, 4};
    uint8_t expected[4] = {1, 2, 3, 4};

    MockPacketID id = process_mock_packet((uint16_t)MOCKID_SETTINGS, sizeof(received), received, out);

    TEST_ASSERT_EQUAL_UINT16(MOCKID_SETTINGS, id);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, out, 4);
}

void test_process_mock_settings_packet_parses_and_calls_write_fn(void) {
    const char *hdr = "FIRM LOG v1.1\n";
    const size_t hdr_len = strlen(hdr);

    FIRMSettings_t firm_in;
    CalibrationSettings_t cal_in;
    HeaderFields header_in;

    memset(&firm_in, 0, sizeof(firm_in));
    memset(&cal_in, 0, sizeof(cal_in));
    memset(&header_in, 0, sizeof(header_in));

    firm_in.device_uid = 0xAABBCCDDEEFF0011ULL;
    strncpy(firm_in.device_name, "MockDevice", sizeof(firm_in.device_name));
    firm_in.frequency_hz = 250;

    cal_in.icm45686_accel.offset_gs[0] = 1.25f;
    cal_in.icm45686_gyro.offset_dps[2] = -3.5f;

    header_in.temp_sf = 10.0f;
    header_in.pressure_sf = 20.0f;

    const uint32_t total_len = (uint32_t)(hdr_len + sizeof(FIRMSettings_t) + sizeof(CalibrationSettings_t) + sizeof(HeaderFields));
    uint8_t payload[512] = {0};
    TEST_ASSERT_TRUE(total_len <= sizeof(payload));

    uint32_t off = 0;
    memcpy(&payload[off], hdr, hdr_len);
    off += (uint32_t)hdr_len;
    memcpy(&payload[off], &firm_in, sizeof(firm_in));
    off += (uint32_t)sizeof(firm_in);
    memcpy(&payload[off], &cal_in, sizeof(cal_in));
    off += (uint32_t)sizeof(cal_in);
    memcpy(&payload[off], &header_in, sizeof(header_in));

    FIRMSettings_t firm_out;
    CalibrationSettings_t cal_out;
    HeaderFields header_out;

    memset(&firm_out, 0, sizeof(firm_out));
    memset(&cal_out, 0, sizeof(cal_out));
    memset(&header_out, 0, sizeof(header_out));

    bool ok = process_mock_settings_packet(payload,
                                          total_len,
                                          &firm_out,
                                          &cal_out,
                                          &header_out,
                                          write_mock_settings,
                                          NULL);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(1, g_write_called);

    TEST_ASSERT_EQUAL_UINT64(firm_in.device_uid, firm_out.device_uid);
    TEST_ASSERT_EQUAL_STRING_LEN(firm_in.device_name, firm_out.device_name, sizeof(firm_out.device_name));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, header_in.temp_sf, header_out.temp_sf);

    TEST_ASSERT_EQUAL_UINT64(firm_in.device_uid, g_firm_seen.device_uid);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cal_in.icm45686_accel.offset_gs[0], g_cal_seen.icm45686_accel.offset_gs[0]);
}
