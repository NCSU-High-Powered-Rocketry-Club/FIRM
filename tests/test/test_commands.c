#include "unity.h"
#include "commands.h"
#include "utils.h"
#include "stm32_hal_stubs.h"
#include "mock_settings.h"

FIRMSettings_t firmSettings;
CalibrationSettings_t calibrationSettings;

void setUp(void) {
    FIRMSettings_t dummy;
    settings_write_firm_settings_ExpectAndReturn(&dummy, true);
    // or you could do a stub
}

void test_commands_update_parser(void) {
    uint8_t received_bytes[COMMAND_READ_CHUNK_SIZE_BYTES];
    CommandsStreamParser_t command_parser = { .len = 0 };
}
