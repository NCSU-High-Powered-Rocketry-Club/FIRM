#include "unity.h"
#include "commands.h"
#include "settings.h"
#include "utils.h"
#include "stm32_hal_stubs.h"

// `commands.c` pulls in `settings.c`, which depends on flash + GPIO.
// Include the mocks so the unit-test link step succeeds.
#include "mock_w25q128jv_stubs.h"
#include "mock_hal_gpio.h"


void setUp(void) {
    // no-op
}
