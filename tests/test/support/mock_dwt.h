#pragma once
#include <stdint.h>

/* Minimal mock of the Cortex-M DWT register block used by preprocessor.c for unit tests.
 * We provide a static instance and a macro `DWT` so code that uses DWT->CYCCNT works
 * without pulling in CMSIS/HAL headers or defining HAL types.
 */
typedef struct {
    volatile uint32_t CYCCNT;
} DWT_Type;

/* single static instance used by tests; defined `static` so multiple translation units
 * including this header don't cause link conflicts.
 */
static DWT_Type _mock_DWT_instance = {0};

/* Provide DWT as a pointer expression like the real CMSIS symbol
 * Usage in production: DWT->CYCCNT
 */
#define DWT (&_mock_DWT_instance)
