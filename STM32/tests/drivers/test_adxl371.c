#include "adxl371.h"
#include "utest.h"
#include <math.h>
#include <stdio.h>

static float float_tolerance(float expected) {
  const float abs_tol = 1e-4F;
  const float rel_tol = 1e-7F;
  return fmaxf(abs_tol, rel_tol * fmaxf(1.0F, fabsf(expected)));
}

#define ASSERT_FLOAT_CLOSE(expected, actual, msg)                                                  \
  ASSERT_NEAR_MSG(expected, actual, float_tolerance(expected), msg)

UTEST(adxl371, convert_no_calibrate) {
  float offsets[3] = {0.0F, 0.0F, 0.0F};
  float matrix[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  adxl371_set_calibration(offsets, matrix);

  struct {
    ADXL371RawData_t input;
    ADXL371BoardReading_t exp;
  } cases[] = {{{0, 0, 0, 0, 0, 0}, {0.0F, 0.0F, 0.0F}},
               {{0x4D, 0x20, 0xC0, 0x00, 0x7F, 0xF0}, {120.5078125F, -100.0F, 199.90234375F}}};

  char msg[32];
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    ADXL371BoardReading_t out;
    snprintf(msg, sizeof(msg), "Failed on input: %zu", i);
    adxl371_convert_and_calibrate(&cases[i].input, &out);
    ASSERT_FLOAT_CLOSE(cases[i].exp.accel_x_g, out.accel_x_g, msg);
    ASSERT_FLOAT_CLOSE(cases[i].exp.accel_y_g, out.accel_y_g, msg);
    ASSERT_FLOAT_CLOSE(cases[i].exp.accel_z_g, out.accel_z_g, msg);
  }
}

UTEST(adxl371, convert_calibrate) {
  float offsets[3] = {0.05F, -0.002F, 0.3F};
  float matrix[9] = {1.1F, -0.11F, 0.05F, -0.11F, 0.98F, 0.006F, 0.05F, 0.006F, 1.03F};
  adxl371_set_calibration(offsets, matrix);

  struct {
    ADXL371RawData_t input;
    ADXL371BoardReading_t exp;
  } cases[] = {
      {{0, 0, 0, 0, 0, 0}, {-0.07022F, 0.00566F, -0.311488F}},
      {{0x4D, 0x20, 0xC0, 0x00, 0x7F, 0xF0}, {153.48349094F, -110.05078531F, 211.01331669F}}};

  char msg[32];
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    ADXL371BoardReading_t out;
    snprintf(msg, sizeof(msg), "Failed on input: %zu", i);
    adxl371_convert_and_calibrate(&cases[i].input, &out);
    ASSERT_FLOAT_CLOSE(cases[i].exp.accel_x_g, out.accel_x_g, msg);
    ASSERT_FLOAT_CLOSE(cases[i].exp.accel_y_g, out.accel_y_g, msg);
    ASSERT_FLOAT_CLOSE(cases[i].exp.accel_z_g, out.accel_z_g, msg);
  }
}

UTEST_MAIN()
