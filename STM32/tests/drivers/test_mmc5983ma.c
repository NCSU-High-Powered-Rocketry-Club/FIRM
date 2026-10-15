#include "mmc5983ma.h"
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

UTEST(mmc5983ma, convert_no_calibrate) {

  struct {
    MMC5983MARawData_t input;
    MMC5983MABoardReading_t exp;
  } cases[] = {{{0, 0, 0, 0, 0, 0, 0}, {-800.0F, -800.0F, -800.0F}},
               {{0x4E, 0x20, 0x00, 0x00, 0xD7, 0x3A, 0x04}, {-311.71875F, -800.0F, 545.172119F}}};

  char msg[32];
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    MMC5983MABoardReading_t out;
    snprintf(msg, sizeof(msg), "Failed on input: %zu", i);
    mmc5983ma_convert_and_calibrate(&cases[i].input, &out);
    ASSERT_FLOAT_CLOSE(cases[i].exp.magnetic_field_x_microteslas, out.magnetic_field_x_microteslas,
                       msg);
    ASSERT_FLOAT_CLOSE(cases[i].exp.magnetic_field_y_microteslas, out.magnetic_field_y_microteslas,
                       msg);
    ASSERT_FLOAT_CLOSE(cases[i].exp.magnetic_field_z_microteslas, out.magnetic_field_z_microteslas,
                       msg);
  }
}

UTEST(mmc5983ma, convert_calibrate) {
  float offsets[3] = {15.3F, -2.6F, -23.8F};
  float matrix[9] = {1.1F, -0.11F, 0.05F, -0.11F, 0.98F, 0.006F, 0.05F, 0.006F, 1.03F};
  mmc5983ma_set_calibration(offsets, matrix);

  struct {
    MMC5983MARawData_t input;
    MMC5983MABoardReading_t exp;
  } cases[] = {{{0, 0, 0, 0, 0, 0, 0}, {-847.926F, -696.4262F, -845.0354F}},
               {{0x4E, 0x20, 0x00, 0x00, 0xD7, 0x3A, 0x04}, {-243.558F, -742.0661F, 564.905945F}}};

  char msg[32];
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    MMC5983MABoardReading_t out;
    snprintf(msg, sizeof(msg), "Failed on input: %zu", i);
    mmc5983ma_convert_and_calibrate(&cases[i].input, &out);
    ASSERT_FLOAT_CLOSE(cases[i].exp.magnetic_field_x_microteslas, out.magnetic_field_x_microteslas,
                       msg);
    ASSERT_FLOAT_CLOSE(cases[i].exp.magnetic_field_y_microteslas, out.magnetic_field_y_microteslas,
                       msg);
    ASSERT_FLOAT_CLOSE(cases[i].exp.magnetic_field_z_microteslas, out.magnetic_field_z_microteslas,
                       msg);
  }
}

UTEST_MAIN()
