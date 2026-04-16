#include "unity.h"
#include <stdio.h>
#include <math.h>
#include "icm45686.h"



void setUp() {}
void tearDown() {}

static void assert_float_close_message(float expected, float actual, const char *msg) {
  const float abs_tol = 1e-4F;
  const float rel_tol = 1e-7F;
  const float tol = fmaxf(abs_tol, rel_tol * fmaxf(1.0F, fabsf(expected)));
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tol, expected, actual, msg);
}

void test_icm45686_convert_no_calibrate(void) {
  
  struct {
    ICM45686RawData_t input;
    ICM45686BoardReading_t exp;
  } cases[] = {
    {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0}},
    {{0x07, 0x53, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xF2, 0x2D, 0x70, 0x00, 0xF0, 0x03, 0x00},{1.83197F, 0.0F, -32.0F, 0.0F, -431.98391F, 3500.0F}}
  };

  char msg[32];
  for (int i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
    ICM45686BoardReading_t out;
    snprintf(msg, sizeof(msg), "Failed on input: %d", i);
    icm45686_convert_and_calibrate(&cases[i].input, &out);
    assert_float_close_message(cases[i].exp.accel_x_g, out.accel_x_g, msg);
    assert_float_close_message(cases[i].exp.accel_y_g, out.accel_y_g, msg);
    assert_float_close_message(cases[i].exp.accel_z_g, out.accel_z_g, msg);
    assert_float_close_message(cases[i].exp.gyro_x_dps, out.gyro_x_dps, msg);
    assert_float_close_message(cases[i].exp.gyro_y_dps, out.gyro_y_dps, msg);
    assert_float_close_message(cases[i].exp.gyro_z_dps, out.gyro_z_dps, msg);
  }
}

void test_icm45686_convert_calibrate(void) {
  
  struct {
    ICM45686RawData_t input;
    ICM45686BoardReading_t exp;
  } cases[] = {
    {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0}},
    {{0x07, 0x53, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xF2, 0x2D, 0x70, 0x00, 0xF0, 0x03, 0x00},{1.83197F, 0.0F, -32.0F, 0.0F, -431.98391F, 3500.0F}}
  };

  char msg[32];
  for (int i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
    ICM45686BoardReading_t out;
    snprintf(msg, sizeof(msg), "Failed on input: %d", i);
    icm45686_convert_and_calibrate(&cases[i].input, &out);
    assert_float_close_message(cases[i].exp.accel_x_g, out.accel_x_g, msg);
    assert_float_close_message(cases[i].exp.accel_y_g, out.accel_y_g, msg);
    assert_float_close_message(cases[i].exp.accel_z_g, out.accel_z_g, msg);
    assert_float_close_message(cases[i].exp.gyro_x_dps, out.gyro_x_dps, msg);
    assert_float_close_message(cases[i].exp.gyro_y_dps, out.gyro_y_dps, msg);
    assert_float_close_message(cases[i].exp.gyro_z_dps, out.gyro_z_dps, msg);
  }
}