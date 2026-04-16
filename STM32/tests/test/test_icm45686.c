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
  float acc_offsets[3] = {0.05F, -0.002F, 0.3F};
  float gyro_offsets[3] = {-0.3F, -1.0F, 0.05F};
  float accel_matrix[9] = {1.1F, -0.11F, 0.05F, -0.11F, 0.98F, 0.006F, 0.05F, 0.006F, 1.03F};
  float gyro_matrix[9] = {-0.003F, -0.96F, 0.1F, 0.99F, 0.003F, -0.14F, 0.14F, 0.1F, -1.11F};
  icm45686_set_accel_calibration(acc_offsets, accel_matrix);
  icm45686_set_gyro_calibration(gyro_offsets, gyro_matrix);

  struct {
    ICM45686RawData_t input;
    ICM45686BoardReading_t exp;
  } cases[] = {
    {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{-0.07022F, 0.00566F, -0.311488F, 0.9821F, -0.29F, -0.0545F}},
    {{0x07, 0x53, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xF2, 0x2D, 0x70, 0x00, 0xF0, 0x03, 0x00},{0.344947F, -0.3878567F, -33.17989F, 63.318F, 348.41405F, -3824.57675F}}
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