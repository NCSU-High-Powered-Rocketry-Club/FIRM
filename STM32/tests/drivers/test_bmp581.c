#include "bmp581.h"
#include "utest.h"

UTEST(bmp581, convert_missing_data) {
  BMP581RawData_t in = {0};
  BMP581BoardReading_t out = {0};

  bmp581_convert(&in, &out);
  ASSERT_NEAR(0.0F, out.pressure_pa, 1e-6);
  ASSERT_NEAR(0.0F, out.temperature_celsius, 1e-6);
}

UTEST(bmp581, convert_normal_values) {
  // ~22.147C, ~100,000.016Pa
  BMP581RawData_t in = {0b10100011, 0b00100101, 0b00010110, 0b00000001, 0b10101000, 0b01100001};
  BMP581BoardReading_t out = {0};

  bmp581_convert(&in, &out);
  ASSERT_NEAR(6400001.0F / 64.0F, out.pressure_pa, 1e-6);
  ASSERT_NEAR(1451427.0F / 65536.0F, out.temperature_celsius, 1e-6);
}

UTEST(bmp581, convert_negative_values) {
  // ~-1.526C, -625Pa
  BMP581RawData_t in = {0b01100000, 0b01111001, 0b11111110, 0b11000000, 0b01100011, 0b11111111};
  BMP581BoardReading_t out = {0};

  bmp581_convert(&in, &out);
  ASSERT_NEAR(-40000.0F / 64.0F, out.pressure_pa, 1e-6);
  ASSERT_NEAR(-100000.0F / 65536.0F, out.temperature_celsius, 1e-6);
}

UTEST_MAIN()
