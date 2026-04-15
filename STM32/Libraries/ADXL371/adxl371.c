#include "adxl371.h"

/**
 * @brief the SPI settings for the ADXL371 to use when accessing device registers
 */
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_channel;
  uint16_t cs_pin;
} SPISettings;

/**
 * @brief Starts up and resets the ADXL371, confirms the SPI read/write functionality is working
 *
 * @retval 0 if successful
 */
static int setup_device();

/**
 * @brief Reads data from the ADXL371 with SPI
 *
 * @param addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t *buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the ADXL371 with SPI
 *
 * @param addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data);

static const uint8_t who_am_i = 0x00;
static const uint8_t reset = 0x41;
static const uint8_t status = 0x04;
static const uint8_t timing = 0x3D;
static const uint8_t int1_map = 0x3B;
static const uint8_t power_ctl = 0x3F;
static const uint8_t measure = 0x3E;
static const uint8_t fifo_ctl = 0x3A;

static const uint8_t xdata_h = 0x08;

static const float accel_scale_factor = 4096.0F / 400.0F;

static SPISettings spiSettings;

/** Calibration and orientation values for converting raw sensor data to board-frame floats */
static float calibration_offsets[3] = {0};
static float calibration_matrix[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};

void set_spi_adxl(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin) {
  spiSettings.hspi = hspi;
  spiSettings.cs_channel = cs_channel;
  spiSettings.cs_pin = cs_pin;
}

int adxl371_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin) {

  if (hspi == NULL || cs_channel == NULL) {
    // Invalid spi handle or chip select pin
    return 1;
  }
  // set up the SPI settings
  set_spi_adxl(hspi, cs_channel, cs_pin);

  // Beginning ADXL371 initialization
  // sets up the accelerometer in SPI mode and ensures SPI is working
  if (setup_device())
    return 1;

  // do a soft-reset of the sensor's settings
  write_register(reset, 0x52);
  // verify correct setup again
  if (setup_device())
    return 1;

  // sets ODR to 1280
  write_register(timing, 0b00000000);

  // Accelerometer is fixed to +-200g

  // bit 3-1 in int1_map are reserved so I am masking. To avoid modifying those bits
  uint8_t value;
  // sets all non reserved bits to 0 thus disabeling interrups before config
  read_registers(int1_map, &value, 1);
  value = value & 0b00001110;
  write_register(int1_map, value);
  // Map Data Ready to Int1 and Active low
  read_registers(int1_map, &value, 1);
  value = value & 0b00001110;
  value = value | 0b10000001;
  write_register(int1_map, value);

  // Disable FIFO
  read_registers(fifo_ctl, &value, 1);
  value = value & 0b11111001;
  value = value | 0b00000000;
  write_register(fifo_ctl, value);

  // Turning off High Pass Filter and Low Pass Filter
  read_registers(power_ctl, &value, 1);
  value = value & 0b11110011;
  value = value | 0b00001100;
  write_register(power_ctl, value);

  // Configures measurement settings, 160hz, low noise operation
  // Turn off autosleep, link loop. Set low noise operation
  write_register(measure, 0b00001000);

  // delay for accel to get ready
  HAL_Delay(12);

  // set adxl to measure more
  read_registers(power_ctl, &value, 1);
  value = value & 0b01111100;
  value = value | 0b00000011;
  write_register(power_ctl, value);

  // ADXL371 startup successful
  return 0;
}

int adxl371_read_data(ADXL371RawData_t *packet) {
  // clear interrupt (pulls interrupt back up high) and verify new data is ready
  uint8_t data_ready = 0;
  read_registers(status, &data_ready, 1);
  if (data_ready & 0x01) { // bit 0 (LSB) will be 1 if new data is ready
    // Burst read xdata_h to zdata_l (0x08 to 0x0D) into packet
    read_registers(xdata_h, (uint8_t *)packet, 6);
    return 0;
  }
  return 1;
}

void adxl371_convert_and_calibrate(ADXL371RawData_t *raw, ADXL371BoardReading_t *out) {
  const float scale = 1.0F / accel_scale_factor;
  // extract acceleration as a 32 bit signed integer, which uses two's complement
  int32_t accel_binary_x = ((int32_t)(int8_t)raw->accX_H << 8 | (int32_t)raw->accX_L);

  int32_t accel_binary_y = ((int32_t)(int8_t)raw->accY_H << 8 | (int32_t)raw->accY_L);

  int32_t accel_binary_z = ((int32_t)(int8_t)raw->accZ_H << 8 | (int32_t)raw->accZ_L);

  // convert acceleration to g's and subtract calibration offset
  float accel_x_float = (float)accel_binary_x * scale - calibration_offsets[0];
  float accel_y_float = (float)accel_binary_y * scale - calibration_offsets[1];
  float accel_z_float = (float)accel_binary_z * scale - calibration_offsets[2];

  // 3x3 calibration matrix
  out->accel_x_g = accel_x_float * calibration_matrix[0] + accel_y_float * calibration_matrix[3] +
                   accel_z_float * calibration_matrix[6];
  out->accel_y_g = accel_x_float * calibration_matrix[1] + accel_y_float * calibration_matrix[4] +
                   accel_z_float * calibration_matrix[7];
  out->accel_z_g = accel_x_float * calibration_matrix[2] + accel_y_float * calibration_matrix[5] +
                   accel_z_float * calibration_matrix[8];
}

void adxl371_set_calibration(float offsets[3], float matrix[9]) {
  for (int i = 0; i < 3; i++) {
    calibration_offsets[i] = offsets[i];
  }
  for (int i = 0; i < 9; i++) {
    calibration_matrix[i] = matrix[i];
  }
}

float adxl371_get_accel_scale_factor(void) { return accel_scale_factor; }

static int setup_device() {
  uint8_t result = 0;

  // datasheet says 6ms to powerup, include some factor of safety (so we using 18!)
  HAL_Delay(18);

  // perform dummy read as required by datasheet
  HAL_StatusTypeDef hal_status = read_registers(who_am_i, &result, 1);
  if (hal_status) {
    return 1;
  }
  // give device enough time to switch to correct mode
  // this is a 5ms delay
  HAL_Delay(4);

  // verify chip ID read works
  read_registers(who_am_i, &result, 1);
  if (result != 0xAD) {
    // could not read chip ID
    return 1;
  }

  // write check
  read_registers(fifo_ctl, &result, 1); // fifo_ctl is default to 0x00

  if (result != 0x00) {
    // could not read fifo ctl register
    return 1;
  }

  write_register(fifo_ctl, 0x38);
  read_registers(fifo_ctl, &result, 1);
  if (result != 0x38) {
    // ADXL SPI Write test failed, wrote to register and did not read expected value back
  }
  write_register(fifo_ctl, 0x00); // Set it back to default (0x80)

  return 0;
}

static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t *buffer, size_t len) {
  addr = (addr << 1) | 0x01;
  return spi_read(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, buffer, len);
}

static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data) {
  addr <<= 1;
  return spi_write(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, data);
}

