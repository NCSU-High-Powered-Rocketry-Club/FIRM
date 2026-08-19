#include "error_state_kalman_filter.h"
#include "eskf_config.h"
#include "settings_manager_host.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define INPUT_VERSION 1U
#define OUTPUT_VERSION 1U
#define INPUT_FLOAT_COUNT 14U
#define OUTPUT_FLOAT_COUNT 15U

static const unsigned char INPUT_MAGIC[8] = {'F', 'I', 'R', 'M', 'I', 'N', '0', '1'};
static const unsigned char OUTPUT_MAGIC[8] = {'F', 'I', 'R', 'M', 'O', 'U', 'T', '1'};

#pragma pack(push, 1)
typedef struct {
  double timestamp;
  float values[INPUT_FLOAT_COUNT];
} InputRecord;

typedef struct {
  double timestamp;
  float values[OUTPUT_FLOAT_COUNT];
} OutputRecord;
#pragma pack(pop)

enum InputField {
  INPUT_PRESSURE = 0,
  INPUT_TEMPERATURE,
  INPUT_ACCEL_X,
  INPUT_ACCEL_Y,
  INPUT_ACCEL_Z,
  INPUT_GYRO_X,
  INPUT_GYRO_Y,
  INPUT_GYRO_Z,
  INPUT_MAG_X,
  INPUT_MAG_Y,
  INPUT_MAG_Z,
  INPUT_HIGH_G_X,
  INPUT_HIGH_G_Y,
  INPUT_HIGH_G_Z,
};

enum OutputField {
  OUTPUT_DT = 0,
  OUTPUT_POSITION_Z,
  OUTPUT_VELOCITY_Z,
  OUTPUT_QUAT_W,
  OUTPUT_QUAT_X,
  OUTPUT_QUAT_Y,
  OUTPUT_QUAT_Z,
  OUTPUT_P_POSITION_Z,
  OUTPUT_P_VELOCITY_Z,
  OUTPUT_P_THETA_X,
  OUTPUT_P_THETA_Y,
  OUTPUT_P_THETA_Z,
  OUTPUT_RAW_BARO_ALTITUDE,
  OUTPUT_PRESSURE_COUPLING,
  OUTPUT_QUAT_NORM,
};

static int read_exact(FILE *file, void *destination, size_t size) {
  return fread(destination, 1U, size, file) == size ? 0 : -1;
}

static int write_exact(FILE *file, const void *source, size_t size) {
  return fwrite(source, 1U, size, file) == size ? 0 : -1;
}

static int read_header(FILE *input, uint64_t *count, double *initialization_seconds,
                       char firmware_version[9]) {
  unsigned char magic[8];
  uint32_t version = 0U;
  uint32_t record_size = 0U;
  char firmware[8];

  if (read_exact(input, magic, sizeof(magic)) != 0 || memcmp(magic, INPUT_MAGIC, 8U) != 0 ||
      read_exact(input, &version, sizeof(version)) != 0 ||
      read_exact(input, &record_size, sizeof(record_size)) != 0 ||
      read_exact(input, count, sizeof(*count)) != 0 ||
      read_exact(input, initialization_seconds, sizeof(*initialization_seconds)) != 0 ||
      read_exact(input, firmware, sizeof(firmware)) != 0) {
    return -1;
  }
  if (version != INPUT_VERSION || record_size != sizeof(InputRecord)) {
    return -1;
  }
  memcpy(firmware_version, firmware, 8U);
  firmware_version[8] = '\0';
  return 0;
}

static int write_header(FILE *output, uint64_t count) {
  const uint32_t version = OUTPUT_VERSION;
  const uint32_t record_size = (uint32_t)sizeof(OutputRecord);
  return write_exact(output, OUTPUT_MAGIC, sizeof(OUTPUT_MAGIC)) ||
                 write_exact(output, &version, sizeof(version)) ||
                 write_exact(output, &record_size, sizeof(record_size)) ||
                 write_exact(output, &count, sizeof(count))
             ? -1
             : 0;
}

static float pressure_to_altitude(float pressure, float initial_pressure) {
  if (!(pressure > 0.0F) || !(initial_pressure > 0.0F)) {
    return NAN;
  }
  return PRESSURE_ALTITUDE_CONST *
         (1.0F - powf(pressure / initial_pressure, 1.0F / PRESSURE_EXPONENT));
}

static int state_is_finite(const ESKF *eskf) {
  for (size_t i = 0U; i < ESKF_NOMINAL_DIM; ++i) {
    if (!isfinite(eskf->x_nom[i])) {
      return 0;
    }
  }
  return 1;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "usage: %s INPUT.bin OUTPUT.bin\n", argv[0]);
    return 2;
  }

  FILE *input = fopen(argv[1], "rb");
  if (input == NULL) {
    fprintf(stderr, "cannot open input '%s': %s\n", argv[1], strerror(errno));
    return 2;
  }

  uint64_t input_count = 0U;
  double initialization_seconds = 0.0;
  char firmware_version[9];
  if (read_header(input, &input_count, &initialization_seconds, firmware_version) != 0) {
    fprintf(stderr, "invalid ESKF replay input header\n");
    fclose(input);
    return 2;
  }
  host_set_firmware_version(firmware_version);

  FILE *output = fopen(argv[2], "wb+");
  if (output == NULL) {
    fprintf(stderr, "cannot open output '%s': %s\n", argv[2], strerror(errno));
    fclose(input);
    return 2;
  }
  if (write_header(output, 0U) != 0) {
    fprintf(stderr, "cannot write output header\n");
    fclose(input);
    fclose(output);
    return 2;
  }

  InputRecord record;
  if (input_count == 0U || read_exact(input, &record, sizeof(record)) != 0) {
    fprintf(stderr, "input contains no replay records\n");
    fclose(input);
    fclose(output);
    return 2;
  }

  const double initialization_end = record.timestamp + initialization_seconds;
  uint64_t input_index = 1U;
  uint64_t accumulation_count = 0U;
  InputRecord last_initial = record;

  while (1) {
    if (record.timestamp > initialization_end) {
      break;
    }
    eskf_accumulate(record.values[INPUT_PRESSURE], &record.values[INPUT_ACCEL_X],
                    &record.values[INPUT_MAG_X]);
    last_initial = record;
    ++accumulation_count;
    if (input_index >= input_count || read_exact(input, &record, sizeof(record)) != 0) {
      break;
    }
    ++input_index;
  }

  if (accumulation_count == 0U ||
      (input_index >= input_count && record.timestamp <= initialization_end)) {
    fprintf(stderr, "dataset does not extend beyond the %.3f second initialization window\n",
            initialization_seconds);
    fclose(input);
    fclose(output);
    return 2;
  }

  ESKF eskf;
  if (eskf_init(&eskf) != 0) {
    fprintf(stderr, "ESKF initialization failed\n");
    fclose(input);
    fclose(output);
    return 2;
  }

  float last_time = (float)last_initial.timestamp - 0.005F;
  uint64_t output_count = 0U;
  uint64_t nonfinite_count = 0U;

  while (1) {
    const float current_time = (float)record.timestamp;
    const float dt = current_time - last_time;
    if (dt > 1e-6F) {
      const float control[ESKF_CONTROL_DIM] = {
          record.values[INPUT_ACCEL_X], record.values[INPUT_ACCEL_Y], record.values[INPUT_ACCEL_Z],
          record.values[INPUT_GYRO_X],  record.values[INPUT_GYRO_Y],  record.values[INPUT_GYRO_Z],
      };
      const float measurement[ESKF_MEASUREMENT_DIM] = {
          record.values[INPUT_PRESSURE], record.values[INPUT_MAG_X], record.values[INPUT_MAG_Y],
          record.values[INPUT_MAG_Z],
      };
      last_time = current_time;
      eskf_predict(&eskf, control, dt);
      eskf_set_measurement(&eskf, measurement);
      eskf_update(&eskf);

      OutputRecord result = {.timestamp = record.timestamp};
      result.values[OUTPUT_DT] = dt;
      memcpy(&result.values[OUTPUT_POSITION_Z], eskf.x_nom, sizeof(eskf.x_nom));
      for (size_t i = 0U; i < ESKF_ERROR_DIM; ++i) {
        result.values[OUTPUT_P_POSITION_Z + i] = eskf.P[i * ESKF_ERROR_DIM + i];
      }
      result.values[OUTPUT_RAW_BARO_ALTITUDE] =
          pressure_to_altitude(record.values[INPUT_PRESSURE], eskf.initial_pressure);
      result.values[OUTPUT_PRESSURE_COUPLING] = eskf.pressure_coupling;
      const float *q = &eskf.x_nom[ESKF_QUAT_W];
      result.values[OUTPUT_QUAT_NORM] =
          sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
      if (!state_is_finite(&eskf)) {
        ++nonfinite_count;
      }
      if (write_exact(output, &result, sizeof(result)) != 0) {
        fprintf(stderr, "failed while writing replay output\n");
        fclose(input);
        fclose(output);
        return 2;
      }
      ++output_count;
    }

    if (input_index >= input_count || read_exact(input, &record, sizeof(record)) != 0) {
      break;
    }
    ++input_index;
  }

  if (fseek(output, 0L, SEEK_SET) != 0 || write_header(output, output_count) != 0) {
    fprintf(stderr, "could not finalize output header\n");
    fclose(input);
    fclose(output);
    return 2;
  }

  fclose(input);
  fclose(output);
  fprintf(stderr,
          "processed %llu rows after %llu initialization rows; %llu non-finite states\n",
          (unsigned long long)output_count, (unsigned long long)accumulation_count,
          (unsigned long long)nonfinite_count);
  return 0;
}
