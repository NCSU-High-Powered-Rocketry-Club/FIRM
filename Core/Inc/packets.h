#include <stdint.h>

typedef struct {
    uint16_t relative_timestamp;
    // Units: G's
    float acc_x;
    float acc_y;
    float acc_z;
    // Units: rad/s
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMUPacket_t;

typedef struct {
    uint16_t relative_timestamp;
    // Units: degrees Celsius
    float temperature;
    // Units: Pascals
    float pressure;
} BMPPacket_t;

typedef struct {
    uint16_t relative_timestamp;
    float mag_x;
    float mag_y;
    float mag_z;
} MMCPacket_t;
