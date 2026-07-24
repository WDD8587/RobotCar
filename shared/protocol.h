/**
 * @file    protocol.h
 * @brief   Shared SPI + UART protocol for RobotCar (RPi + ESP32-S3 + STM32F103)
 *
 * Three-board heterogeneous architecture (production vacuum robot pattern):
 *
 *   Raspberry Pi 4B (SoC/Application)       ESP32-S3 (MCU/Real-Time)        STM32F103 (Sensor Hub)
 *   ├─ SPI Master @ 8MHz                     ├─ SPI Slave                   ├─ UART TX to ESP32
 *   ├─ OpenCV vision                         ├─ PID motor control            ├─ HC-SR04 ×2
 *   ├─ MQTT/WiFi                             ├─ Encoder reading (PCNT)      ├─ Bumper ×2
 *   └─ High-level planning                   ├─ IMU (MPU6050, I2C)          └─ Battery ADC
 *        │                                       │                               │
 *        └────── SPI [motor_cmd / telemetry] ────┘       ── UART [sensor] ──────┘
 *
 * SPI: 32-byte packets @ 8 MHz, CRC16-IBM
 * UART: 16-byte packets @ 115200, simple checksum
 */

#ifndef ROBOTCAR_PROTOCOL_H
#define ROBOTCAR_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * SPI Protocol (RPi ↔ ESP32-S3)
 * =========================================================================== */

#define SPI_FRAME_SIZE  32
#define SPI_HEADER_RPI  0xA5   /* RPi -> ESP32 command */
#define SPI_HEADER_ESP  0x5A   /* ESP32 -> RPi telemetry */

/* Motor command: RPi -> ESP32 (16 data bytes + CRC) */
typedef struct __attribute__((packed)) {
    uint8_t  header;        /* 0xA5 */
    uint8_t  seq;           /* sequence number */
    float    v_left;        /* target speed mm/s, left wheel */
    float    v_right;       /* target speed mm/s, right wheel */
    uint8_t  flags;         /* bit0=ESTOP, bit1=headlight */
    uint8_t  _pad[2];
    uint16_t crc;           /* CRC16-IBM over bytes 0..13 */
} motor_cmd_t;

/* Telemetry: ESP32 -> RPi (24 data bytes + CRC) */
typedef struct __attribute__((packed)) {
    uint8_t  header;        /* 0x5A */
    uint8_t  seq;
    int32_t  enc_left;      /* cumulative encoder pulses */
    int32_t  enc_right;
    int16_t  imu_yaw;       /* 0.01 deg */
    int16_t  imu_gyro_z;    /* 0.01 deg/s */
    uint16_t us_front_mm;   /* ultrasound distance mm */
    uint16_t us_rear_mm;
    uint8_t  bumper;        /* bit0=left, bit1=right */
    uint8_t  battery_pct;   /* 0-100% */
    uint16_t crc;
} telemetry_t;

_Static_assert(sizeof(motor_cmd_t) == 15, "motor_cmd_t size");
_Static_assert(sizeof(telemetry_t) == 22, "telemetry_t size");

/* ===========================================================================
 * UART Protocol (STM32F103 -> ESP32-S3)
 * =========================================================================== */

#define UART_FRAME_SIZE 16
#define UART_HEADER     0xCC

/* Sensor data: STM32 -> ESP32 (12 data bytes + checksum) */
typedef struct __attribute__((packed)) {
    uint8_t  header;        /* 0xCC */
    uint8_t  seq;
    uint16_t us1_mm;        /* HC-SR04 front distance (0 = timeout) */
    uint16_t us2_mm;        /* HC-SR04 rear distance */
    uint16_t us3_mm;        /* spare channel */
    uint8_t  bumper;        /* bit0=L bit1=R bit2=rear */
    uint16_t bat_mv;        /* battery mV from ADC */
    uint16_t checksum;      /* sum of bytes 0..13 */
    uint8_t  _pad[2];
} sensor_data_t;

/* ===========================================================================
 * CRC16-IBM
 * =========================================================================== */

static const uint16_t crc16_table[256] = {
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
    0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
    0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
    0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
    0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
    0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
    0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
    0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
    0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
    0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
    0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
    0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
    0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
    0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
    0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
    0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
    0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
    0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
    0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
    0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
    0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
    0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
    0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
    0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
    0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
    0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
    0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
    0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
    0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
    0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
    0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

static inline uint16_t crc16_ibm(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) crc = (crc >> 8) ^ crc16_table[(crc ^ *data++) & 0xFF];
    return crc;
}

/* ===========================================================================
 * Frame packing helpers
 * =========================================================================== */

static inline void pack_motor_cmd(uint8_t *frame, float vl, float vr, uint8_t flags, uint8_t seq) {
    frame[0] = SPI_HEADER_RPI;
    frame[1] = seq;
    *(uint32_t*)(frame + 2)  = *(uint32_t*)&vl;
    *(uint32_t*)(frame + 6)  = *(uint32_t*)&vr;
    frame[10] = flags;
    frame[11] = 0;
    frame[12] = 0;
    *(uint16_t*)(frame + 13) = crc16_ibm(frame, 13);
}

static inline int unpack_motor_cmd(const uint8_t *frame, motor_cmd_t *cmd) {
    if (frame[0] != SPI_HEADER_RPI) return -1;
    uint16_t calc = crc16_ibm(frame, sizeof(motor_cmd_t) - 2);
    if (calc != *(const uint16_t*)(frame + sizeof(motor_cmd_t) - 2)) return -2;
    memset(cmd, 0, sizeof(*cmd));
    cmd->header  = frame[0];
    cmd->seq     = frame[1];
    cmd->v_left  = *(const float*)(frame + 2);
    cmd->v_right = *(const float*)(frame + 6);
    cmd->flags   = frame[10];
    return 0;
}

static inline void pack_telemetry(uint8_t *frame, const telemetry_t *t, uint8_t seq) {
    memset(frame, 0, SPI_FRAME_SIZE);
    frame[0] = SPI_HEADER_ESP;
    frame[1] = seq;
    *(int32_t*)(frame + 2)  = t->enc_left;
    *(int32_t*)(frame + 6)  = t->enc_right;
    *(int16_t*)(frame + 10) = t->imu_yaw;
    *(int16_t*)(frame + 12) = t->imu_gyro_z;
    *(uint16_t*)(frame + 14) = t->us_front_mm;
    *(uint16_t*)(frame + 16) = t->us_rear_mm;
    frame[18] = t->bumper;
    frame[19] = t->battery_pct;
    *(uint16_t*)(frame + 20) = crc16_ibm(frame, 20);
}

static inline int unpack_telemetry(const uint8_t *frame, telemetry_t *t) {
    if (frame[0] != SPI_HEADER_ESP) return -1;
    uint16_t calc = crc16_ibm(frame, sizeof(telemetry_t) - 2);
    if (calc != *(const uint16_t*)(frame + sizeof(telemetry_t) - 2)) return -2;
    t->seq              = frame[1];
    t->enc_left         = *(const int32_t*)(frame + 2);
    t->enc_right        = *(const int32_t*)(frame + 6);
    t->imu_yaw          = *(const int16_t*)(frame + 10);
    t->imu_gyro_z       = *(const int16_t*)(frame + 12);
    t->us_front_mm      = *(const uint16_t*)(frame + 14);
    t->us_rear_mm       = *(const uint16_t*)(frame + 16);
    t->bumper           = frame[18];
    t->battery_pct      = frame[19];
    return 0;
}

/* ===========================================================================
 * UART sensor data packing
 * =========================================================================== */

static inline void pack_sensor_data(uint8_t *frame, const sensor_data_t *s) {
    frame[0] = UART_HEADER;
    frame[1] = s->seq;
    *(uint16_t*)(frame + 2) = s->us1_mm;
    *(uint16_t*)(frame + 4) = s->us2_mm;
    *(uint16_t*)(frame + 6) = s->us3_mm;
    frame[8] = s->bumper;
    *(uint16_t*)(frame + 9) = s->bat_mv;
    /* simple checksum: sum of all bytes */
    uint16_t sum = 0;
    for (int i = 0; i < 14; i++) sum += frame[i];
    *(uint16_t*)(frame + 14) = sum;
}

static inline bool unpack_sensor_data(const uint8_t *frame, sensor_data_t *s) {
    if (frame[0] != UART_HEADER) return false;
    uint16_t sum = 0;
    for (int i = 0; i < 14; i++) sum += frame[i];
    if (sum != *(const uint16_t*)(frame + 14)) return false;
    s->seq     = frame[1];
    s->us1_mm  = *(const uint16_t*)(frame + 2);
    s->us2_mm  = *(const uint16_t*)(frame + 4);
    s->us3_mm  = *(const uint16_t*)(frame + 6);
    s->bumper  = frame[8];
    s->bat_mv  = *(const uint16_t*)(frame + 9);
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* ROBOTCAR_PROTOCOL_H */
