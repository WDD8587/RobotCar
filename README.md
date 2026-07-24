# RobotCar — Heterogeneous Embedded Robot Platform

Three-board architecture: **Raspberry Pi 4B + ESP32-S3 + STM32F103**

Same pattern as production vacuum robots (Ecovacs/Dreame/Roborock):
SoC runs vision/planning, MCU runs real-time motor control, third board handles sensors.

## Architecture

```
Raspberry Pi 4B (Linux, Python)    ESP32-S3 (FreeRTOS, C)        STM32F103 (Bare metal, C)
├─ SPI Master @ 8MHz               ├─ SPI Slave                  ├─ HC-SR04 Ultrasonic ×2
├─ OpenCV vision (15 fps)          ├─ PID speed control 1kHz     ├─ Bumper sensors ×2
├─ W/A/S/D keyboard control        ├─ Encoder PCNT               ├─ Battery ADC
└─ WiFi / MQTT                     ├─ MPU6050 IMU (I2C)          └─ UART TX → ESP32
     │                                  │                              │
     └──── SPI [cmd/telemetry] ────────┘    ── UART [sensor] ──────────┘
```

## Hardware Wiring

```
RPi (SPI Master)              ESP32-S3 (SPI Slave)             STM32F103
  GPIO10 (MOSI) ──────────── GPIO13 (MOSI)
  GPIO9  (MISO) ──────────── GPIO12 (MISO)
  GPIO11 (SCLK) ──────────── GPIO14 (SCLK)
  GPIO8  (CE0)  ──────────── GPIO15 (CS)

ESP32-S3 (Motor)               Motors & Encoders
  GPIO4  (PWMA_L) ────────── L298N IN1 → Motor L
  GPIO5  (PWMB_L) ────────── L298N IN2
  GPIO6  (PWMA_R) ────────── L298N IN3 → Motor R
  GPIO7  (PWMB_R) ────────── L298N IN4
  GPIO16,17 ──────────────── Encoder L (AB phases)
  GPIO18,19 ──────────────── Encoder R (AB phases)

ESP32-S3 (IMU)                MPU6050
  GPIO21 (SDA) ───────────── SDA
  GPIO22 (SCL) ───────────── SCL

ESP32-S3 (UART to STM32)     STM32F103 (UART)
  GPIO9  (TX)  ───────────── PA10 (RX)
  GPIO10 (RX)  ───────────── PA9  (TX)
                           
STM32F103 (Sensors)
  PA0 ─────────────────────── HC-SR04 #1 TRIG
  PA1 ─────────────────────── HC-SR04 #1 ECHO
  PA6 ─────────────────────── HC-SR04 #2 TRIG
  PA7 ─────────────────────── HC-SR04 #2 ECHO
  PB0 ─────────────────────── Bumper Left (active low)
  PB1 ─────────────────────── Bumper Right (active low)
  PA0 ─────────────────────── Battery voltage (10:1 divider)
```

## Build & Run

### ESP32-S3 (Motor Controller)

```bash
cd esp32_foc
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### STM32F103 (Sensor Hub)

```bash
cd stm32_sensors
make            # compiles with FWLib + FreeRTOS
make flash      # flash via ST-Link (st-flash write firmware.bin 0x08000000)
```

**Files:**
```
stm32_sensors/
├── Makefile                           # arm-none-eabi-gcc, references FWLib from a-meter project
├── stm32f103c8tx_flash.ld            # 64KB Flash, 20KB RAM
├── FreeRTOSConfig.h                   # 1000Hz tick, 20KB heap (from a-meter project)
├── Core/
│   ├── main.c                         # FWLib + FreeRTOS: sensor tasks, UART TX, ISRs
│   ├── robotcar_hw_config.h           # ALL pin maps, PWM params, encoder config
│   └── system_stm32f1xx.c             # HSE→PLL→72MHz clock init
├── startup/
│   └── startup_stm32f10x_hd.s         # Cortex-M3 vector table (from a-meter)
├── Protocol/
│   ├── protocol.h / .c                # UART framing + CRC32 (from a-meter)
└── HARDWARE/                          # sensor drivers (extending from a-meter)
```

**FWLib NOT modified** — referenced via `-I` path from original a-meter project at:
`C:/Users/EC/Desktop/a-meter-sweeping-robot-stm32-master/firmware/STM32F10x_FWLib/`

### Raspberry Pi 4B (Vision + Control)

```bash
cd raspberry_pi
pip install spidev opencv-python picamera
python3 main.py             # WASD control
python3 main.py --vision    # with tennis tracking
```

## Protocol

- **SPI RPi↔ESP32**: 32-byte frames, 8 MHz, Mode 3, CRC16-IBM
  - RPi→ESP: motor_cmd_t (target speeds, flags)
  - ESP→RPi: telemetry_t (encoders, IMU, ultrasound, battery)
- **UART ESP32↔STM32**: 16-byte frames, 115200, checksum
  - STM32→ESP: sensor_data_t (ultrasound ×3, bumper, battery mV)

## Interview Talking Points

1. "Three-board heterogeneous architecture — same pattern as Ecovacs X2 (Allwinner + STM32)"
2. "FreeRTOS on ESP32-S3, dual-core: Core 0 for motor PID at 1kHz, Core 1 for SPI/IMU"
3. "SPI protocol with CRC16 — designed the frame format from scratch"
4. "PID with anti-windup + feedforward on the encoder feedback loop"
5. "Bare-metal STM32F103 with register-level coding — no HAL, no libraries"
6. "Complementary filter for IMU yaw estimation"
