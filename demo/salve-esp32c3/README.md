# Salve-ESP32C3 — ESP-NOW Sensor Node

ESP32-C3 based slave device for IMU data acquisition and wireless transmission.

## Hardware

- ESP32-C3
- MPU9250 9-axis IMU (accelerometer + gyroscope + AK8963 magnetometer)
- I2C bus (GPIO 5 SDA, GPIO 4 SCL)

## Architecture

```
main.cpp (state machine)
├── driver::I2cBus     — I2C bus
├── sensor::Mpu9250    — MPU9250 driver (accel, gyro, mag)
├── sensor::ImuEkf     — 13-state EKF orientation estimation
└── comm::EspNow       — ESP-NOW communication
```

## State Machine

```
INIT → WAIT_ID → WAIT_CALIBRATE → CALIBRATING → READY_TO_STREAM → STREAMING
                                             ↑                         |
                                             └─────── STOP ────────────┘
```

| State | Behavior |
|-------|----------|
| INIT | Init hardware, send READY broadcast |
| WAIT_ID | Wait for master to assign ID |
| WAIT_CALIBRATE | Wait for CALIBRATE command |
| CALIBRATING | Run EKF calibration (5 seconds), send CALIB_DONE when complete |
| READY_TO_STREAM | Ready, wait for START |
| STREAMING | Read IMU at 100Hz, run EKF, send ImuState |

## Data Packet (ImuState, 37 bytes)

```
#pragma pack(push, 1)
struct ImuState {
    float accel_x, accel_y, accel_z;  // m/s^2
    float gyro_x,  gyro_y,  gyro_z;   // deg/s
    float roll, pitch, yaw;           // deg
    uint8_t is_calibrated;            // 1 = done
};
#pragma pack(pop)
```

## Multi-Device Support

One firmware image supports multiple devices:
- No hardcoded MAC address or ID
- ID assigned dynamically by master
- Broadcasts READY to `FF:FF:FF:FF:FF:FF`

## Build

```bash
cd salve-esp32c3
idf.py build flash monitor
```

ESP-IDF v5.5.3 required. Managed component: `espressif/esp-dsp` v1.8.2.

## Usage

1. Flash the firmware to each slave device
2. Power on — slave auto-connects to master on channel 1
3. Calibration is triggered automatically by master
4. Data streaming starts when master sends START command
5. Pressing STOP on master stops streaming
