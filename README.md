# ICM20948

TDK ICM-20648 / ICM-20948 SPI 6-axis IMU driver module for XRobot.

This module initializes the IMU over SPI, samples accelerometer / gyroscope /
temperature data from the data-ready interrupt, publishes gyro and accel topics,
and stores gyroscope zero-offset calibration in `LibXR::Database`.

The current driver covers the 6-axis IMU path only. It does not initialize or
publish data from the ICM-20948 internal AK09916 magnetometer.

## Required Hardware

- `spi1`
- `IMU_CS`
- `IMU_INT`
- `ramfs`
- `database`

The SPI, CS, and INT names are constructor arguments, so projects may use other
hardware aliases if needed.

## Constructor Arguments

- `data_rate`: default `ICM20948::DataRate::DATA_RATE_1KHZ`
- `accl_range`: default `ICM20948::AcclRange::RANGE_16G`
- `gyro_range`: default `ICM20948::GyroRange::DPS_2000`
- `rotation`: sensor-frame to application-frame quaternion
- `gyro_topic_name`: default `"icm20948_gyro"`
- `accl_topic_name`: default `"icm20948_accl"`
- `task_stack_depth`: default `2048`
- `spi_name`: default `"spi1"`
- `cs_pin_name`: default `"IMU_CS"`
- `int_pin_name`: default `"IMU_INT"`

## Published Topics

- `gyro_topic_name`: `Eigen::Matrix<float, 3, 1>`, rad/s
- `accl_topic_name`: `Eigen::Matrix<float, 3, 1>`, m/s^2

## Shell Commands

The module registers `bin/icm20948` in `RamFS`.

- `bin/icm20948` or `bin/icm20948 status`: print chip ID, init state, and temperature
- `bin/icm20948 list_offset`: print current gyroscope calibration offset
- `bin/icm20948 cali`: collect still gyroscope data and save offset to `database`

## XRobot Configuration Example

```yaml
- id: imu
  name: ICM20948
  constructor_args:
    data_rate: ICM20948::DataRate::DATA_RATE_1KHZ
    accl_range: ICM20948::AcclRange::RANGE_16G
    gyro_range: ICM20948::GyroRange::DPS_2000
    rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
    gyro_topic_name: "icm20948_gyro"
    accl_topic_name: "icm20948_accl"
    task_stack_depth: 2048
    spi_name: "spi1"
    cs_pin_name: "IMU_CS"
    int_pin_name: "IMU_INT"
```

## Validation

Validated on OpenCR / STM32F746ZG with the onboard ICM-20648-compatible IMU:

- data-ready interrupt driven sampling
- gyroscope calibration load / save through `LibXR::Database`
- `bin/icm20948` command path through `RamFS`
- downstream AHRS topic consumption and VOFA+ quaternion output
