# ICM20948

XRobot Module for the TDK ICM-20648 / ICM-20948 6-axis IMU over SPI.

The driver covers the 6-axis IMU path only. It does not initialize or publish data
from the ICM-20948 internal AK09916 magnetometer.

## Behaviour

- The constructor drives CS high, configures the INT pin as rising-edge interrupt
  and initializes the chip: SPI mode 3 with prescaler DIV_32, device reset,
  `WHO_AM_I` check (`0xEA` ICM-20948 or `0xE0` ICM-20648), clock and power setup,
  sample-rate dividers, ranges with the digital low-pass filter enabled, and the
  raw data-ready interrupt (active high, pulse). A failed step logs the `WHO_AM_I`
  value and retries every 100 ms until it succeeds.
- The `icm20948_thread` thread (REALTIME priority) waits for the data-ready
  interrupt, burst-reads accelerometer, gyroscope and temperature, converts them and
  publishes both topics.
- Acceleration is in m/s² and is multiplied by `rotation`. Angular rate is in rad/s;
  the gyroscope offset is subtracted before the rotation. Temperature is
  `raw / 333.87 + 21` °C (shown by the `status` command).
- The gyroscope offset (rad/s) is stored in the Database key `icm20948_gyro_data`.
- `OnMonitor()` logs a warning when the data contain NaN or Inf.

## Topics

| Topic | Type | Content |
| --- | --- | --- |
| `gyro_topic_name` (default `icm20948_gyro`) | `Eigen::Matrix<float, 3, 1>` | Angular rate, rad/s, offset removed and rotated |
| `accl_topic_name` (default `icm20948_accl`) | `Eigen::Matrix<float, 3, 1>` | Acceleration, m/s², rotated |

## RamFS command

The Module registers `icm20948` in the RamFS `bin` directory.

- `bin/icm20948` or `bin/icm20948 status`: print `WHO_AM_I`, init state and
  temperature.
- `bin/icm20948 list_offset`: print the current gyroscope offset.
- `bin/icm20948 cali`: gyroscope offset calibration. Keep the device still; the
  command waits 3 s, averages the gyroscope for 60 s and saves the offset to the
  Database.

## Dependencies

No other Modules; LibXR only.

## Constructor

```cpp
ICM20948(LibXR::GPIO& cs_pin,
         LibXR::GPIO& int_pin,
         LibXR::SPI& spi,
         LibXR::Database& database,
         LibXR::RamFS& ramfs,
         const Param& param = {.data_rate = ICM20948::DataRate::DATA_RATE_1KHZ,
                               .accl_range = ICM20948::AcclRange::RANGE_16G,
                               .gyro_range = ICM20948::GyroRange::DPS_2000,
                               .rotation = {1.0f, 0.0f, 0.0f, 0.0f},
                               .gyro_topic_name = "icm20948_gyro",
                               .accl_topic_name = "icm20948_accl",
                               .task_stack_depth = 2048});
```

Dependencies:

- `cs_pin`: chip-select GPIO (output, active low), driven by the Module.
- `int_pin`: data-ready interrupt GPIO, configured as rising-edge interrupt.
- `spi`: `LibXR::SPI` connected to the IMU; the Module sets mode 3 and DIV_32.
- `database`: `LibXR::Database` that stores the gyroscope offset.
- `ramfs`: `LibXR::RamFS` that receives the `bin/icm20948` command.

Configuration (`Param` fields):

- `data_rate`: sample-rate setting, default `DATA_RATE_1KHZ`. The enum value minus 1
  is written to both gyroscope and accelerometer sample-rate dividers
  (`DATA_RATE_4KHZ` 0, `DATA_RATE_1KHZ` 3, `DATA_RATE_500HZ` 7, `DATA_RATE_250HZ` 15,
  `DATA_RATE_125HZ` 31). Per the datasheet the output rate with the low-pass filter
  enabled is 1.125 kHz / (1 + divider), so the enum names are not the actual rates.
- `accl_range`: accelerometer range, default `RANGE_16G` (`RANGE_2G`, `RANGE_4G`,
  `RANGE_8G`, `RANGE_16G`).
- `gyro_range`: gyroscope range, default `DPS_2000` (`DPS_250`, `DPS_500`,
  `DPS_1000`, `DPS_2000`).
- `rotation`: quaternion `{w, x, y, z}` from sensor frame to application frame,
  default identity.
- `gyro_topic_name` / `accl_topic_name`: topic names, defaults `"icm20948_gyro"` /
  `"icm20948_accl"`.
- `task_stack_depth`: stack depth of the sampling thread, default 2048.

## Use

```sh
xrobot module add xrobot-org/ICM20948
xrobot setup
xrobot instance add xrobot-org/ICM20948
```

`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of objects
the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/ICM20948
    id: icm20948_0
    args:
      - cs_pin: imu_cs
      - int_pin: imu_int
      - spi: spi1
      - database: database
      - ramfs: ramfs
      - param:
          data_rate: ICM20948::DataRate::DATA_RATE_1KHZ
          accl_range: ICM20948::AcclRange::RANGE_16G
          gyro_range: ICM20948::GyroRange::DPS_2000
          rotation:
            - 1.0f
            - 0.0f
            - 0.0f
            - 0.0f
          gyro_topic_name: '"icm20948_gyro"'
          accl_topic_name: '"icm20948_accl"'
          task_stack_depth: '2048'
```

BSP side:

```cpp
XR_REGISTER(imu_cs, LibXR::GPIO);
XR_REGISTER(imu_int, LibXR::GPIO);
XR_REGISTER(spi1, LibXR::SPI);
XR_REGISTER(database, LibXR::Database);
XR_REGISTER(ramfs, LibXR::RamFS);
```

Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/ICM20948` in a BSP, prints the manifest and
the current constructor.

## Hardware notes

An earlier version of this driver was validated on OpenCR (STM32F746ZG) with the
onboard ICM-20648-compatible IMU: interrupt-driven sampling, calibration load/save
through the Database, the `bin/icm20948` command and downstream AHRS use.
