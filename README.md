# ICM20948

TDK ICM-20648 / ICM-20948 6 轴 IMU（SPI）驱动模块 / Driver Module for the TDK ICM-20648 / ICM-20948 6-axis IMU over SPI

## 1. 模块作用 / Purpose

ICM20948 驱动 IMU 的 6 轴部分：加速度计、陀螺仪与温度。

构造时，ICM20948 把 CS 拉高，将 INT 引脚配置为上升沿中断，并初始化芯片：SPI 模式 3、分频 DIV_32，器件复位，校验 `WHO_AM_I`（`0xEA` 为 ICM-20948，`0xE0` 为 ICM-20648），设置时钟与电源，写入采样率分频，写入量程并开启数字低通滤波，开启原始数据就绪中断（高电平有效，脉冲）。任一步失败时输出包含 `WHO_AM_I` 值的错误日志，并每 100 ms 重试，直到成功。

线程 `icm20948_thread`（REALTIME 优先级）等待数据就绪中断，用一次 burst 读出加速度、角速度和温度，换算后发布两个 Topic。加速度单位为 m/s²，乘以 `rotation`；角速度单位为 rad/s，先减去零偏再乘以 `rotation`。温度为 `raw / 333.87 + 21`（°C），由 `status` 命令显示。

陀螺仪零偏（rad/s）保存在 Database 的键 `icm20948_gyro_data` 中。

`OnMonitor()` 在数据含 NaN 或 Inf 时输出警告。

模块在 RamFS 的 `bin` 目录注册命令 `icm20948`：

- `bin/icm20948` 或 `bin/icm20948 status`：打印 `WHO_AM_I`、初始化状态和温度。
- `bin/icm20948 list_offset`：打印当前陀螺仪零偏。
- `bin/icm20948 cali`：陀螺仪零偏校准，期间设备保持静止。命令先等待 3 s，再对陀螺仪采集 60 s 求平均，并把零偏保存到 Database。

ICM20948 drives the 6-axis part of the IMU: accelerometer, gyroscope and temperature.

Upon construction, ICM20948 drives CS high, configures the INT pin as a rising-edge interrupt and initializes the chip: SPI mode 3 with prescaler DIV_32, device reset, `WHO_AM_I` check (`0xEA` for ICM-20948, `0xE0` for ICM-20648), clock and power setup, sample-rate dividers, ranges with the digital low-pass filter enabled, and the raw data-ready interrupt (active high, pulse). When any step fails, it logs an error containing the `WHO_AM_I` value and retries every 100 ms until it succeeds.

The thread `icm20948_thread` (REALTIME priority) waits for the data-ready interrupt, reads the acceleration, angular velocity and temperature in one burst, converts them and publishes both Topics. The acceleration unit is m/s², multiplied by `rotation`; the angular velocity unit is rad/s, with the zero offset subtracted before the multiplication by `rotation`. The temperature is `raw / 333.87 + 21` (°C) and is shown by the `status` command.

The gyroscope zero offset (rad/s) is stored in the Database under the key `icm20948_gyro_data`.

`OnMonitor()` logs a warning when the data contains NaN or Inf.

The Module registers the command `icm20948` in the RamFS `bin` directory:

- `bin/icm20948` or `bin/icm20948 status`: print `WHO_AM_I`, the initialization state and the temperature.
- `bin/icm20948 list_offset`: print the current gyroscope zero offset.
- `bin/icm20948 cali`: gyroscope zero-offset calibration, with the device held still. The command waits 3 s, averages the gyroscope for 60 s and saves the zero offset to the Database.

## 2. 构造接口 / Constructor

```cpp
ICM20948(LibXR::GPIO& cs_pin,
         LibXR::GPIO& int_pin,
         LibXR::SPI& spi,
         LibXR::Database& database,
         LibXR::RamFS& ramfs,
         const Param& param = {.data_rate = ICM20948::DataRate::DATA_RATE_281HZ,
                               .accl_range = ICM20948::AcclRange::RANGE_16G,
                               .gyro_range = ICM20948::GyroRange::DPS_2000,
                               .rotation = {1.0f, 0.0f, 0.0f, 0.0f},
                               .gyro_topic_name = "icm20948_gyro",
                               .accl_topic_name = "icm20948_accl",
                               .task_stack_depth = 2048});
```

依赖：

- `cs_pin`：片选 GPIO（输出，低有效），由模块控制，取自 BSP 的硬件注册（`XR_REGISTER`）。
- `int_pin`：数据就绪中断 GPIO，模块将其配置为上升沿中断。
- `spi`：连接 IMU 的 `LibXR::SPI`，模块将其设置为模式 3、分频 DIV_32。
- `database`：保存陀螺仪零偏的 `LibXR::Database`。
- `ramfs`：接收 `bin/icm20948` 命令的 `LibXR::RamFS`。

配置参数（`Param`）：

- `data_rate`：采样率设置，默认 `DATA_RATE_281HZ`。枚举值减 1 写入陀螺仪与加速度计的采样率分频寄存器（`DATA_RATE_1125HZ` 为 0，`DATA_RATE_281HZ` 为 3，`DATA_RATE_141HZ` 为 7，`DATA_RATE_70HZ` 为 15，`DATA_RATE_35HZ` 为 31）。数字低通滤波开启时，输出速率为 1.125 kHz / (1 + 分频值)，枚举名中的频率为该速率取整。
- `accl_range`：加速度计量程，默认 `RANGE_16G`；可选 `RANGE_2G`、`RANGE_4G`、`RANGE_8G`、`RANGE_16G`。
- `gyro_range`：陀螺仪量程，默认 `DPS_2000`；可选 `DPS_250`、`DPS_500`、`DPS_1000`、`DPS_2000`。
- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`，默认单位四元数。
- `gyro_topic_name`、`accl_topic_name`：发布的 Topic 名称，默认 `"icm20948_gyro"`、`"icm20948_accl"`。
- `task_stack_depth`：采样线程栈深，默认 2048。

Dependencies:

- `cs_pin`: chip-select GPIO (output, active low), driven by the Module and taken from the BSP's Registration (`XR_REGISTER`).
- `int_pin`: data-ready interrupt GPIO; the Module configures it as a rising-edge interrupt.
- `spi`: the `LibXR::SPI` connected to the IMU; the Module sets mode 3 and DIV_32.
- `database`: the `LibXR::Database` that stores the gyroscope zero offset.
- `ramfs`: the `LibXR::RamFS` that receives the `bin/icm20948` command.

Configuration parameters (`Param`):

- `data_rate`: sample-rate setting, default `DATA_RATE_281HZ`. The enum value minus 1 is written to the sample-rate divider registers of the gyroscope and the accelerometer (`DATA_RATE_1125HZ` 0, `DATA_RATE_281HZ` 3, `DATA_RATE_141HZ` 7, `DATA_RATE_70HZ` 15, `DATA_RATE_35HZ` 31). With the digital low-pass filter enabled, the output rate is 1.125 kHz / (1 + divider); the frequency in each enumerator name is that rate, rounded.
- `accl_range`: accelerometer range, default `RANGE_16G`; options are `RANGE_2G`, `RANGE_4G`, `RANGE_8G`, `RANGE_16G`.
- `gyro_range`: gyroscope range, default `DPS_2000`; options are `DPS_250`, `DPS_500`, `DPS_1000`, `DPS_2000`.
- `rotation`: quaternion `{w, x, y, z}` from the sensor frame to the application frame, default identity.
- `gyro_topic_name`, `accl_topic_name`: names of the published Topics, default `"icm20948_gyro"` and `"icm20948_accl"`.
- `task_stack_depth`: stack depth of the sampling thread, default 2048.

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `gyro_topic_name`（默认 `icm20948_gyro`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 rad/s，已去零偏并旋转 |
| `accl_topic_name`（默认 `icm20948_accl`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 m/s²，已旋转 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `gyro_topic_name` (default `icm20948_gyro`) | Publish | `Eigen::Matrix<float, 3, 1>` | Angular velocity in rad/s, zero offset removed and rotated |
| `accl_topic_name` (default `icm20948_accl`) | Publish | `Eigen::Matrix<float, 3, 1>` | Acceleration in m/s², rotated |

## 4. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/ICM20948` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/ICM20948`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/ICM20948
    id: imu
    args:
      - cs_pin: IMU_CS
      - int_pin: IMU_INT
      - spi: spi1
      - database: database
      - ramfs: ramfs
      - param:
          data_rate: ICM20948::DataRate::DATA_RATE_281HZ
          accl_range: ICM20948::AcclRange::RANGE_16G
          gyro_range: ICM20948::GyroRange::DPS_2000
          rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
          gyro_topic_name: "icm20948_gyro"
          accl_topic_name: "icm20948_accl"
          task_stack_depth: 2048
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片通过 SPI 连接的 ICM-20648 或 ICM-20948，带片选 GPIO 与数据就绪中断 GPIO；SPI、GPIO、Database 与 RamFS 由 BSP 通过 `XR_REGISTER` 注册。

Dependencies: LibXR.

Hardware: one ICM-20648 or ICM-20948 connected over SPI, with a chip-select GPIO and a data-ready interrupt GPIO; the SPI, GPIOs, Database and RamFS are registered by the BSP with `XR_REGISTER`.
