#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: TDK ICM-20648 / ICM-20948 6 轴 IMU（SPI）驱动模块 / Driver Module for the TDK ICM-20648 / ICM-20948 6-axis IMU over SPI
depends: []
=== END MANIFEST === */
// clang-format on

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "database.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "spi.hpp"
#include "thread.hpp"
#include "transform.hpp"

/**
 * @brief ICM-20648 / ICM-20948 6 轴 IMU 驱动模块，采样加速度与角速度并发布到 Topic。
 *        Driver Module for the ICM-20648 / ICM-20948 6-axis IMU; it samples the
 *        acceleration and angular velocity and publishes them to Topics.
 */
class ICM20948
{
 public:
  /// 角度到弧度的换算系数 (rad/deg)
  /// Degree-to-radian factor (rad/deg)
  static constexpr float M_DEG2RAD_MULT = 0.01745329251f;
  /// 标准重力加速度 (m/s^2)
  /// Standard gravity (m/s^2)
  static constexpr float STANDARD_GRAVITY = 9.80665f;

  /**
   * @brief 采样率设置；枚举值减 1 写入采样率分频寄存器。
   *        Sample-rate setting; the enum value minus 1 is written to the sample-rate
   *        divider registers.
   */
  enum class DataRate : uint8_t
  {
    DATA_RATE_4KHZ = 1,    ///< 分频值 0
                           ///< Divider 0
    DATA_RATE_1KHZ = 4,    ///< 分频值 3
                           ///< Divider 3
    DATA_RATE_500HZ = 8,   ///< 分频值 7
                           ///< Divider 7
    DATA_RATE_250HZ = 16,  ///< 分频值 15
                           ///< Divider 15
    DATA_RATE_125HZ = 32,  ///< 分频值 31
                           ///< Divider 31
  };

  /**
   * @brief 陀螺仪量程。
   *        Gyroscope range.
   */
  enum class GyroRange : uint8_t
  {
    DPS_250 = 0,   ///< ±250 dps
    DPS_500 = 1,   ///< ±500 dps
    DPS_1000 = 2,  ///< ±1000 dps
    DPS_2000 = 3,  ///< ±2000 dps
  };

  /**
   * @brief 加速度计量程。
   *        Accelerometer range.
   */
  enum class AcclRange : uint8_t
  {
    RANGE_2G = 0,   ///< ±2 g
    RANGE_4G = 1,   ///< ±4 g
    RANGE_8G = 2,   ///< ±8 g
    RANGE_16G = 3,  ///< ±16 g
  };

  /**
   * @brief ICM20948 配置参数。
   *        ICM20948 configuration parameters.
   */
  struct Param
  {
    DataRate data_rate;  ///< 采样率设置
    ///< Sample-rate setting
    AcclRange accl_range;  ///< 加速度计量程
    ///< Accelerometer range
    GyroRange gyro_range;  ///< 陀螺仪量程
    ///< Gyroscope range
    LibXR::Quaternion<float> rotation;  ///< 传感器到应用坐标系的四元数 (w, x, y, z)
    ///< Quaternion (w, x, y, z), sensor to application frame
    const char* gyro_topic_name;  ///< 陀螺仪 Topic 名称
    ///< Gyroscope Topic name
    const char* accl_topic_name;  ///< 加速度计 Topic 名称
    ///< Accelerometer Topic name
    size_t task_stack_depth;  ///< 采样线程栈深
    ///< Sampling thread stack depth
  };

  /**
   * @brief 构造 ICM20948：配置中断与 RamFS 命令，初始化芯片，创建采样线程。
   *        Construct ICM20948: configure the interrupt and the RamFS command, initialize
   *        the chip, and create the sampling thread.
   *
   * @param cs_pin 片选 GPIO。
   *               Chip-select GPIO.
   * @param int_pin 数据就绪中断 GPIO。
   *                Data-ready interrupt GPIO.
   * @param spi 连接 IMU 的 SPI。
   *            SPI connected to the IMU.
   * @param database 保存陀螺仪零偏的 Database。
   *                 Database that stores the gyroscope zero offset.
   * @param ramfs 接收 `bin/icm20948` 命令的 RamFS。
   *              RamFS that receives the `bin/icm20948` command.
   * @param param 配置参数。
   *              Configuration parameters.
   */
  ICM20948(LibXR::GPIO& cs_pin, LibXR::GPIO& int_pin, LibXR::SPI& spi,
           LibXR::Database& database, LibXR::RamFS& ramfs,
           const Param& param = {.data_rate = ICM20948::DataRate::DATA_RATE_1KHZ,
                                 .accl_range = ICM20948::AcclRange::RANGE_16G,
                                 .gyro_range = ICM20948::GyroRange::DPS_2000,
                                 .rotation = {1.0f, 0.0f, 0.0f, 0.0f},
                                 .gyro_topic_name = "icm20948_gyro",
                                 .accl_topic_name = "icm20948_accl",
                                 .task_stack_depth = 2048})
      : data_rate_(param.data_rate),
        accl_range_(param.accl_range),
        gyro_range_(param.gyro_range),
        topic_gyro_(
            LibXR::Topic::CreateTopic<decltype(gyro_data_)>(param.gyro_topic_name)),
        topic_accl_(
            LibXR::Topic::CreateTopic<decltype(accl_data_)>(param.accl_topic_name)),
        cs_(std::addressof(cs_pin)),
        int_(std::addressof(int_pin)),
        spi_(std::addressof(spi)),
        rotation_(std::move(param.rotation)),
        op_spi_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("icm20948", CommandFunc, this)),
        gyro_data_key_(database, "icm20948_gyro_data",
                       Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f))
  {
    ramfs.bin_.Add(cmd_file_);

    cs_->Write(true);
    int_->DisableInterrupt();
    int_->SetConfig({.direction = LibXR::GPIO::Direction::RISING_INTERRUPT,
                     .pull = LibXR::GPIO::Pull::NONE});

    auto int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, ICM20948* self) { self->new_data_.PostFromCallback(in_isr); },
        this);
    int_->RegisterCallback(int_cb);

    while (!Init())
    {
      XR_LOG_ERROR("ICM20948: Init failed, who_am_i=0x%02X. Retry...", who_am_i_);
      LibXR::Thread::Sleep(100);
    }
    XR_LOG_PASS("ICM20948: Init success, who_am_i=0x%02X.", who_am_i_);

    thread_.Create(this, ThreadFunc, "icm20948_thread", param.task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

  /**
   * @brief 监控回调：数据含 NaN 或 Inf 时输出警告。
   *        Monitor callback: log a warning when the data contains NaN or Inf.
   */
  void OnMonitor()
  {
    if (std::isinf(gyro_data_.x()) || std::isinf(gyro_data_.y()) ||
        std::isinf(gyro_data_.z()) || std::isinf(accl_data_.x()) ||
        std::isinf(accl_data_.y()) || std::isinf(accl_data_.z()) ||
        std::isnan(gyro_data_.x()) || std::isnan(gyro_data_.y()) ||
        std::isnan(gyro_data_.z()) || std::isnan(accl_data_.x()) ||
        std::isnan(accl_data_.y()) || std::isnan(accl_data_.z()))
    {
      XR_LOG_WARN("ICM20948: NaN data detected. gyro: %f %f %f, accl: %f %f %f",
                  gyro_data_.x(), gyro_data_.y(), gyro_data_.z(), accl_data_.x(),
                  accl_data_.y(), accl_data_.z());
    }
  }

 private:
  static constexpr uint8_t WHO_AM_I_ICM20948 = 0xEA;
  static constexpr uint8_t WHO_AM_I_ICM20648 = 0xE0;
  static constexpr uint8_t REG_BANK_SEL = 0x7F;
  static constexpr uint8_t BANK0_WHO_AM_I = 0x00;
  static constexpr uint8_t BANK0_USER_CTRL = 0x03;
  static constexpr uint8_t BANK0_LP_CONFIG = 0x05;
  static constexpr uint8_t BANK0_PWR_MGMT_1 = 0x06;
  static constexpr uint8_t BANK0_PWR_MGMT_2 = 0x07;
  static constexpr uint8_t BANK0_INT_PIN_CFG = 0x0F;
  static constexpr uint8_t BANK0_INT_ENABLE_1 = 0x11;
  static constexpr uint8_t BANK0_INT_STATUS_1 = 0x1A;
  static constexpr uint8_t BANK0_ACCEL_XOUT_H = 0x2D;
  static constexpr uint8_t BANK2_GYRO_SMPLRT_DIV = 0x00;
  static constexpr uint8_t BANK2_GYRO_CONFIG_1 = 0x01;
  static constexpr uint8_t BANK2_ACCEL_SMPLRT_DIV_1 = 0x10;
  static constexpr uint8_t BANK2_ACCEL_SMPLRT_DIV_2 = 0x11;
  static constexpr uint8_t BANK2_ACCEL_CONFIG = 0x14;
  static constexpr uint8_t READ_LEN = 14;

  static int CommandFunc(ICM20948* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      WriteStatus(self);
      return 0;
    }

    if (argc == 2)
    {
      if (strcmp(argv[1], "status") == 0)
      {
        WriteStatus(self);
        return 0;
      }
      if (strcmp(argv[1], "list_offset") == 0)
      {
        WriteOffset(self);
        return 0;
      }
      if (strcmp(argv[1], "cali") == 0)
      {
        return Calibrate(self);
      }
    }

    return -1;
  }

  static void WriteStatus(ICM20948* self)
  {
    LibXR::STDIO::Printf<"ICM who=%02X ok=%d t=%.3f\r\n">(
        self->who_am_i_, self->init_ok_ ? 1 : 0, self->temperature_);
  }

  static void WriteOffset(ICM20948* self)
  {
    LibXR::STDIO::Printf<"offset x=%.3f y=%.3f z=%.3f\r\n">(
        self->gyro_data_key_.data_.x(), self->gyro_data_key_.data_.y(),
        self->gyro_data_key_.data_.z());
  }

  static int Calibrate(ICM20948* self)
  {
    self->gyro_data_key_.data_.x() = 0.0f;
    self->gyro_data_key_.data_.y() = 0.0f;
    self->gyro_data_key_.data_.z() = 0.0f;
    self->gyro_cali_ = Eigen::Matrix<std::int64_t, 3, 1>(0, 0, 0);
    self->cali_counter_ = 0;
    self->in_cali_ = true;
    LibXR::STDIO::Printf<"cali start\r\n">();
    LibXR::Thread::Sleep(3000);
    LibXR::Thread::Sleep(60000);
    self->in_cali_ = false;
    LibXR::Thread::Sleep(1000);

    if (!SaveCalibration(self))
    {
      LibXR::STDIO::Printf<"cali failed\r\n">();
      return -1;
    }

    WriteOffset(self);
    if (self->gyro_data_key_.Set(self->gyro_data_key_.data_) == LibXR::ErrorCode::OK)
    {
      LibXR::STDIO::Printf<"cali saved\r\n">();
      return 0;
    }

    LibXR::STDIO::Printf<"cali save failed\r\n">();
    return -1;
  }

  static bool SaveCalibration(ICM20948* self)
  {
    if (self->cali_counter_ == 0)
    {
      return false;
    }

    float scale = self->GetGyroLSB() / static_cast<float>(self->cali_counter_);
    self->gyro_data_key_.data_.x() = self->gyro_cali_.data()[0] * scale;
    self->gyro_data_key_.data_.y() = self->gyro_cali_.data()[1] * scale;
    self->gyro_data_key_.data_.z() = self->gyro_cali_.data()[2] * scale;
    return true;
  }

  bool Init()
  {
    ConfigureBus();
    if (!SelectBank(0))
    {
      return false;
    }
    if (!WriteRegister(BANK0_PWR_MGMT_1, 0x80))
    {
      return false;
    }
    LibXR::Thread::Sleep(100);

    if (!SelectBank(0))
    {
      return false;
    }
    if (!ReadRegister(BANK0_WHO_AM_I, who_am_i_))
    {
      return false;
    }
    if (who_am_i_ != WHO_AM_I_ICM20948 && who_am_i_ != WHO_AM_I_ICM20648)
    {
      return false;
    }

    if (!WriteRegister(BANK0_PWR_MGMT_1, 0x01))
    {
      return false;
    }
    LibXR::Thread::Sleep(10);
    if (!WriteRegister(BANK0_PWR_MGMT_2, 0x00) || !WriteRegister(BANK0_LP_CONFIG, 0x00) ||
        !WriteRegister(BANK0_USER_CTRL, 0x10))
    {
      return false;
    }

    if (!SelectBank(2))
    {
      return false;
    }
    if (!WriteRegister(BANK2_GYRO_SMPLRT_DIV, static_cast<uint8_t>(data_rate_) - 1U) ||
        !WriteRegister(
            BANK2_GYRO_CONFIG_1,
            static_cast<uint8_t>((static_cast<uint8_t>(gyro_range_) << 1U) | 0x01U)) ||
        !WriteRegister(BANK2_ACCEL_SMPLRT_DIV_1, 0x00) ||
        !WriteRegister(BANK2_ACCEL_SMPLRT_DIV_2, static_cast<uint8_t>(data_rate_) - 1U) ||
        !WriteRegister(
            BANK2_ACCEL_CONFIG,
            static_cast<uint8_t>((static_cast<uint8_t>(accl_range_) << 1U) | 0x01U)))
    {
      return false;
    }

    if (!SelectBank(0))
    {
      return false;
    }
    if (!WriteRegister(BANK0_INT_ENABLE_1, 0x00))
    {
      return false;
    }
    // Default interrupt pin configuration: active high, pulse.
    if (!WriteRegister(BANK0_INT_PIN_CFG, 0x00))
    {
      return false;
    }
    // Clear the stale data-ready status before enabling the interrupt.
    uint8_t status = 0;
    if (!ReadRegister(BANK0_INT_STATUS_1, status))
    {
      return false;
    }
    if (!WriteRegister(BANK0_INT_ENABLE_1, 0x01))
    {
      return false;
    }
    int_->EnableInterrupt();
    init_ok_ = true;
    return true;
  }

  static void ThreadFunc(ICM20948* self)
  {
    while (true)
    {
      if (self->new_data_.Wait() == LibXR::ErrorCode::OK)
      {
        self->ReadSample();
      }
    }
  }

  void ConfigureBus()
  {
    LibXR::SPI::Configuration config;
    config.clock_polarity = LibXR::SPI::ClockPolarity::HIGH;
    config.clock_phase = LibXR::SPI::ClockPhase::EDGE_2;
    config.prescaler = LibXR::SPI::Prescaler::DIV_32;
    spi_->SetConfig(config);
  }

  bool SelectBank(uint8_t bank)
  {
    return WriteRegister(REG_BANK_SEL, static_cast<uint8_t>(bank << 4U));
  }

  bool WriteRegister(uint8_t reg, uint8_t data)
  {
    cs_->Write(false);
    auto ans = spi_->MemWrite(reg, data, op_spi_);
    cs_->Write(true);
    return ans == LibXR::ErrorCode::OK;
  }

  bool ReadRegister(uint8_t reg, uint8_t& data) { return ReadRegisters(reg, &data, 1); }

  bool ReadRegisters(uint8_t reg, uint8_t* data, uint8_t len)
  {
    cs_->Write(false);
    auto ans = spi_->MemRead(reg, {data, static_cast<size_t>(len)}, op_spi_);
    cs_->Write(true);
    return ans == LibXR::ErrorCode::OK;
  }

  void ReadSample()
  {
    if (!ReadRegisters(BANK0_ACCEL_XOUT_H, buffer_, READ_LEN))
    {
      return;
    }
    Parse();
    topic_accl_.Publish(accl_data_);
    topic_gyro_.Publish(gyro_data_);
  }

  void Parse()
  {
    std::array<int16_t, 3> accl_raw_i16;
    std::array<int16_t, 3> gyro_raw_i16;
    std::array<float, 3> accl_raw;
    std::array<float, 3> gyro_raw;

    // ICM-20948 burst layout from ACCEL_XOUT_H is accel XYZ, gyro XYZ, then temperature.
    for (int i = 0; i < 3; i++)
    {
      accl_raw_i16[i] = static_cast<int16_t>(buffer_[i * 2] << 8 | buffer_[i * 2 + 1]);
      accl_raw[i] = static_cast<float>(accl_raw_i16[i]) * GetAcclLSB();
      gyro_raw_i16[i] =
          static_cast<int16_t>(buffer_[i * 2 + 6] << 8 | buffer_[i * 2 + 7]);
      gyro_raw[i] = static_cast<float>(gyro_raw_i16[i]) * GetGyroLSB();
    }

    if (in_cali_)
    {
      gyro_cali_.data()[0] += gyro_raw_i16[0];
      gyro_cali_.data()[1] += gyro_raw_i16[1];
      gyro_cali_.data()[2] += gyro_raw_i16[2];
      cali_counter_++;
    }

    int16_t temp_raw = static_cast<int16_t>(buffer_[12] << 8 | buffer_[13]);
    temperature_ = static_cast<float>(temp_raw) / 333.87f + 21.0f;

    accl_data_ =
        rotation_ * Eigen::Matrix<float, 3, 1>(accl_raw[0], accl_raw[1], accl_raw[2]);
    gyro_data_ = rotation_ *
                 Eigen::Matrix<float, 3, 1>(
                     Eigen::Matrix<float, 3, 1>(gyro_raw[0], gyro_raw[1], gyro_raw[2]) -
                     gyro_data_key_.data_);
  }

  float GetAcclLSB() const
  {
    switch (accl_range_)
    {
      case AcclRange::RANGE_2G:
        return 2.0f * STANDARD_GRAVITY / 32768.0f;
      case AcclRange::RANGE_4G:
        return 4.0f * STANDARD_GRAVITY / 32768.0f;
      case AcclRange::RANGE_8G:
        return 8.0f * STANDARD_GRAVITY / 32768.0f;
      case AcclRange::RANGE_16G:
        return 16.0f * STANDARD_GRAVITY / 32768.0f;
      default:
        ASSERT(false);
        return 0.0f;
    }
  }

  float GetGyroLSB() const
  {
    float dps = 0.0f;
    switch (gyro_range_)
    {
      case GyroRange::DPS_250:
        dps = 250.0f;
        break;
      case GyroRange::DPS_500:
        dps = 500.0f;
        break;
      case GyroRange::DPS_1000:
        dps = 1000.0f;
        break;
      case GyroRange::DPS_2000:
        dps = 2000.0f;
        break;
      default:
        ASSERT(false);
        break;
    }
    return dps / 32768.0f * M_DEG2RAD_MULT;
  }

  DataRate data_rate_;
  AcclRange accl_range_;
  GyroRange gyro_range_;
  float temperature_ = 0.0f;
  uint8_t who_am_i_ = 0;
  bool init_ok_ = false;
  bool in_cali_ = false;
  uint32_t cali_counter_ = 0;
  Eigen::Matrix<std::int64_t, 3, 1> gyro_cali_;

  uint8_t buffer_[READ_LEN] = {};
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;

  LibXR::Topic topic_gyro_, topic_accl_;
  LibXR::GPIO *cs_, *int_;
  LibXR::SPI* spi_;
  LibXR::Quaternion<float> rotation_;
  LibXR::Semaphore sem_spi_, new_data_;
  LibXR::SPI::OperationRW op_spi_;
  LibXR::Thread thread_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_data_key_;
};
