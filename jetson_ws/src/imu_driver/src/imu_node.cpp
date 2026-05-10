#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

// ------------------------------------------------------------
// MPU-6500 / MPU-9250-compatible registers
// ------------------------------------------------------------
static constexpr uint8_t REG_SMPLRT_DIV      = 0x19;
static constexpr uint8_t REG_CONFIG          = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG     = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG    = 0x1C;
static constexpr uint8_t REG_ACCEL_CONFIG2   = 0x1D;
static constexpr uint8_t REG_ACCEL_XOUT_H    = 0x3B;
static constexpr uint8_t REG_PWR_MGMT_1      = 0x6B;
static constexpr uint8_t REG_WHO_AM_I        = 0x75;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
static int16_t int16_from_bytes(uint8_t msb, uint8_t lsb)
{
  return static_cast<int16_t>((msb << 8) | lsb);
}

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("imu_node"), fd_(-1)
  {
    bus_ = this->declare_parameter<int>("bus", 7);
    imu_i2c_address_ = this->declare_parameter<int>("imu_i2c_address", 0x68);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
    imu_rate_hz_ = this->declare_parameter<double>("imu_rate_hz", 200.0);
    accel_range_g_ = this->declare_parameter<int>("accel_range_g", 2);
    gyro_range_dps_ = this->declare_parameter<int>("gyro_range_dps", 250);
    publish_temperature_in_imu_msg_ =
      this->declare_parameter<bool>("publish_temperature_in_imu_msg", true);

    if (imu_rate_hz_ <= 0.0) {
      throw std::runtime_error("imu_rate_hz must be > 0");
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "data_raw", rclcpp::SensorDataQoS());

    open_i2c_device();
    configure_sensor();

    const auto period = std::chrono::duration<double>(1.0 / imu_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ImuNode::poll_and_publish, this));

    RCLCPP_INFO(
      this->get_logger(),
      "imu_node started: bus=%d addr=0x%02X rate=%.1f Hz frame_id=%s",
      bus_, imu_i2c_address_, imu_rate_hz_, frame_id_.c_str());
  }

  ~ImuNode() override
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  void open_i2c_device()
  {
    const std::string dev_path = "/dev/i2c-" + std::to_string(bus_);

    fd_ = open(dev_path.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open " + dev_path + ": " + std::strerror(errno));
    }

    if (ioctl(fd_, I2C_SLAVE, imu_i2c_address_) < 0) {
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("Failed to select I2C slave: " + std::string(std::strerror(errno)));
    }

    const uint8_t who_am_i = read_u8(REG_WHO_AM_I);
    RCLCPP_INFO(this->get_logger(), "MPU WHO_AM_I = 0x%02X", who_am_i);
  }

  void configure_sensor()
  {
    // Wake sensor
    write_u8(REG_PWR_MGMT_1, 0x00);
    usleep(100000);

    // Set sample divider based on requested output rate.
    // Assumes internal rate about 1 kHz with DLPF enabled.
    int divider = static_cast<int>(std::round(1000.0 / imu_rate_hz_ - 1.0));
    if (divider < 0) {
      divider = 0;
    }
    if (divider > 255) {
      divider = 255;
    }

    write_u8(REG_SMPLRT_DIV, static_cast<uint8_t>(divider));

    // DLPF config similar to your earlier tests
    write_u8(REG_CONFIG, 0x03);

    // Gyro range
    uint8_t gyro_cfg = 0x00;  // ±250 dps
    if (gyro_range_dps_ == 500) {
      gyro_cfg = 0x08;
    } else if (gyro_range_dps_ == 1000) {
      gyro_cfg = 0x10;
    } else if (gyro_range_dps_ == 2000) {
      gyro_cfg = 0x18;
    }
    write_u8(REG_GYRO_CONFIG, gyro_cfg);

    // Accel range
    uint8_t accel_cfg = 0x00;  // ±2 g
    if (accel_range_g_ == 4) {
      accel_cfg = 0x08;
    } else if (accel_range_g_ == 8) {
      accel_cfg = 0x10;
    } else if (accel_range_g_ == 16) {
      accel_cfg = 0x18;
    }
    write_u8(REG_ACCEL_CONFIG, accel_cfg);

    // Accel DLPF
    write_u8(REG_ACCEL_CONFIG2, 0x03);

    usleep(50000);
  }

  void write_u8(uint8_t reg, uint8_t value)
  {
    uint8_t buffer[2] = {reg, value};
    const ssize_t written = write(fd_, buffer, 2);
    if (written != 2) {
      throw std::runtime_error("I2C write failed: " + std::string(std::strerror(errno)));
    }
  }

  uint8_t read_u8(uint8_t reg)
  {
    if (write(fd_, &reg, 1) != 1) {
      throw std::runtime_error("I2C register select failed: " + std::string(std::strerror(errno)));
    }

    uint8_t value = 0;
    if (read(fd_, &value, 1) != 1) {
      throw std::runtime_error("I2C read_u8 failed: " + std::string(std::strerror(errno)));
    }

    return value;
  }

  void read_block(uint8_t start_reg, uint8_t * data, size_t len)
  {
    if (write(fd_, &start_reg, 1) != 1) {
      throw std::runtime_error("I2C block register select failed: " + std::string(std::strerror(errno)));
    }

    if (read(fd_, data, len) != static_cast<ssize_t>(len)) {
      throw std::runtime_error("I2C block read failed: " + std::string(std::strerror(errno)));
    }
  }

  bool read_imu_sample(
    double & ax, double & ay, double & az,
    double & gx, double & gy, double & gz,
    double & temperature_c)
  {
    uint8_t data[14] = {0};

    try {
      read_block(REG_ACCEL_XOUT_H, data, 14);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU read failed: %s", e.what());
      return false;
    }

    const int16_t raw_ax = int16_from_bytes(data[0], data[1]);
    const int16_t raw_ay = int16_from_bytes(data[2], data[3]);
    const int16_t raw_az = int16_from_bytes(data[4], data[5]);
    const int16_t raw_temp = int16_from_bytes(data[6], data[7]);
    const int16_t raw_gx = int16_from_bytes(data[8], data[9]);
    const int16_t raw_gy = int16_from_bytes(data[10], data[11]);
    const int16_t raw_gz = int16_from_bytes(data[12], data[13]);

    double accel_lsb_per_g = 16384.0;  // ±2 g
    if (accel_range_g_ == 4) {
      accel_lsb_per_g = 8192.0;
    } else if (accel_range_g_ == 8) {
      accel_lsb_per_g = 4096.0;
    } else if (accel_range_g_ == 16) {
      accel_lsb_per_g = 2048.0;
    }

    double gyro_lsb_per_dps = 131.0;  // ±250 dps
    if (gyro_range_dps_ == 500) {
      gyro_lsb_per_dps = 65.5;
    } else if (gyro_range_dps_ == 1000) {
      gyro_lsb_per_dps = 32.8;
    } else if (gyro_range_dps_ == 2000) {
      gyro_lsb_per_dps = 16.4;
    }

    constexpr double g0 = 9.80665;
    constexpr double deg_to_rad = M_PI / 180.0;

    ax = (static_cast<double>(raw_ax) / accel_lsb_per_g) * g0;
    ay = (static_cast<double>(raw_ay) / accel_lsb_per_g) * g0;
    az = (static_cast<double>(raw_az) / accel_lsb_per_g) * g0;

    gx = (static_cast<double>(raw_gx) / gyro_lsb_per_dps) * deg_to_rad;
    gy = (static_cast<double>(raw_gy) / gyro_lsb_per_dps) * deg_to_rad;
    gz = (static_cast<double>(raw_gz) / gyro_lsb_per_dps) * deg_to_rad;

    temperature_c = (static_cast<double>(raw_temp) / 333.87) + 21.0;

    return true;
  }

  void poll_and_publish()
  {
    double ax = 0.0, ay = 0.0, az = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    double temperature_c = 0.0;

    if (!read_imu_sample(ax, ay, az, gx, gy, gz, temperature_c)) {
      return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;

    // This node does not estimate orientation.
    msg.orientation_covariance[0] = -1.0;

    // Unknown for now. Fill later after characterization.
    msg.angular_velocity_covariance[0] = 0.0;
    msg.angular_velocity_covariance[4] = 0.0;
    msg.angular_velocity_covariance[8] = 0.0;

    msg.linear_acceleration_covariance[0] = 0.0;
    msg.linear_acceleration_covariance[4] = 0.0;
    msg.linear_acceleration_covariance[8] = 0.0;

    (void)publish_temperature_in_imu_msg_;  // kept as parameter for now
    (void)temperature_c;

    imu_pub_->publish(msg);
  }

  int bus_;
  int imu_i2c_address_;
  std::string frame_id_;
  double imu_rate_hz_;
  int accel_range_g_;
  int gyro_range_dps_;
  bool publish_temperature_in_imu_msg_;

  int fd_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}