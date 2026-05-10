#pragma once
// GY-91 driver: MPU9250 (9-DOF IMU) via I2C.
// BMP280 (pressure/temp) on the GY-91 is not used for VIO.
//
// Register map based on MPU-9250 datasheet rev 1.1.

#include <cstdint>
#include <driver/i2c.h>

// ── I2C addresses ────────────────────────────────────────────────────
constexpr uint8_t MPU9250_ADDR_DEFAULT = 0x68;  // AD0 low
constexpr uint8_t MPU9250_ADDR_ALT     = 0x69;  // AD0 high

// ── Register addresses ───────────────────────────────────────────────
constexpr uint8_t REG_WHO_AM_I    = 0x75;
constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2  = 0x6C;
constexpr uint8_t REG_CONFIG      = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t REG_SMPLRT_DIV  = 0x19;
constexpr uint8_t REG_INT_PIN_CFG = 0x37;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // accel + temp + gyro, 14 bytes

constexpr uint8_t WHO_AM_I_EXPECTED = 0x71;

// ── Scale factors ────────────────────────────────────────────────────
// Convert raw 16-bit signed int to physical units.
// Values from MPU-9250 register map.

// Gyro: raw * scale = °/s
constexpr float GYRO_SCALE_250  = 131.0f;
constexpr float GYRO_SCALE_500  = 65.5f;
constexpr float GYRO_SCALE_1000 = 32.8f;
constexpr float GYRO_SCALE_2000 = 16.4f;

// Accel: raw * scale = g
constexpr float ACCEL_SCALE_2G  = 16384.0f;
constexpr float ACCEL_SCALE_4G  = 8192.0f;
constexpr float ACCEL_SCALE_8G  = 4096.0f;
constexpr float ACCEL_SCALE_16G = 2048.0f;

// ── DLPF bandwidth ───────────────────────────────────────────────────
// DLPF_CFG values for REG_CONFIG and REG_ACCEL_CONFIG2
enum class DlpfBandwidth : uint8_t {
    BW_184HZ = 0x01,  // Gyro: 184Hz, 2.9ms delay; Accel: 184Hz, 5.8ms delay
    BW_92HZ  = 0x02,
    BW_41HZ  = 0x03,
    BW_20HZ  = 0x04,
    BW_10HZ  = 0x05,
    BW_5HZ   = 0x06,
};

// ── Gyro full-scale ──────────────────────────────────────────────────
enum class GyroRange : uint8_t {
    DPS_250  = 0x00,
    DPS_500  = 0x08,
    DPS_1000 = 0x10,
    DPS_2000 = 0x18,
};

// ── Accel full-scale ─────────────────────────────────────────────────
enum class AccelRange : uint8_t {
    G_2  = 0x00,
    G_4  = 0x08,
    G_8  = 0x10,
    G_16 = 0x18,
};

// ── Single IMU reading ───────────────────────────────────────────────
struct IMUReading {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
    int16_t temp;  // raw temperature
};

// ── Driver class ─────────────────────────────────────────────────────
class GY91 {
public:
    GY91(i2c_port_t port, uint8_t addr);

    // Initialise: wake up, configure ranges + DLPF, verify WHO_AM_I.
    // Returns true on success.
    bool init(GyroRange gyro_range, AccelRange accel_range,
              DlpfBandwidth dlpf);

    // Read all 14 sensor registers in one I2C burst.
    bool read_all(IMUReading &out);

    // Convenience: read and convert to float.
    bool read_float(float &gx, float &gy, float &gz,
                    float &ax, float &ay, float &az);

    uint8_t addr() const { return addr_; }

private:
    i2c_port_t port_;
    uint8_t    addr_;
    float      gyro_scale_;
    float      accel_scale_;

    bool write_reg(uint8_t reg, uint8_t val);
    bool read_regs(uint8_t reg, uint8_t *buf, size_t len);
};
