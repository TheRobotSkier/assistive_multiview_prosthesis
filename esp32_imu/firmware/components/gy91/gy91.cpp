#include "gy91.h"
#include <esp_log.h>

static const char *TAG = "gy91";

GY91::GY91(i2c_port_t port, uint8_t addr)
    : port_(port), addr_(addr), gyro_scale_(1.0f), accel_scale_(1.0f) {}

bool GY91::init(GyroRange gyro_range, AccelRange accel_range,
                DlpfBandwidth dlpf) {
    // ── Verify WHO_AM_I ──────────────────────────────────────────────
    uint8_t whoami = 0;
    if (!read_regs(REG_WHO_AM_I, &whoami, 1)) {
        ESP_LOGE(TAG, "addr 0x%02X: I2C read WHO_AM_I failed", addr_);
        return false;
    }
    if (whoami != WHO_AM_I_EXPECTED) {
        ESP_LOGE(TAG, "addr 0x%02X: bad WHO_AM_I 0x%02X (expected 0x%02X)",
                 addr_, whoami, WHO_AM_I_EXPECTED);
        return false;
    }
    ESP_LOGI(TAG, "addr 0x%02X: MPU9250 found", addr_);

    // ── Wake up ──────────────────────────────────────────────────────
    // Reset sleep bit and set clock source to PLL.
    if (!write_reg(REG_PWR_MGMT_1, 0x01)) return false;
    vTaskDelay(pdMS_TO_TICKS(50));

    // ── Set sample rate divider ────────────────────────────────────
    // Sample rate = gyro ODR / (1 + SMPLRT_DIV).
    // Gyro ODR is 1kHz with DLPF enabled.
    // For 200Hz output: 1000 / (1 + 4) = 200Hz.
    if (!write_reg(REG_SMPLRT_DIV, 4)) return false;

    // ── Gyro config ──────────────────────────────────────────────────
    uint8_t gyro_val = static_cast<uint8_t>(gyro_range);
    if (!write_reg(REG_GYRO_CONFIG, gyro_val)) return false;

    switch (gyro_range) {
        case GyroRange::DPS_250:  gyro_scale_  = GYRO_SCALE_250;  break;
        case GyroRange::DPS_500:  gyro_scale_  = GYRO_SCALE_500;  break;
        case GyroRange::DPS_1000: gyro_scale_  = GYRO_SCALE_1000; break;
        case GyroRange::DPS_2000: gyro_scale_  = GYRO_SCALE_2000; break;
    }

    // ── Accel config ─────────────────────────────────────────────────
    uint8_t accel_val = static_cast<uint8_t>(accel_range);
    if (!write_reg(REG_ACCEL_CONFIG, accel_val)) return false;

    switch (accel_range) {
        case AccelRange::G_2:  accel_scale_ = ACCEL_SCALE_2G;  break;
        case AccelRange::G_4:  accel_scale_ = ACCEL_SCALE_4G;  break;
        case AccelRange::G_8:  accel_scale_ = ACCEL_SCALE_8G;  break;
        case AccelRange::G_16: accel_scale_ = ACCEL_SCALE_16G; break;
    }

    // ── DLPF config ──────────────────────────────────────────────────
    uint8_t dlpf_val = static_cast<uint8_t>(dlpf);
    if (!write_reg(REG_CONFIG, dlpf_val)) return false;
    if (!write_reg(REG_ACCEL_CONFIG2, dlpf_val)) return false;

    ESP_LOGI(TAG, "addr 0x%02X: init OK (gyro_fs=%d accel_fs=%d dlpf=%d)",
             addr_, (int)gyro_range, (int)accel_range, (int)dlpf);
    return true;
}

bool GY91::read_all(IMUReading &out) {
    uint8_t buf[14];
    if (!read_regs(REG_ACCEL_XOUT_H, buf, 14)) return false;

    out.accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    out.accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    out.accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    out.temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    out.gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    out.gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    out.gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
    return true;
}

bool GY91::read_float(float &gx, float &gy, float &gz,
                       float &ax, float &ay, float &az) {
    IMUReading raw;
    if (!read_all(raw)) return false;

    gx = raw.gyro_x  / gyro_scale_;
    gy = raw.gyro_y  / gyro_scale_;
    gz = raw.gyro_z  / gyro_scale_;
    ax = raw.accel_x / accel_scale_;
    ay = raw.accel_y / accel_scale_;
    az = raw.accel_z / accel_scale_;
    return true;
}

// ── Private helpers ────────────────────────────────────────────────────
bool GY91::write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    esp_err_t err = i2c_master_write_to_device(
        port_, addr_, buf, 2, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "addr 0x%02X: write reg 0x%02X failed: %s",
                 addr_, reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool GY91::read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    esp_err_t err = i2c_master_write_read_device(
        port_, addr_, &reg, 1, buf, len, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "addr 0x%02X: read %d regs from 0x%02X failed: %s",
                 addr_, (int)len, reg, esp_err_to_name(err));
        return false;
    }
    return true;
}
