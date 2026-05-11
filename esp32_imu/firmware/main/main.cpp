// ESP32 Dual GY-91 IMU → Serial bridge
// =====================================
// Boot sequence:
//   1. AD0 pin LOW  → both IMUs at 0x68
//   2. Configure both via 0x68
//   3. AD0 pin HIGH → arm IMU moves to 0x69
//   4. Loop: read head@0x68 + arm@0x69, send binary packet over USB serial
//
// Binary packet format (33 bytes, little-endian):
//   [0-1]   Header:    0xAA 0x55
//   [2-3]   Seq:       uint16  (wraps 0..65535)
//   [4-7]   Timestamp: uint32  (microseconds since boot)
//   [8-13]  Head accel: int16 x, y, z  (raw sensor units)
//   [14-19] Head gyro:  int16 x, y, z
//   [20-25] Arm accel:  int16 x, y, z
//   [26-31] Arm gyro:   int16 x, y, z
//   [32]    Checksum:   uint8  (XOR of bytes [2..31])

#include <cstdint>
#include <cstring>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "gy91.h"

static const char *TAG = "main";

// ── Pin assignments ─────────────────────────────────────────────────
constexpr gpio_num_t AD0_PIN       = GPIO_NUM_19;  // arm IMU SAO/AD0
constexpr gpio_num_t I2C_SDA       = GPIO_NUM_21;
constexpr gpio_num_t I2C_SCL       = GPIO_NUM_22;
constexpr i2c_port_t I2C_PORT      = I2C_NUM_0;
constexpr uint32_t    I2C_FREQ_HZ  = 400000;

constexpr uart_port_t UART_PORT    = UART_NUM_0;
constexpr int         UART_BAUD    = 921600;
constexpr int         UART_TX_PIN  = 1;   // USB serial TX
constexpr int         UART_RX_PIN  = 3;   // USB serial RX

constexpr uint8_t  HEAD_ADDR = 0x68;
constexpr uint8_t  ARM_ADDR  = 0x69;

constexpr int      OUTPUT_RATE_HZ = 200;
constexpr uint32_t PERIOD_US     = 1'000'000 / OUTPUT_RATE_HZ;  // 5000 µs

// ── Packet ──────────────────────────────────────────────────────────
#pragma pack(push, 1)
struct IMUPacket {
    uint8_t  header[2];   // 0xAA 0x55
    uint16_t seq;
    uint32_t timestamp_us;
    int16_t  head_accel[3];
    int16_t  head_gyro[3];
    int16_t  arm_accel[3];
    int16_t  arm_gyro[3];
    uint8_t  checksum;
};
#pragma pack(pop)

static_assert(sizeof(IMUPacket) == 33, "packet size must be 33 bytes");

static void compute_checksum(IMUPacket &pkt) {
    uint8_t c = 0;
    uint8_t *data = reinterpret_cast<uint8_t *>(&pkt);
    // XOR bytes 2..31 (skip header, skip checksum field at 32)
    for (size_t i = 2; i < 32; i++) {
        c ^= data[i];
    }
    pkt.checksum = c;
}

// ── I2C init ────────────────────────────────────────────────────────
static bool i2c_init() {
    i2c_config_t cfg = {};
    cfg.mode             = I2C_MODE_MASTER;
    cfg.sda_io_num       = I2C_SDA;
    cfg.scl_io_num       = I2C_SCL;
    cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = I2C_FREQ_HZ;
    cfg.clk_flags        = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t err = i2c_param_config(I2C_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config: %s", esp_err_to_name(err));
        return false;
    }
    err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

// ── UART init ───────────────────────────────────────────────────────
static bool uart_init() {
    uart_config_t cfg = {};
    cfg.baud_rate  = UART_BAUD;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(UART_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err));
        return false;
    }
    err = uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(err));
        return false;
    }
    err = uart_driver_install(UART_PORT, 256, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

// ── Main ────────────────────────────────────────────────────────────
extern "C" void app_main() {
    // ── Step 0: GPIO for AD0 control ────────────────────────────────
    gpio_set_direction(AD0_PIN, GPIO_MODE_OUTPUT);

    // ── Step 1: AD0 LOW, both IMUs at 0x68 ─────────────────────────
    gpio_set_level(AD0_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (!i2c_init() || !uart_init()) {
        ESP_LOGE(TAG, "Hardware init failed — halting");
        vTaskSuspend(nullptr);
    }

    // ── Step 2: Configure both IMUs at 0x68 ────────────────────────
    bool head_ok = false, arm_ok = false;

    GY91 head_imu(I2C_PORT, HEAD_ADDR);
    if (head_imu.init(GyroRange::DPS_2000, AccelRange::G_16,
                      DlpfBandwidth::BW_184HZ)) {
        head_ok = true;
    } else {
        ESP_LOGW(TAG, "Head IMU at 0x68 not found — will send zeros");
    }

    // ── Step 3: AD0 HIGH, arm IMU moves to 0x69 ───────────────────
    gpio_set_level(AD0_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    GY91 arm_imu(I2C_PORT, ARM_ADDR);
    // The arm IMU was already configured in step 2 (same registers wrote
    // to both devices).  Just verify we can read it at the new address.
    {
        uint8_t whoami = 0;
        uint8_t reg = REG_WHO_AM_I;
        esp_err_t err = i2c_master_write_read_device(
            I2C_PORT, ARM_ADDR, &reg, 1, &whoami, 1, pdMS_TO_TICKS(10));
        if (err != ESP_OK) {
            ESP_LOGW(TAG,
                     "Arm IMU at 0x69 not responding — check SAO wiring. "
                     "Will send zeros for arm.");
        } else {
            arm_ok = true;
            ESP_LOGI(TAG, "Arm IMU confirmed at 0x69 (WHO_AM_I=0x%02X)",
                     whoami);
        }
    }

    if (!head_ok && !arm_ok) {
        ESP_LOGW(TAG, "No IMUs detected — streaming zeros until connected");
    }

    ESP_LOGI(TAG, "Dual IMU streaming at %d Hz (head=%s arm=%s)",
             OUTPUT_RATE_HZ,
             head_ok ? "OK" : "MISSING",
             arm_ok  ? "OK" : "MISSING");

    // ── Step 4: Main loop ──────────────────────────────────────────
    uint16_t seq = 0;
    int64_t last_wake = esp_timer_get_time();

    while (true) {
        IMUPacket pkt;
        pkt.header[0]    = 0xAA;
        pkt.header[1]    = 0x55;
        pkt.seq          = seq++;
        pkt.timestamp_us = (uint32_t)esp_timer_get_time();

        // Read head IMU
        if (head_ok) {
            IMUReading raw;
            if (head_imu.read_all(raw)) {
                pkt.head_accel[0] = raw.accel_x;
                pkt.head_accel[1] = raw.accel_y;
                pkt.head_accel[2] = raw.accel_z;
                pkt.head_gyro[0]  = raw.gyro_x;
                pkt.head_gyro[1]  = raw.gyro_y;
                pkt.head_gyro[2]  = raw.gyro_z;
            } else {
                memset(pkt.head_accel, 0, 6);
                memset(pkt.head_gyro,  0, 6);
            }
        } else {
            memset(pkt.head_accel, 0, 6);
            memset(pkt.head_gyro,  0, 6);
        }

        // Read arm IMU
        if (arm_ok) {
            IMUReading raw;
            if (arm_imu.read_all(raw)) {
                pkt.arm_accel[0] = raw.accel_x;
                pkt.arm_accel[1] = raw.accel_y;
                pkt.arm_accel[2] = raw.accel_z;
                pkt.arm_gyro[0]  = raw.gyro_x;
                pkt.arm_gyro[1]  = raw.gyro_y;
                pkt.arm_gyro[2]  = raw.gyro_z;
            } else {
                memset(pkt.arm_accel, 0, 6);
                memset(pkt.arm_gyro,  0, 6);
            }
        } else {
            memset(pkt.arm_accel, 0, 6);
            memset(pkt.arm_gyro,  0, 6);
        }

        compute_checksum(pkt);

        int written = uart_write_bytes(
            UART_PORT, &pkt, sizeof(IMUPacket));
        if (written != sizeof(IMUPacket)) {
            ESP_LOGW(TAG, "UART write short: %d/%d",
                     written, (int)sizeof(IMUPacket));
        }

        // Rate control: busy-wait until next period
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - last_wake;
        if (elapsed < PERIOD_US) {
            vTaskDelay(pdMS_TO_TICKS((PERIOD_US - elapsed) / 1000));
        }
        last_wake += PERIOD_US;
        // Prevent drift accumulation
        if (last_wake < now - PERIOD_US) {
            last_wake = now;
        }
    }
}
