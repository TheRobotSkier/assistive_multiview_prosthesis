#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint8_t MPU9250_I2C_ADDR = 0x68;
constexpr uint8_t MPU_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU_CONFIG = 0x1A;
constexpr uint8_t MPU_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_WHO_AM_I = 0x75;

int16_t int16_from_bytes(uint8_t msb, uint8_t lsb) {
    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb));
}

double timespec_to_sec(const timespec &ts) {
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

timespec sec_to_timespec(double sec) {
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(std::floor(sec));
    double frac = sec - static_cast<double>(ts.tv_sec);
    ts.tv_nsec = static_cast<long>(std::llround(frac * 1e9));
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000L;
    }
    return ts;
}

double monotonic_sec() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return timespec_to_sec(ts);
}

double realtime_sec() {
    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    return timespec_to_sec(ts);
}

void sleep_until_monotonic(double target_sec) {
    timespec ts = sec_to_timespec(target_sec);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

struct Config {
    int bus = 7;
    int address = MPU9250_I2C_ADDR;
    double rate_hz = 200.0;
    double duration_sec = 120.0;
    std::string output_csv;
    bool no_csv = false;
    int status_interval_samples = 1000;
    int warmup_samples = 5;
    int sensor_divider = -1;
};

struct Sample {
    uint64_t sample_index = 0;
    double scheduled_time_sec = 0.0;
    double wake_time_sec = 0.0;
    double read_start_sec = 0.0;
    double read_end_sec = 0.0;
    double monotonic_timestamp_sec = 0.0;
    double unix_timestamp_sec = 0.0;
    bool read_ok = false;
    int error_errno = 0;
    double scheduler_lag_sec = 0.0;
    double read_duration_sec = 0.0;
    double loop_duration_sec = 0.0;
    double dt_sec = 0.0;
    std::array<int16_t, 3> raw_accel{};
    std::array<int16_t, 3> raw_gyro{};
    int16_t raw_temp = 0;
    double temperature_c = 0.0;
    std::array<double, 3> accel_mps2{};
    std::array<double, 3> gyro_rps{};
};

class I2cDevice {
public:
    explicit I2cDevice(const Config &cfg) : cfg_(cfg) {}

    void open_device() {
        std::ostringstream path;
        path << "/dev/i2c-" << cfg_.bus;
        fd_ = ::open(path.str().c_str(), O_RDWR);
        if (fd_ < 0) throw std::runtime_error("Failed to open " + path.str() + ": " + std::strerror(errno));
    }

    ~I2cDevice() {
        if (fd_ >= 0) ::close(fd_);
    }

    uint8_t read_u8(uint8_t reg) {
        uint8_t reg_buf = reg;
        uint8_t out = 0;
        i2c_msg msgs[2]{};
        msgs[0].addr = static_cast<__u16>(cfg_.address);
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = &reg_buf;
        msgs[1].addr = static_cast<__u16>(cfg_.address);
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = 1;
        msgs[1].buf = &out;
        i2c_rdwr_ioctl_data ioctl_data{};
        ioctl_data.msgs = msgs;
        ioctl_data.nmsgs = 2;
        if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
            throw std::runtime_error("I2C read_u8 failed: " + std::string(std::strerror(errno)));
        }
        return out;
    }

    void write_u8(uint8_t reg, uint8_t value) {
        uint8_t buf[2]{reg, value};
        i2c_msg msg{};
        msg.addr = static_cast<__u16>(cfg_.address);
        msg.flags = 0;
        msg.len = 2;
        msg.buf = buf;
        i2c_rdwr_ioctl_data ioctl_data{};
        ioctl_data.msgs = &msg;
        ioctl_data.nmsgs = 1;
        if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
            throw std::runtime_error("I2C write_u8 failed: " + std::string(std::strerror(errno)));
        }
    }

    bool read_block(uint8_t reg, uint8_t *dst, uint16_t len, int &error_out) {
        uint8_t reg_buf = reg;
        i2c_msg msgs[2]{};
        msgs[0].addr = static_cast<__u16>(cfg_.address);
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = &reg_buf;
        msgs[1].addr = static_cast<__u16>(cfg_.address);
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = len;
        msgs[1].buf = dst;
        i2c_rdwr_ioctl_data ioctl_data{};
        ioctl_data.msgs = msgs;
        ioctl_data.nmsgs = 2;
        if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
            error_out = errno;
            return false;
        }
        error_out = 0;
        return true;
    }

private:
    Config cfg_;
    int fd_ = -1;
};

class Mpu9250 {
public:
    explicit Mpu9250(const Config &cfg) : cfg_(cfg), i2c_(cfg) {}

    void open_device() { i2c_.open_device(); }
    uint8_t read_u8(uint8_t reg) { return i2c_.read_u8(reg); }

    void initialize(int divider) {
        i2c_.write_u8(MPU_PWR_MGMT_1, 0x00);
        usleep(100000);
        i2c_.write_u8(MPU_SMPLRT_DIV, static_cast<uint8_t>(divider));
        i2c_.write_u8(MPU_CONFIG, 0x03);
        i2c_.write_u8(MPU_GYRO_CONFIG, 0x00);
        i2c_.write_u8(MPU_ACCEL_CONFIG, 0x00);
        i2c_.write_u8(MPU_ACCEL_CONFIG2, 0x03);
        usleep(50000);
    }

    bool read_sample(Sample &sample) {
        uint8_t data[14]{};
        sample.unix_timestamp_sec = realtime_sec();
        sample.monotonic_timestamp_sec = monotonic_sec();
        int err = 0;
        if (!i2c_.read_block(MPU_ACCEL_XOUT_H, data, 14, err)) {
            sample.error_errno = err;
            sample.read_ok = false;
            return false;
        }

        sample.raw_accel = {
            int16_from_bytes(data[0], data[1]),
            int16_from_bytes(data[2], data[3]),
            int16_from_bytes(data[4], data[5])
        };
        sample.raw_temp = int16_from_bytes(data[6], data[7]);
        sample.raw_gyro = {
            int16_from_bytes(data[8], data[9]),
            int16_from_bytes(data[10], data[11]),
            int16_from_bytes(data[12], data[13])
        };
        constexpr double g0 = 9.80665;
        constexpr double deg_to_rad = M_PI / 180.0;
        sample.accel_mps2 = {
            static_cast<double>(sample.raw_accel[0]) / 16384.0 * g0,
            static_cast<double>(sample.raw_accel[1]) / 16384.0 * g0,
            static_cast<double>(sample.raw_accel[2]) / 16384.0 * g0,
        };
        sample.gyro_rps = {
            static_cast<double>(sample.raw_gyro[0]) / 131.0 * deg_to_rad,
            static_cast<double>(sample.raw_gyro[1]) / 131.0 * deg_to_rad,
            static_cast<double>(sample.raw_gyro[2]) / 131.0 * deg_to_rad,
        };
        sample.temperature_c = static_cast<double>(sample.raw_temp) / 333.87 + 21.0;
        sample.read_ok = true;
        sample.error_errno = 0;
        return true;
    }

private:
    Config cfg_;
    I2cDevice i2c_;
};

struct Summary {
    uint64_t scheduled_rows = 0;
    uint64_t successful_reads = 0;
    uint64_t read_errors = 0;
    uint64_t wake_late_gt_25pct = 0;
    uint64_t wake_late_gt_50pct = 0;
    uint64_t dt_gt_125pct = 0;
    uint64_t dt_gt_150pct = 0;
    uint64_t dt_gt_200pct = 0;
};

void write_csv_header(std::ofstream &out) {
    out << "sample_index,scheduled_time_sec,wake_time_sec,read_start_sec,read_end_sec,"
           "timestamp_unix_sec,timestamp_monotonic_sec,read_ok,error_errno,scheduler_lag_sec,"
           "read_duration_sec,loop_duration_sec,dt_sec,raw_accel_x,raw_accel_y,raw_accel_z,"
           "raw_gyro_x,raw_gyro_y,raw_gyro_z,temperature_c,accel_x_mps2,accel_y_mps2,accel_z_mps2,"
           "gyro_x_rps,gyro_y_rps,gyro_z_rps\n";
}

void write_csv_row(std::ofstream &out, const Sample &s) {
    out << s.sample_index << ','
        << std::fixed << std::setprecision(9)
        << s.scheduled_time_sec << ','
        << s.wake_time_sec << ','
        << s.read_start_sec << ','
        << s.read_end_sec << ','
        << s.unix_timestamp_sec << ','
        << s.monotonic_timestamp_sec << ','
        << (s.read_ok ? 1 : 0) << ','
        << s.error_errno << ','
        << s.scheduler_lag_sec << ','
        << s.read_duration_sec << ','
        << s.loop_duration_sec << ','
        << s.dt_sec << ','
        << s.raw_accel[0] << ',' << s.raw_accel[1] << ',' << s.raw_accel[2] << ','
        << s.raw_gyro[0] << ',' << s.raw_gyro[1] << ',' << s.raw_gyro[2] << ','
        << s.temperature_c << ','
        << s.accel_mps2[0] << ',' << s.accel_mps2[1] << ',' << s.accel_mps2[2] << ','
        << s.gyro_rps[0] << ',' << s.gyro_rps[1] << ',' << s.gyro_rps[2] << '\n';
}

int divider_for_rate(double rate_hz) {
    double divider = std::round(1000.0 / rate_hz - 1.0);
    divider = std::clamp(divider, 0.0, 255.0);
    return static_cast<int>(divider);
}

double configured_rate_from_divider(int divider) {
    return 1000.0 / static_cast<double>(divider + 1);
}

Config parse_args(int argc, char **argv) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto take = [&](const std::string &name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + name);
            return argv[++i];
        };
        if (arg == "--output") cfg.output_csv = take(arg);
        else if (arg == "--duration-sec") cfg.duration_sec = std::stod(take(arg));
        else if (arg == "--rate-hz") cfg.rate_hz = std::stod(take(arg));
        else if (arg == "--bus") cfg.bus = std::stoi(take(arg));
        else if (arg == "--address") cfg.address = std::stoi(take(arg), nullptr, 0);
        else if (arg == "--status-interval-samples") cfg.status_interval_samples = std::stoi(take(arg));
        else if (arg == "--warmup-samples") cfg.warmup_samples = std::stoi(take(arg));
        else if (arg == "--sensor-divider") cfg.sensor_divider = std::stoi(take(arg));
        else if (arg == "--no-csv") cfg.no_csv = true;
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: imu_i2c_rate_test_logger_v2 --output file.csv --duration-sec 120 --rate-hz 200 [--no-csv]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    if (!cfg.no_csv && cfg.output_csv.empty()) {
        throw std::runtime_error("--output is required unless --no-csv is used");
    }
    return cfg;
}

} // namespace

int main(int argc, char **argv) {
    try {
        Config cfg = parse_args(argc, argv);
        const double requested_period_sec = 1.0 / cfg.rate_hz;
        const int divider = (cfg.sensor_divider >= 0) ? cfg.sensor_divider : divider_for_rate(cfg.rate_hz);
        const double configured_rate_hz = configured_rate_from_divider(divider);
        const double configured_period_sec = 1.0 / configured_rate_hz;

        Mpu9250 imu(cfg);
        imu.open_device();
        uint8_t who = imu.read_u8(MPU_WHO_AM_I);
        imu.initialize(divider);

        std::ofstream csv;
        if (!cfg.no_csv) {
            csv.open(cfg.output_csv);
            if (!csv) throw std::runtime_error("Failed to open output CSV: " + cfg.output_csv);
            write_csv_header(csv);
        }

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "MPU WHO_AM_I: 0x" << std::hex << std::uppercase << static_cast<int>(who) << std::dec << "\n";
        std::cout << "Requested rate:           " << cfg.rate_hz << " Hz\n";
        std::cout << "Requested period:         " << requested_period_sec * 1000.0 << " ms\n";
        std::cout << "Configured sensor rate:   " << configured_rate_hz << " Hz\n";
        std::cout << "Configured sensor period: " << configured_period_sec * 1000.0 << " ms\n";
        std::cout << "Duration:                 " << cfg.duration_sec << " s\n";
        std::cout << "Warmup samples skipped:   " << cfg.warmup_samples << "\n";
        if (!cfg.no_csv) std::cout << "Output CSV:               " << cfg.output_csv << "\n";
        else std::cout << "Output CSV:               disabled (--no-csv)\n";
        std::cout << '\n';

        const double t0 = monotonic_sec();
        double next_time = t0;
        double last_good_ts = 0.0;
        Summary summary;
        std::vector<double> dt_hist, read_hist, lag_hist;
        const size_t reserve_n = static_cast<size_t>(std::ceil(cfg.duration_sec * cfg.rate_hz));
        dt_hist.reserve(reserve_n);
        read_hist.reserve(reserve_n);
        lag_hist.reserve(reserve_n);

        auto percentile = [](std::vector<double> v, double p) {
            if (v.empty()) return 0.0;
            std::sort(v.begin(), v.end());
            size_t idx = static_cast<size_t>(std::clamp(p, 0.0, 1.0) * static_cast<double>(v.size() - 1));
            return v[idx];
        };
        auto mean = [](const std::vector<double> &v) {
            if (v.empty()) return 0.0;
            double s = 0.0;
            for (double x : v) s += x;
            return s / static_cast<double>(v.size());
        };
        auto stddev = [&](const std::vector<double> &v) {
            if (v.empty()) return 0.0;
            double m = mean(v);
            double acc = 0.0;
            for (double x : v) {
                double d = x - m;
                acc += d * d;
            }
            return std::sqrt(acc / static_cast<double>(v.size()));
        };

        for (uint64_t idx = 0;; ++idx, next_time += requested_period_sec) {
            double before_sleep = monotonic_sec();
            if (before_sleep - t0 >= cfg.duration_sec) break;

            sleep_until_monotonic(next_time);

            Sample s;
            s.sample_index = idx;
            s.scheduled_time_sec = next_time;
            s.wake_time_sec = monotonic_sec();
            s.scheduler_lag_sec = s.wake_time_sec - s.scheduled_time_sec;
            s.read_start_sec = monotonic_sec();
            imu.read_sample(s);
            s.read_end_sec = monotonic_sec();
            s.read_duration_sec = s.read_end_sec - s.read_start_sec;
            s.loop_duration_sec = s.read_end_sec - s.wake_time_sec;
            if (s.read_ok && last_good_ts > 0.0) s.dt_sec = s.monotonic_timestamp_sec - last_good_ts;
            if (s.read_ok) last_good_ts = s.monotonic_timestamp_sec;

            summary.scheduled_rows++;
            if (s.read_ok) summary.successful_reads++; else summary.read_errors++;

            if (idx >= static_cast<uint64_t>(cfg.warmup_samples)) {
                if (s.scheduler_lag_sec > 0.25 * requested_period_sec) summary.wake_late_gt_25pct++;
                if (s.scheduler_lag_sec > 0.50 * requested_period_sec) summary.wake_late_gt_50pct++;
                if (s.read_ok && s.dt_sec > 0.0) {
                    dt_hist.push_back(s.dt_sec);
                    if (s.dt_sec > 1.25 * requested_period_sec) summary.dt_gt_125pct++;
                    if (s.dt_sec > 1.50 * requested_period_sec) summary.dt_gt_150pct++;
                    if (s.dt_sec > 2.00 * requested_period_sec) summary.dt_gt_200pct++;
                }
                read_hist.push_back(s.read_duration_sec);
                lag_hist.push_back(std::max(0.0, s.scheduler_lag_sec));
            }

            if (!cfg.no_csv) write_csv_row(csv, s);

            if (cfg.status_interval_samples > 0 && (idx + 1) % static_cast<uint64_t>(cfg.status_interval_samples) == 0) {
                double achieved_rate = static_cast<double>(summary.scheduled_rows) / (monotonic_sec() - t0);
                std::cout << "samples=" << summary.scheduled_rows
                          << " | achieved=" << achieved_rate << " Hz"
                          << " | errors=" << summary.read_errors
                          << " | wake>50%=" << summary.wake_late_gt_50pct
                          << " | dt>150%=" << summary.dt_gt_150pct
                          << " | dt_std=" << stddev(dt_hist) * 1000.0 << " ms"
                          << " | read_p99=" << percentile(read_hist, 0.99) * 1000.0 << " ms\n";
            }
        }

        const double elapsed = monotonic_sec() - t0;
        std::cout << "\nTest finished.\n";
        std::cout << "Total scheduled samples:              " << summary.scheduled_rows << "\n";
        std::cout << "Total elapsed time:                   " << elapsed << " s\n";
        std::cout << "Average achieved schedule rate:       " << (static_cast<double>(summary.scheduled_rows) / elapsed) << " Hz\n";
        std::cout << "Read errors:                          " << summary.read_errors << "\n";
        std::cout << "Wake late by >25% of period:          " << summary.wake_late_gt_25pct << "\n";
        std::cout << "Wake late by >50% of period:          " << summary.wake_late_gt_50pct << "\n";
        std::cout << "dt >125% of period:                   " << summary.dt_gt_125pct << "\n";
        std::cout << "dt >150% of period:                   " << summary.dt_gt_150pct << "\n";
        std::cout << "dt >200% of period:                   " << summary.dt_gt_200pct << "\n";
        std::cout << "dt mean:                              " << mean(dt_hist) * 1000.0 << " ms\n";
        std::cout << "dt std:                               " << stddev(dt_hist) * 1000.0 << " ms\n";
        std::cout << "dt p99:                               " << percentile(dt_hist, 0.99) * 1000.0 << " ms\n";
        std::cout << "dt p99.9:                             " << percentile(dt_hist, 0.999) * 1000.0 << " ms\n";
        std::cout << "dt max:                               " << percentile(dt_hist, 1.0) * 1000.0 << " ms\n";
        std::cout << "read duration mean:                   " << mean(read_hist) * 1000.0 << " ms\n";
        std::cout << "read duration p99:                    " << percentile(read_hist, 0.99) * 1000.0 << " ms\n";
        std::cout << "read duration p99.9:                  " << percentile(read_hist, 0.999) * 1000.0 << " ms\n";
        std::cout << "read duration max:                    " << percentile(read_hist, 1.0) * 1000.0 << " ms\n";
        std::cout << "positive schedule lag mean:           " << mean(lag_hist) * 1000.0 << " ms\n";
        std::cout << "schedule lag p99:                     " << percentile(lag_hist, 0.99) * 1000.0 << " ms\n";
        std::cout << "schedule lag p99.9:                   " << percentile(lag_hist, 0.999) * 1000.0 << " ms\n";

        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
