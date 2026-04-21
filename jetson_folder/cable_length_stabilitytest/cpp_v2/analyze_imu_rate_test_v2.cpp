#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Row {
    double scheduler_lag_sec = 0.0;
    double read_duration_sec = 0.0;
    double dt_sec = 0.0;
    int read_ok = 0;
    double accel_norm = 0.0;
    double gyro_norm = 0.0;
};

struct Stats {
    double mean = 0.0;
    double stddev = 0.0;
    double p99 = 0.0;
    double p999 = 0.0;
    double max = 0.0;
};

std::vector<std::string> split_csv_line(const std::string &line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(item);
    return out;
}

Stats compute_stats(std::vector<double> v) {
    Stats s;
    if (v.empty()) return s;
    double sum = 0.0;
    for (double x : v) sum += x;
    s.mean = sum / static_cast<double>(v.size());
    double var = 0.0;
    for (double x : v) {
        double d = x - s.mean;
        var += d * d;
    }
    s.stddev = std::sqrt(var / static_cast<double>(v.size()));
    std::sort(v.begin(), v.end());
    auto pct = [&](double p) {
        size_t idx = static_cast<size_t>(std::clamp(p, 0.0, 1.0) * static_cast<double>(v.size() - 1));
        return v[idx];
    };
    s.p99 = pct(0.99);
    s.p999 = pct(0.999);
    s.max = pct(1.0);
    return s;
}

struct FileReport {
    std::string path;
    double requested_rate_hz = 0.0;
    double requested_period_sec = 0.0;
    size_t rows = 0;
    size_t good_reads = 0;
    size_t read_errors = 0;
    size_t wake_gt_25pct = 0;
    size_t wake_gt_50pct = 0;
    size_t dt_gt_125pct = 0;
    size_t dt_gt_150pct = 0;
    size_t dt_gt_200pct = 0;
    Stats dt_stats;
    Stats read_stats;
    Stats lag_stats;
    Stats accel_norm_stats;
    Stats gyro_norm_stats;
    std::string verdict;
};

FileReport analyze_file(const std::string &path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Failed to open " + path);

    std::string header;
    if (!std::getline(in, header)) throw std::runtime_error("Empty file: " + path);

    std::vector<double> dt, read, lag, accel, gyro;
    FileReport r;
    r.path = path;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto c = split_csv_line(line);
        if (c.size() < 26) continue;
        Row row;
        row.read_ok = std::stoi(c[7]);
        row.scheduler_lag_sec = std::stod(c[9]);
        row.read_duration_sec = std::stod(c[10]);
        row.dt_sec = std::stod(c[12]);
        double ax = std::stod(c[20]);
        double ay = std::stod(c[21]);
        double az = std::stod(c[22]);
        double gx = std::stod(c[23]);
        double gy = std::stod(c[24]);
        double gz = std::stod(c[25]);
        row.accel_norm = std::sqrt(ax*ax + ay*ay + az*az);
        row.gyro_norm = std::sqrt(gx*gx + gy*gy + gz*gz);

        r.rows++;
        if (row.read_ok) r.good_reads++; else r.read_errors++;
        lag.push_back(std::max(0.0, row.scheduler_lag_sec));
        read.push_back(row.read_duration_sec);
        accel.push_back(row.accel_norm);
        gyro.push_back(row.gyro_norm);
        if (row.dt_sec > 0.0) dt.push_back(row.dt_sec);
    }

    if (dt.empty()) throw std::runtime_error("No dt values found in " + path);

    double dt_mean = 0.0;
    for (double v : dt) dt_mean += v;
    dt_mean /= static_cast<double>(dt.size());
    r.requested_period_sec = dt_mean; // use observed mean rather than filename assumptions
    r.requested_rate_hz = 1.0 / dt_mean;

    for (double x : lag) {
        if (x > 0.25 * r.requested_period_sec) r.wake_gt_25pct++;
        if (x > 0.50 * r.requested_period_sec) r.wake_gt_50pct++;
    }
    for (double x : dt) {
        if (x > 1.25 * r.requested_period_sec) r.dt_gt_125pct++;
        if (x > 1.50 * r.requested_period_sec) r.dt_gt_150pct++;
        if (x > 2.00 * r.requested_period_sec) r.dt_gt_200pct++;
    }

    r.dt_stats = compute_stats(dt);
    r.read_stats = compute_stats(read);
    r.lag_stats = compute_stats(lag);
    r.accel_norm_stats = compute_stats(accel);
    r.gyro_norm_stats = compute_stats(gyro);

    const double read_err_frac = static_cast<double>(r.read_errors) / static_cast<double>(std::max<size_t>(1, r.rows));
    const double wake50_frac = static_cast<double>(r.wake_gt_50pct) / static_cast<double>(std::max<size_t>(1, lag.size()));
    const double dt150_frac = static_cast<double>(r.dt_gt_150pct) / static_cast<double>(std::max<size_t>(1, dt.size()));
    const double dt200_frac = static_cast<double>(r.dt_gt_200pct) / static_cast<double>(std::max<size_t>(1, dt.size()));
    const double read_p99_frac = r.read_stats.p99 / r.requested_period_sec;

    if (read_err_frac > 0.0 || dt200_frac > 0.001 || r.read_stats.max > 2.0 * r.requested_period_sec) {
        r.verdict = "UNSTABLE";
    } else if (wake50_frac > 0.01 || dt150_frac > 0.005 || read_p99_frac > 0.35) {
        r.verdict = "MARGINAL";
    } else {
        r.verdict = "GOOD";
    }

    return r;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: analyze_imu_rate_test_v2 file1.csv [file2.csv ...]\n";
        return 1;
    }

    std::vector<FileReport> reports;
    try {
        for (int i = 1; i < argc; ++i) reports.push_back(analyze_file(argv[i]));
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(6);
    for (const auto &r : reports) {
        std::cout << "File:                         " << r.path << "\n";
        std::cout << "Estimated rate:               " << r.requested_rate_hz << " Hz\n";
        std::cout << "Estimated period:             " << r.requested_period_sec * 1000.0 << " ms\n";
        std::cout << "Scheduled rows:               " << r.rows << "\n";
        std::cout << "Successful reads:             " << r.good_reads << "\n";
        std::cout << "Read errors:                  " << r.read_errors << "\n";
        std::cout << "Wake late >25% period:        " << r.wake_gt_25pct << "\n";
        std::cout << "Wake late >50% period:        " << r.wake_gt_50pct << "\n";
        std::cout << "dt >125% period:              " << r.dt_gt_125pct << "\n";
        std::cout << "dt >150% period:              " << r.dt_gt_150pct << "\n";
        std::cout << "dt >200% period:              " << r.dt_gt_200pct << "\n";
        std::cout << "dt mean/std:                  " << r.dt_stats.mean * 1000.0 << " / " << r.dt_stats.stddev * 1000.0 << " ms\n";
        std::cout << "dt p99/p99.9/max:             " << r.dt_stats.p99 * 1000.0 << " / " << r.dt_stats.p999 * 1000.0 << " / " << r.dt_stats.max * 1000.0 << " ms\n";
        std::cout << "read mean/p99/p99.9/max:      " << r.read_stats.mean * 1000.0 << " / " << r.read_stats.p99 * 1000.0 << " / " << r.read_stats.p999 * 1000.0 << " / " << r.read_stats.max * 1000.0 << " ms\n";
        std::cout << "lag mean/p99/p99.9/max:       " << r.lag_stats.mean * 1000.0 << " / " << r.lag_stats.p99 * 1000.0 << " / " << r.lag_stats.p999 * 1000.0 << " / " << r.lag_stats.max * 1000.0 << " ms\n";
        std::cout << "accel norm mean/std:          " << r.accel_norm_stats.mean << " / " << r.accel_norm_stats.stddev << " m/s^2\n";
        std::cout << "gyro norm mean/std:           " << r.gyro_norm_stats.mean << " / " << r.gyro_norm_stats.stddev << " rad/s\n";
        std::cout << "Verdict:                      " << r.verdict << "\n\n";
    }

    if (reports.size() > 1) {
        std::cout << "Summary by file:\n";
        std::cout << "rate_hz,verdict,read_p99_ms,dt_p99_ms,dt_p99_9_ms,wake50_count,dt150_count,read_errors,file\n";
        for (const auto &r : reports) {
            std::cout << r.requested_rate_hz << ','
                      << r.verdict << ','
                      << r.read_stats.p99 * 1000.0 << ','
                      << r.dt_stats.p99 * 1000.0 << ','
                      << r.dt_stats.p999 * 1000.0 << ','
                      << r.wake_gt_50pct << ','
                      << r.dt_gt_150pct << ','
                      << r.read_errors << ','
                      << r.path << '\n';
        }
    }

    return 0;
}
