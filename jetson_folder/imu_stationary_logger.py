#!/usr/bin/env python3
"""
imu_stationary_logger.py

Log stationary IMU data from a GY-91 / MPU-9250 to CSV for later noise analysis.

Purpose
-------
- collect a short stationary dataset (for example 5 minutes)
- collect a long stationary dataset (for example 45 minutes)
- preserve timestamps and sample timing information
- make it easy to estimate IMU noise statistics later

Recommended use
---------------
1. Place the IMU on a stable surface.
2. Do not touch it during logging.
3. Start with a short test, e.g. 60-300 s.
4. Use the quick-look plotting script to inspect the CSV.
5. Then run a long test, e.g. 2700 s (45 min).

Notes
-----
- This script targets about 200 Hz because the current IMU configuration is set to about 200 Hz.
- It is intentionally single-threaded for simplicity and traceability.
- For this data rate and this task, a single-threaded logger is usually sufficient.
"""

import argparse
import csv
import math
import os
import time
from dataclasses import dataclass

import numpy as np
from smbus2 import SMBus


# ============================================================
# MPU-9250 register definitions
# ============================================================

MPU9250_I2C_ADDR = 0x68

MPU_PWR_MGMT_1 = 0x6B
MPU_SMPLRT_DIV = 0x19
MPU_CONFIG = 0x1A
MPU_GYRO_CONFIG = 0x1B
MPU_ACCEL_CONFIG = 0x1C
MPU_ACCEL_CONFIG2 = 0x1D
MPU_ACCEL_XOUT_H = 0x3B
MPU_WHO_AM_I = 0x75


# ============================================================
# Helper functions
# ============================================================

def int16_from_bytes(msb, lsb):
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value


def apply_axis_remap(raw_vec, remap_matrix):
    """Convert a 3D vector from sensor frame into chosen body frame."""
    return remap_matrix @ raw_vec


# ============================================================
# Configuration
# ============================================================

@dataclass
class LoggerConfig:
    bus_num: int = 7
    i2c_addr: int = MPU9250_I2C_ADDR
    target_rate_hz: float = 200.0

    # Keep identity until you decide on a different body-frame convention.
    accel_remap_matrix: np.ndarray = np.eye(3, dtype=np.float64)
    gyro_remap_matrix: np.ndarray = np.eye(3, dtype=np.float64)

    # Standard gravity for conversion from g to m/s^2.
    # The later analysis can estimate the actual stationary norm.
    g0_mps2: float = 9.80665


# ============================================================
# IMU reader
# ============================================================

class Mpu9250Reader:
    def __init__(self, i2c_bus, config: LoggerConfig):
        self.i2c_bus = i2c_bus
        self.config = config
        self.i2c_addr = config.i2c_addr

    def write_u8(self, register_addr, value):
        self.i2c_bus.write_byte_data(self.i2c_addr, register_addr, value)

    def read_u8(self, register_addr):
        return self.i2c_bus.read_byte_data(self.i2c_addr, register_addr)

    def read_block(self, register_addr, length):
        return self.i2c_bus.read_i2c_block_data(self.i2c_addr, register_addr, length)

    def check_connection(self):
        return self.read_u8(MPU_WHO_AM_I)

    def initialize(self):
        # Wake up sensor
        self.write_u8(MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # With DLPF enabled, internal sample rate is typically 1 kHz.
        # divider=4 -> 1000 / (1 + 4) = 200 Hz
        self.write_u8(MPU_SMPLRT_DIV, 0x04)
        self.write_u8(MPU_CONFIG, 0x03)
        self.write_u8(MPU_GYRO_CONFIG, 0x00)   # ±250 deg/s
        self.write_u8(MPU_ACCEL_CONFIG, 0x00)  # ±2 g
        self.write_u8(MPU_ACCEL_CONFIG2, 0x03)
        time.sleep(0.05)

    def read_sample(self):
        # Two timestamps:
        # - wall clock: easier for human reference and CSV inspection
        # - monotonic: better for dt and timing analysis
        timestamp_unix_sec = time.time()
        timestamp_monotonic_sec = time.perf_counter()

        data = self.read_block(MPU_ACCEL_XOUT_H, 14)

        raw_accel_x = int16_from_bytes(data[0], data[1])
        raw_accel_y = int16_from_bytes(data[2], data[3])
        raw_accel_z = int16_from_bytes(data[4], data[5])
        raw_temp = int16_from_bytes(data[6], data[7])
        raw_gyro_x = int16_from_bytes(data[8], data[9])
        raw_gyro_y = int16_from_bytes(data[10], data[11])
        raw_gyro_z = int16_from_bytes(data[12], data[13])

        accel_mps2 = np.array([
            raw_accel_x / 16384.0 * self.config.g0_mps2,
            raw_accel_y / 16384.0 * self.config.g0_mps2,
            raw_accel_z / 16384.0 * self.config.g0_mps2,
        ], dtype=np.float64)

        deg_to_rad = math.pi / 180.0
        gyro_rps = np.array([
            raw_gyro_x / 131.0 * deg_to_rad,
            raw_gyro_y / 131.0 * deg_to_rad,
            raw_gyro_z / 131.0 * deg_to_rad,
        ], dtype=np.float64)

        temperature_c = (raw_temp / 333.87) + 21.0

        accel_mps2 = apply_axis_remap(accel_mps2, self.config.accel_remap_matrix)
        gyro_rps = apply_axis_remap(gyro_rps, self.config.gyro_remap_matrix)

        return {
            "timestamp_unix_sec": timestamp_unix_sec,
            "timestamp_monotonic_sec": timestamp_monotonic_sec,
            "accel_mps2": accel_mps2,
            "gyro_rps": gyro_rps,
            "temperature_c": float(temperature_c),
            "raw_accel": np.array([raw_accel_x, raw_accel_y, raw_accel_z], dtype=np.int32),
            "raw_gyro": np.array([raw_gyro_x, raw_gyro_y, raw_gyro_z], dtype=np.int32),
        }


# ============================================================
# Logging
# ============================================================

def log_stationary_imu(output_csv_path: str, duration_sec: float, config: LoggerConfig):
    sample_period_sec = 1.0 / config.target_rate_hz

    os.makedirs(os.path.dirname(output_csv_path) or ".", exist_ok=True)

    with SMBus(config.bus_num) as i2c_bus:
        imu_reader = Mpu9250Reader(i2c_bus, config)

        who_am_i = imu_reader.check_connection()
        print(f"MPU WHO_AM_I: 0x{who_am_i:02X}")

        imu_reader.initialize()

        print(f"Target rate: {config.target_rate_hz:.1f} Hz")
        print(f"Target sample period: {sample_period_sec*1000.0:.3f} ms")
        print(f"Duration: {duration_sec:.1f} s")
        print(f"Output CSV: {output_csv_path}")
        print("Keep the IMU completely stationary during the run.")

        start_monotonic_sec = time.perf_counter()
        next_sample_time_sec = start_monotonic_sec
        sample_index = 0
        dropped_deadline_count = 0
        last_report_time_sec = start_monotonic_sec

        with open(output_csv_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "sample_index",
                "timestamp_unix_sec",
                "timestamp_monotonic_sec",
                "dt_sec",
                "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
                "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
                "temperature_c",
                "raw_accel_x", "raw_accel_y", "raw_accel_z",
                "raw_gyro_x", "raw_gyro_y", "raw_gyro_z",
            ])

            previous_timestamp_monotonic_sec = None

            while True:
                now_sec = time.perf_counter()
                elapsed_sec = now_sec - start_monotonic_sec
                if elapsed_sec >= duration_sec:
                    break

                sleep_sec = next_sample_time_sec - now_sec
                if sleep_sec > 0.0:
                    time.sleep(sleep_sec)
                else:
                    if sample_index > 0:
                        dropped_deadline_count += 1

                sample = imu_reader.read_sample()
                current_timestamp_monotonic_sec = sample["timestamp_monotonic_sec"]

                if previous_timestamp_monotonic_sec is None:
                    dt_sec = 0.0
                else:
                    dt_sec = current_timestamp_monotonic_sec - previous_timestamp_monotonic_sec
                previous_timestamp_monotonic_sec = current_timestamp_monotonic_sec

                ax, ay, az = sample["accel_mps2"]
                gx, gy, gz = sample["gyro_rps"]
                raw_ax, raw_ay, raw_az = sample["raw_accel"]
                raw_gx, raw_gy, raw_gz = sample["raw_gyro"]

                writer.writerow([
                    sample_index,
                    f"{sample['timestamp_unix_sec']:.9f}",
                    f"{sample['timestamp_monotonic_sec']:.9f}",
                    f"{dt_sec:.9f}",
                    f"{ax:.9f}", f"{ay:.9f}", f"{az:.9f}",
                    f"{gx:.9f}", f"{gy:.9f}", f"{gz:.9f}",
                    f"{sample['temperature_c']:.6f}",
                    raw_ax, raw_ay, raw_az,
                    raw_gx, raw_gy, raw_gz,
                ])

                sample_index += 1
                next_sample_time_sec += sample_period_sec

                report_now_sec = time.perf_counter()
                if report_now_sec - last_report_time_sec >= 5.0:
                    actual_elapsed_sec = report_now_sec - start_monotonic_sec
                    actual_rate_hz = sample_index / max(actual_elapsed_sec, 1e-9)
                    print(
                        f"Logged {sample_index} samples | "
                        f"elapsed {actual_elapsed_sec:.1f} s | "
                        f"actual rate {actual_rate_hz:.2f} Hz | "
                        f"deadline misses {dropped_deadline_count}"
                    )
                    last_report_time_sec = report_now_sec

        total_elapsed_sec = time.perf_counter() - start_monotonic_sec
        actual_rate_hz = sample_index / max(total_elapsed_sec, 1e-9)

        print("\nLogging finished.")
        print(f"Total samples: {sample_index}")
        print(f"Total elapsed time: {total_elapsed_sec:.3f} s")
        print(f"Average achieved rate: {actual_rate_hz:.3f} Hz")
        print(f"Deadline misses: {dropped_deadline_count}")


def main():
    parser = argparse.ArgumentParser(description="Log stationary IMU data to CSV.")
    parser.add_argument("--output", type=str, required=True, help="Output CSV path")
    parser.add_argument("--duration-sec", type=float, required=True, help="Logging duration in seconds")
    parser.add_argument("--rate-hz", type=float, default=200.0, help="Target logging rate in Hz")
    parser.add_argument("--bus", type=int, default=7, help="I2C bus number")
    args = parser.parse_args()

    config = LoggerConfig(
        bus_num=args.bus,
        target_rate_hz=args.rate_hz,
    )

    log_stationary_imu(
        output_csv_path=args.output,
        duration_sec=args.duration_sec,
        config=config,
    )


if __name__ == "__main__":
    main()
