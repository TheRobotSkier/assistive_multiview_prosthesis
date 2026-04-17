#!/usr/bin/env python3
"""
imu_i2c_rate_test_logger.py

Purpose
-------
Measure whether an MPU-9250 / GY-91 connected over I2C can be read
reliably at a chosen target rate.

What this script measures
-------------------------
- achieved sample rate
- sample-to-sample dt jitter using time.perf_counter()
- I2C transaction duration for each read
- scheduler lag relative to the requested sample schedule
- deadline misses
- read errors (OSError)
- raw IMU data, so timing outliers can be compared against data spikes

Why this is useful
------------------
If a 1.5 m I2C cable is marginal, the first symptoms are usually:
- occasional long read transactions
- irregular dt
- deadline misses
- occasional read errors
- occasional one-sample raw data spikes

This script is intentionally simple and single-threaded so the timing path
is easy to understand.
"""

import argparse
import csv
import math
import os
import statistics
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from smbus2 import SMBus


MPU9250_I2C_ADDR = 0x68
MPU_PWR_MGMT_1 = 0x6B
MPU_SMPLRT_DIV = 0x19
MPU_CONFIG = 0x1A
MPU_GYRO_CONFIG = 0x1B
MPU_ACCEL_CONFIG = 0x1C
MPU_ACCEL_CONFIG2 = 0x1D
MPU_ACCEL_XOUT_H = 0x3B
MPU_WHO_AM_I = 0x75


@dataclass
class LoggerConfig:
    bus_num: int = 7
    i2c_addr: int = MPU9250_I2C_ADDR
    target_rate_hz: float = 200.0
    dlpf_cfg: int = 0x03
    accel_dlpf_cfg: int = 0x03
    gyro_fs_sel: int = 0x00   # +-250 dps
    accel_fs_sel: int = 0x00  # +-2 g
    g0_mps2: float = 9.80665
    warmup_sec: float = 0.25


def int16_from_bytes(msb: int, lsb: int) -> int:
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value


class Mpu9250Reader:
    def __init__(self, i2c_bus: SMBus, config: LoggerConfig):
        self.i2c_bus = i2c_bus
        self.config = config
        self.i2c_addr = config.i2c_addr

    def write_u8(self, register_addr: int, value: int) -> None:
        self.i2c_bus.write_byte_data(self.i2c_addr, register_addr, value)

    def read_u8(self, register_addr: int) -> int:
        return self.i2c_bus.read_byte_data(self.i2c_addr, register_addr)

    def read_block(self, register_addr: int, length: int):
        return self.i2c_bus.read_i2c_block_data(self.i2c_addr, register_addr, length)

    def check_connection(self) -> int:
        return self.read_u8(MPU_WHO_AM_I)

    def compute_sample_rate_divider(self) -> int:
        # With DLPF enabled, the internal sample rate is typically 1 kHz.
        internal_rate_hz = 1000.0
        divider = max(0, int(round((internal_rate_hz / self.config.target_rate_hz) - 1.0)))
        divider = min(divider, 255)
        return divider

    def configured_output_rate_hz(self) -> float:
        divider = self.compute_sample_rate_divider()
        return 1000.0 / (1.0 + divider)

    def initialize(self) -> None:
        self.write_u8(MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        divider = self.compute_sample_rate_divider()
        self.write_u8(MPU_SMPLRT_DIV, divider)
        self.write_u8(MPU_CONFIG, self.config.dlpf_cfg)
        self.write_u8(MPU_GYRO_CONFIG, self.config.gyro_fs_sel)
        self.write_u8(MPU_ACCEL_CONFIG, self.config.accel_fs_sel)
        self.write_u8(MPU_ACCEL_CONFIG2, self.config.accel_dlpf_cfg)
        time.sleep(0.05)

    def read_sample(self):
        read_start_monotonic_sec = time.perf_counter()
        data = self.read_block(MPU_ACCEL_XOUT_H, 14)
        read_end_monotonic_sec = time.perf_counter()
        timestamp_unix_sec = time.time()

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

        return {
            "timestamp_unix_sec": timestamp_unix_sec,
            "timestamp_monotonic_sec": read_end_monotonic_sec,
            "read_start_monotonic_sec": read_start_monotonic_sec,
            "read_end_monotonic_sec": read_end_monotonic_sec,
            "read_duration_sec": read_end_monotonic_sec - read_start_monotonic_sec,
            "accel_mps2": accel_mps2,
            "gyro_rps": gyro_rps,
            "temperature_c": float(temperature_c),
            "raw_accel": np.array([raw_accel_x, raw_accel_y, raw_accel_z], dtype=np.int32),
            "raw_gyro": np.array([raw_gyro_x, raw_gyro_y, raw_gyro_z], dtype=np.int32),
        }


def percentile_or_nan(values, pct: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=np.float64), pct))


def run_rate_test(output_csv_path: str, duration_sec: float, config: LoggerConfig, report_period_sec: float = 5.0):
    requested_period_sec = 1.0 / config.target_rate_hz

    os.makedirs(os.path.dirname(output_csv_path) or ".", exist_ok=True)

    with SMBus(config.bus_num) as i2c_bus:
        imu_reader = Mpu9250Reader(i2c_bus, config)
        who_am_i = imu_reader.check_connection()
        imu_reader.initialize()

        configured_rate_hz = imu_reader.configured_output_rate_hz()
        configured_period_sec = 1.0 / configured_rate_hz

        print(f"MPU WHO_AM_I: 0x{who_am_i:02X}")
        print(f"Requested rate:           {config.target_rate_hz:.3f} Hz")
        print(f"Requested period:         {requested_period_sec * 1000.0:.3f} ms")
        print(f"Configured sensor rate:   {configured_rate_hz:.3f} Hz")
        print(f"Configured sensor period: {configured_period_sec * 1000.0:.3f} ms")
        print(f"Duration:                 {duration_sec:.1f} s")
        print(f"Output CSV:               {output_csv_path}")
        print()

        warmup_end_sec = time.perf_counter() + config.warmup_sec
        while time.perf_counter() < warmup_end_sec:
            try:
                _ = imu_reader.read_sample()
            except OSError:
                pass

        start_monotonic_sec = time.perf_counter()
        next_sample_time_sec = start_monotonic_sec
        last_sample_monotonic_sec: Optional[float] = None
        sample_index = 0
        read_error_count = 0
        deadline_miss_count = 0
        late_by_more_than_half_period_count = 0
        max_consecutive_error_burst = 0
        current_consecutive_error_burst = 0
        last_report_time_sec = start_monotonic_sec

        dt_values = []
        read_duration_values = []
        schedule_lag_values = []

        with open(output_csv_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "sample_index",
                "timestamp_unix_sec",
                "timestamp_monotonic_sec",
                "dt_sec",
                "requested_period_sec",
                "configured_sensor_period_sec",
                "scheduled_time_monotonic_sec",
                "schedule_lag_sec",
                "deadline_missed",
                "late_by_more_than_half_period",
                "read_ok",
                "read_duration_sec",
                "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
                "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
                "temperature_c",
                "raw_accel_x", "raw_accel_y", "raw_accel_z",
                "raw_gyro_x", "raw_gyro_y", "raw_gyro_z",
                "error_text",
            ])

            while True:
                now_sec = time.perf_counter()
                elapsed_sec = now_sec - start_monotonic_sec
                if elapsed_sec >= duration_sec:
                    break

                sleep_sec = next_sample_time_sec - now_sec
                if sleep_sec > 0.0:
                    time.sleep(sleep_sec)

                scheduled_time_sec = next_sample_time_sec
                wake_time_sec = time.perf_counter()
                schedule_lag_sec = wake_time_sec - scheduled_time_sec
                deadline_missed = int(schedule_lag_sec > 0.0)
                late_half_period = int(schedule_lag_sec > 0.5 * requested_period_sec)

                if deadline_missed:
                    deadline_miss_count += 1
                if late_half_period:
                    late_by_more_than_half_period_count += 1

                try:
                    sample = imu_reader.read_sample()
                    current_consecutive_error_burst = 0

                    timestamp_monotonic_sec = sample["timestamp_monotonic_sec"]
                    if last_sample_monotonic_sec is None:
                        dt_sec = 0.0
                    else:
                        dt_sec = timestamp_monotonic_sec - last_sample_monotonic_sec
                        dt_values.append(dt_sec)
                    last_sample_monotonic_sec = timestamp_monotonic_sec

                    read_duration_values.append(sample["read_duration_sec"])
                    schedule_lag_values.append(schedule_lag_sec)

                    ax, ay, az = sample["accel_mps2"]
                    gx, gy, gz = sample["gyro_rps"]
                    raw_ax, raw_ay, raw_az = sample["raw_accel"]
                    raw_gx, raw_gy, raw_gz = sample["raw_gyro"]

                    writer.writerow([
                        sample_index,
                        f"{sample['timestamp_unix_sec']:.9f}",
                        f"{timestamp_monotonic_sec:.9f}",
                        f"{dt_sec:.9f}",
                        f"{requested_period_sec:.9f}",
                        f"{configured_period_sec:.9f}",
                        f"{scheduled_time_sec:.9f}",
                        f"{schedule_lag_sec:.9f}",
                        deadline_missed,
                        late_half_period,
                        1,
                        f"{sample['read_duration_sec']:.9f}",
                        f"{ax:.9f}", f"{ay:.9f}", f"{az:.9f}",
                        f"{gx:.9f}", f"{gy:.9f}", f"{gz:.9f}",
                        f"{sample['temperature_c']:.6f}",
                        raw_ax, raw_ay, raw_az,
                        raw_gx, raw_gy, raw_gz,
                        "",
                    ])
                except OSError as exc:
                    read_error_count += 1
                    current_consecutive_error_burst += 1
                    max_consecutive_error_burst = max(max_consecutive_error_burst, current_consecutive_error_burst)
                    writer.writerow([
                        sample_index,
                        "", "", "",
                        f"{requested_period_sec:.9f}",
                        f"{configured_period_sec:.9f}",
                        f"{scheduled_time_sec:.9f}",
                        f"{schedule_lag_sec:.9f}",
                        deadline_missed,
                        late_half_period,
                        0,
                        "",
                        "", "", "",
                        "", "", "",
                        "",
                        "", "", "",
                        "", "", "",
                        str(exc).replace("\n", " "),
                    ])

                sample_index += 1
                next_sample_time_sec += requested_period_sec

                report_now_sec = time.perf_counter()
                if report_now_sec - last_report_time_sec >= report_period_sec:
                    actual_elapsed_sec = report_now_sec - start_monotonic_sec
                    achieved_rate_hz = sample_index / max(actual_elapsed_sec, 1e-12)
                    dt_std_ms = statistics.pstdev(dt_values) * 1000.0 if len(dt_values) >= 2 else float("nan")
                    read_p99_ms = percentile_or_nan(read_duration_values, 99.0) * 1000.0
                    print(
                        f"samples={sample_index} | achieved={achieved_rate_hz:.2f} Hz | "
                        f"errors={read_error_count} | deadline_misses={deadline_miss_count} | "
                        f"dt_std={dt_std_ms:.3f} ms | read_p99={read_p99_ms:.3f} ms"
                    )
                    last_report_time_sec = report_now_sec

        total_elapsed_sec = time.perf_counter() - start_monotonic_sec
        achieved_rate_hz = sample_index / max(total_elapsed_sec, 1e-12)

        print("\nTest finished.")
        print(f"Total scheduled samples:              {sample_index}")
        print(f"Total elapsed time:                   {total_elapsed_sec:.3f} s")
        print(f"Average achieved schedule rate:       {achieved_rate_hz:.3f} Hz")
        print(f"Read errors:                          {read_error_count}")
        print(f"Deadline misses:                      {deadline_miss_count}")
        print(f"Late by > half a requested period:    {late_by_more_than_half_period_count}")
        print(f"Max consecutive read error burst:     {max_consecutive_error_burst}")
        if dt_values:
            dt_mean_ms = statistics.mean(dt_values) * 1000.0
            dt_std_ms = statistics.pstdev(dt_values) * 1000.0 if len(dt_values) > 1 else 0.0
            print(f"dt mean:                              {dt_mean_ms:.6f} ms")
            print(f"dt std:                               {dt_std_ms:.6f} ms")
            print(f"dt p99:                               {percentile_or_nan(dt_values, 99.0) * 1000.0:.6f} ms")
            print(f"dt p99.9:                             {percentile_or_nan(dt_values, 99.9) * 1000.0:.6f} ms")
            print(f"dt max:                               {max(dt_values) * 1000.0:.6f} ms")
        if read_duration_values:
            print(f"read duration mean:                   {statistics.mean(read_duration_values) * 1000.0:.6f} ms")
            print(f"read duration p99:                    {percentile_or_nan(read_duration_values, 99.0) * 1000.0:.6f} ms")
            print(f"read duration p99.9:                  {percentile_or_nan(read_duration_values, 99.9) * 1000.0:.6f} ms")
            print(f"read duration max:                    {max(read_duration_values) * 1000.0:.6f} ms")
        if schedule_lag_values:
            positive_lag = [x for x in schedule_lag_values if x > 0.0]
            print(f"schedule lag p99:                     {percentile_or_nan(schedule_lag_values, 99.0) * 1000.0:.6f} ms")
            print(f"schedule lag p99.9:                   {percentile_or_nan(schedule_lag_values, 99.9) * 1000.0:.6f} ms")
            if positive_lag:
                print(f"positive schedule lag mean:           {statistics.mean(positive_lag) * 1000.0:.6f} ms")



def main():
    parser = argparse.ArgumentParser(description="Log IMU timing and bus stability data to CSV.")
    parser.add_argument("--output", type=str, required=True, help="Output CSV path")
    parser.add_argument("--duration-sec", type=float, required=True, help="Test duration in seconds")
    parser.add_argument("--rate-hz", type=float, default=200.0, help="Requested polling rate in Hz")
    parser.add_argument("--bus", type=int, default=7, help="I2C bus number")
    args = parser.parse_args()

    config = LoggerConfig(
        bus_num=args.bus,
        target_rate_hz=args.rate_hz,
    )

    run_rate_test(
        output_csv_path=args.output,
        duration_sec=args.duration_sec,
        config=config,
    )


if __name__ == "__main__":
    main()
