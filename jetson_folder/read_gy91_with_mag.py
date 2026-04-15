#!/usr/bin/env python3
from smbus2 import SMBus
import time

BUS = 7

MPU_ADDR = 0x68
BMP_ADDR = 0x76
MAG_ADDR = 0x0C   # AK8963 inside MPU-9250

# MPU9250 registers
MPU_PWR_MGMT_1   = 0x6B
MPU_INT_PIN_CFG  = 0x37
MPU_USER_CTRL    = 0x6A
MPU_WHO_AM_I     = 0x75
MPU_ACCEL_XOUT_H = 0x3B

# AK8963 registers
AK_WHO_AM_I = 0x00
AK_ST1      = 0x02
AK_HXL      = 0x03
AK_ST2      = 0x09
AK_CNTL1    = 0x0A
AK_ASAX     = 0x10

def s16(msb, lsb):
    v = (msb << 8) | lsb
    if v & 0x8000:
        v -= 65536
    return v

def s16_le(lsb, msb):
    v = (msb << 8) | lsb
    if v & 0x8000:
        v -= 65536
    return v

def read_mag_adjust(bus):
    # Fuse ROM access mode
    bus.write_byte_data(MAG_ADDR, AK_CNTL1, 0x00)
    time.sleep(0.01)
    bus.write_byte_data(MAG_ADDR, AK_CNTL1, 0x0F)
    time.sleep(0.01)

    asa = bus.read_i2c_block_data(MAG_ADDR, AK_ASAX, 3)

    # Sensitivity adjustment values from datasheet
    adj = [((x - 128) / 256.0) + 1.0 for x in asa]

    # Power down again
    bus.write_byte_data(MAG_ADDR, AK_CNTL1, 0x00)
    time.sleep(0.01)

    return adj

def init_mpu_and_mag(bus):
    # Wake MPU
    bus.write_byte_data(MPU_ADDR, MPU_PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # Make sure MPU master mode is off, so bypass works cleanly
    bus.write_byte_data(MPU_ADDR, MPU_USER_CTRL, 0x00)
    time.sleep(0.01)

    # Enable bypass: host I2C can access AK8963 directly at 0x0C
    bus.write_byte_data(MPU_ADDR, MPU_INT_PIN_CFG, 0x02)
    time.sleep(0.01)

    mpu_who = bus.read_byte_data(MPU_ADDR, MPU_WHO_AM_I)
    print(f"MPU WHO_AM_I: 0x{mpu_who:02X}")

    mag_who = bus.read_byte_data(MAG_ADDR, AK_WHO_AM_I)
    print(f"AK8963 WHO_AM_I: 0x{mag_who:02X}")

    if mag_who != 0x48:
        raise RuntimeError(
            f"Magnetometer not detected as AK8963. Got WHO_AM_I=0x{mag_who:02X}"
        )

    mag_adj = read_mag_adjust(bus)
    print(f"AK8963 sensitivity adjustment: {mag_adj}")

    # Power down
    bus.write_byte_data(MAG_ADDR, AK_CNTL1, 0x00)
    time.sleep(0.01)

    # Continuous measurement mode 2, 16-bit output, 100 Hz
    bus.write_byte_data(MAG_ADDR, AK_CNTL1, 0x16)
    time.sleep(0.01)

    return mag_adj

def read_mpu_accel_gyro(bus):
    data = bus.read_i2c_block_data(MPU_ADDR, MPU_ACCEL_XOUT_H, 14)

    ax = s16(data[0], data[1]) / 16384.0
    ay = s16(data[2], data[3]) / 16384.0
    az = s16(data[4], data[5]) / 16384.0

    temp_raw = s16(data[6], data[7])
    temp_c = (temp_raw / 333.87) + 21.0

    gx = s16(data[8], data[9]) / 131.0
    gy = s16(data[10], data[11]) / 131.0
    gz = s16(data[12], data[13]) / 131.0

    return ax, ay, az, gx, gy, gz, temp_c

def read_mag(bus, mag_adj):
    st1 = bus.read_byte_data(MAG_ADDR, AK_ST1)

    # Data ready?
    if (st1 & 0x01) == 0:
        return None

    # Read HXL..HZH + ST2 in one burst
    data = bus.read_i2c_block_data(MAG_ADDR, AK_HXL, 7)

    mx_raw = s16_le(data[0], data[1])
    my_raw = s16_le(data[2], data[3])
    mz_raw = s16_le(data[4], data[5])
    st2 = data[6]

    # Overflow
    if st2 & 0x08:
        return None

    # 16-bit mode scale = 0.15 uT/LSB
    mx_uT = mx_raw * 0.15 * mag_adj[0]
    my_uT = my_raw * 0.15 * mag_adj[1]
    mz_uT = mz_raw * 0.15 * mag_adj[2]

    return mx_raw, my_raw, mz_raw, mx_uT, my_uT, mz_uT

def main():
    with SMBus(BUS) as bus:
        mag_adj = init_mpu_and_mag(bus)

        while True:
            ax, ay, az, gx, gy, gz, temp_c = read_mpu_accel_gyro(bus)
            mag = read_mag(bus, mag_adj)

            print(f"ACC  g   : {ax:+.3f} {ay:+.3f} {az:+.3f}")
            print(f"GYRO dps : {gx:+.3f} {gy:+.3f} {gz:+.3f}")
            print(f"TEMP C   : {temp_c:.2f}")

            if mag is None:
                print("MAG       : no new data")
            else:
                mx_raw, my_raw, mz_raw, mx_uT, my_uT, mz_uT = mag
                print(f"MAG raw   : {mx_raw:+6d} {my_raw:+6d} {mz_raw:+6d}")
                print(f"MAG uT    : {mx_uT:+8.2f} {my_uT:+8.2f} {mz_uT:+8.2f}")

            print("-" * 50)
            time.sleep(0.5)

if __name__ == "__main__":
    main()