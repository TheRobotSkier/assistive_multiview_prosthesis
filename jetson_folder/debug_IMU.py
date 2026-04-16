#!/usr/bin/env python3
import time
from smbus2 import SMBus

BUS_NUM = 7          # change if needed
MPU_ADDR = 0x68
AK_ADDR  = 0x0C

# MPU registers
PWR_MGMT_1  = 0x6B
USER_CTRL   = 0x6A
INT_PIN_CFG = 0x37
WHO_AM_I_MPU = 0x75

# AK8963 registers
WHO_AM_I_AK   = 0x00
AK_ST1        = 0x02
AK_XOUT_L     = 0x03
AK_CNTL1      = 0x0A
AK_ASAX       = 0x10
AK_ST2        = 0x09

def read_u8(bus, addr, reg):
    return bus.read_byte_data(addr, reg)

def write_u8(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val)
    time.sleep(0.01)

def read_block(bus, addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)

def to_int16(lo, hi):
    value = (hi << 8) | lo
    if value & 0x8000:
        value -= 65536
    return value

def setup_mpu_bypass(bus):
    # Wake MPU
    write_u8(bus, MPU_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    who = read_u8(bus, MPU_ADDR, WHO_AM_I_MPU)
    print(f"MPU WHO_AM_I: 0x{who:02X}")

    # Disable internal I2C master, enable bypass
    write_u8(bus, MPU_ADDR, USER_CTRL, 0x00)
    write_u8(bus, MPU_ADDR, INT_PIN_CFG, 0x02)
    time.sleep(0.05)

    return who

def init_ak8963(bus):
    who = read_u8(bus, AK_ADDR, WHO_AM_I_AK)
    print(f"AK8963 WHO_AM_I: 0x{who:02X}")
    if who != 0x48:
        raise RuntimeError("AK8963 not detected at 0x0C")

    # Power down
    write_u8(bus, AK_ADDR, AK_CNTL1, 0x00)
    time.sleep(0.01)

    # Enter Fuse ROM access mode to read sensitivity adjustment
    write_u8(bus, AK_ADDR, AK_CNTL1, 0x0F)
    time.sleep(0.01)

    asa = read_block(bus, AK_ADDR, AK_ASAX, 3)
    adj = [((x - 128) / 256.0) + 1.0 for x in asa]
    print(f"AK8963 ASA: {asa}, adj={adj}")

    # Power down again
    write_u8(bus, AK_ADDR, AK_CNTL1, 0x00)
    time.sleep(0.01)

    # Continuous measurement mode 2, 16-bit output, 100 Hz
    write_u8(bus, AK_ADDR, AK_CNTL1, 0x16)
    time.sleep(0.01)

    return adj

def read_mag(bus, adj):
    st1 = read_u8(bus, AK_ADDR, AK_ST1)
    if (st1 & 0x01) == 0:
        return None

    data = read_block(bus, AK_ADDR, AK_XOUT_L, 7)
    mx = to_int16(data[0], data[1])
    my = to_int16(data[2], data[3])
    mz = to_int16(data[4], data[5])
    st2 = data[6]

    # Check magnetic sensor overflow
    if st2 & 0x08:
        print("Mag overflow")
        return None

    # Convert to adjusted raw counts
    mx_adj = mx * adj[0]
    my_adj = my * adj[1]
    mz_adj = mz * adj[2]

    return mx_adj, my_adj, mz_adj

def main():
    with SMBus(BUS_NUM) as bus:
        who = setup_mpu_bypass(bus)

        if who not in (0x71, 0x73):
            print("Warning: IMU is not identifying as MPU-9250/9255.")
            print("If WHO_AM_I is 0x70, it is likely MPU-6500 and has no AK8963.")
            return

        adj = init_ak8963(bus)

        print("Reading magnetometer. Move the board around...")
        while True:
            mag = read_mag(bus, adj)
            if mag is not None:
                mx, my, mz = mag
                print(f"MAG: X={mx:8.2f}  Y={my:8.2f}  Z={mz:8.2f}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()