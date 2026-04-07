from smbus2 import SMBus
import time

BUS = 7
MPU_ADDR = 0x68
BMP_ADDR = 0x76

# MPU9250 registers
MPU_PWR_MGMT_1 = 0x6B
MPU_ACCEL_XOUT_H = 0x3B

def s16(msb, lsb):
    v = (msb << 8) | lsb
    if v & 0x8000:
        v -= 65536
    return v

with SMBus(BUS) as bus:
    # Wake up MPU9250
    bus.write_byte_data(MPU_ADDR, MPU_PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    while True:
        # Read accel/temp/gyro block
        data = bus.read_i2c_block_data(MPU_ADDR, MPU_ACCEL_XOUT_H, 14)

        ax = s16(data[0], data[1]) / 16384.0
        ay = s16(data[2], data[3]) / 16384.0
        az = s16(data[4], data[5]) / 16384.0
        temp_raw = s16(data[6], data[7])
        gx = s16(data[8], data[9]) / 131.0
        gy = s16(data[10], data[11]) / 131.0
        gz = s16(data[12], data[13]) / 131.0
        temp_c = (temp_raw / 333.87) + 21.0

        print(f"ACC  g : {ax:+.3f} {ay:+.3f} {az:+.3f}")
        print(f"GYRO dps: {gx:+.3f} {gy:+.3f} {gz:+.3f}")
        print(f"IMU temp: {temp_c:.2f} C")
        print("-" * 40)

        time.sleep(0.5)