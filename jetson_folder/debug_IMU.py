#!/usr/bin/env python3
from smbus2 import SMBus
import time

BUS = 1          # change to 7 for the other board
MPU = 0x68
MAG = 0x0C

# MPU registers
WHO_AM_I     = 0x75
PWR_MGMT_1   = 0x6B
USER_CTRL    = 0x6A
INT_PIN_CFG  = 0x37
I2C_MST_CTRL = 0x24

I2C_SLV0_ADDR = 0x25
I2C_SLV0_REG  = 0x26
I2C_SLV0_CTRL = 0x27
EXT_SENS_DATA_00 = 0x49

def rb(bus, reg):
    return bus.read_byte_data(MPU, reg)

def wb(bus, reg, val):
    bus.write_byte_data(MPU, reg, val)

def try_direct_mpu(bus):
    who = rb(bus, WHO_AM_I)
    print(f"MPU WHO_AM_I: 0x{who:02X}")

def read_ext_sens(bus, slave_addr, slave_reg, length=1, delay=0.05):
    # Set slave 0 to read from external I2C slave
    # Bit7=1 means read, lower 7 bits are address
    wb(bus, I2C_SLV0_ADDR, 0x80 | (slave_addr & 0x7F))
    wb(bus, I2C_SLV0_REG, slave_reg)
    wb(bus, I2C_SLV0_CTRL, 0x80 | (length & 0x0F))  # enable + length
    time.sleep(delay)

    data = bus.read_i2c_block_data(MPU, EXT_SENS_DATA_00, length)

    # disable slave after read
    wb(bus, I2C_SLV0_CTRL, 0x00)
    return data

def main():
    with SMBus(BUS) as bus:
        print(f"Using bus {BUS}")

        # wake device
        wb(bus, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        try_direct_mpu(bus)

        # turn OFF bypass, turn ON internal I2C master
        wb(bus, INT_PIN_CFG, 0x00)
        time.sleep(0.01)

        # USER_CTRL:
        # bit 5 = I2C_MST_EN
        wb(bus, USER_CTRL, 0x20)
        time.sleep(0.01)

        # set I2C master clock
        wb(bus, I2C_MST_CTRL, 0x0D)
        time.sleep(0.01)

        print("Trying internal-master read from candidate AK8963 at 0x0C...")

        try:
            data = read_ext_sens(bus, MAG, 0x00, 1)
            print(f"Read via EXT_SENS_DATA_00: 0x{data[0]:02X}")

            if data[0] == 0x48:
                print("AK8963 detected.")
            elif data[0] in (0x00, 0xFF):
                print("No convincing AK8963 response.")
            else:
                print("Got a byte back, but it does not match AK8963 WHO_AM_I=0x48.")
        except OSError as e:
            print(f"Internal-master read failed: {e}")

if __name__ == "__main__":
    main()