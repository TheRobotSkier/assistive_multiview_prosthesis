"""
read_gy91_all_sensors.py

Read sensor data from a GY-91 module connected to a Jetson Orin Nano over I2C.

This script reads:
- MPU-9250 accelerometer
- MPU-9250 gyroscope
- MPU-9250 internal temperature
- BMP280 temperature
- BMP280 pressure

Hardware setup used:
- Jetson pin 1  -> GY-91 VIN
- Jetson pin 6  -> GY-91 GND
- Jetson pin 3  -> GY-91 SDA
- Jetson pin 5  -> GY-91 SCL

Detected I2C addresses:
- 0x68 = MPU-9250
- 0x76 = BMP280

References
----------
1) MPU-9250 Product Specification
   Used for:
   - accelerometer sensitivity values (for example 16384 LSB/g at ±2 g)
   - gyroscope sensitivity values (for example 131 LSB/(deg/s) at ±250 dps)
   - temperature conversion formula
   URL:
   https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU9250REV1.0.pdf

2) MPU-9250 Register Map and Descriptions
   Used for:
   - register addresses such as PWR_MGMT_1, GYRO_CONFIG, ACCEL_CONFIG
   - meaning of configuration values written during initialization
   URL:
   https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf

3) Bosch BMP280 Datasheet
   Used for:
   - BMP280 chip ID (0x58)
   - calibration register block starting at 0x88
   - raw data registers starting at 0xF7
   - CTRL_MEAS and CONFIG register meanings
   - temperature and pressure compensation formulas
   URL:
   https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf

Notes
-----
- This script uses I2C bus 7, which is typically the I2C bus for Jetson
  header pins 3 and 5 on the Orin Nano dev kit.
- Pressure is printed in hPa.
- Acceleration is printed in g.
- Angular velocity is printed in degrees per second (dps).
"""

import time
from smbus2 import SMBus

# ============================================================
# Jetson / I2C configuration
# ============================================================
# On the Jetson Orin Nano dev kit, pins 3/5 on the 40-pin header
# are typically connected to I2C bus 7.
BUS_NUM = 7

# I2C addresses found earlier with:
#   sudo i2cdetect -y -r 7
#
# On this board:
#   0x68 = MPU-9250 IMU
#   0x76 = BMP280 pressure/temperature sensor
MPU_ADDR = 0x68
BMP_ADDR = 0x76


# ============================================================
# MPU-9250 register addresses
# ============================================================
# These register addresses come from the MPU-9250 register map.
# We write configuration values to these registers during setup,
# then read measurement data starting at ACCEL_XOUT_H.
MPU_PWR_MGMT_1    = 0x6B
MPU_SMPLRT_DIV    = 0x19
MPU_CONFIG        = 0x1A
MPU_GYRO_CONFIG   = 0x1B
MPU_ACCEL_CONFIG  = 0x1C
MPU_ACCEL_CONFIG2 = 0x1D
MPU_ACCEL_XOUT_H  = 0x3B


def s16(msb, lsb):
    """
    Combine two 8-bit bytes into one signed 16-bit integer.

    The sensor sends many values as:
      - one high byte (MSB)
      - one low byte (LSB)

    This function converts the two bytes into a normal Python int.
    """
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value


class MPU9250:
    """
    Minimal driver for reading accelerometer, gyroscope, and
    internal temperature data from the MPU-9250 over I2C.
    """

    def __init__(self, bus, addr):
        self.bus = bus
        self.addr = addr

    def write8(self, reg, value):
        """Write one byte to one sensor register."""
        self.bus.write_byte_data(self.addr, reg, value)

    def read_block(self, reg, length):
        """Read multiple bytes starting from register 'reg'."""
        return self.bus.read_i2c_block_data(self.addr, reg, length)

    def init(self):
        """
        Initialize the MPU-9250.

        Configuration used here:
        - wake the device up
        - sample rate divider = 4
        - digital low-pass filter enabled
        - gyro full-scale range = ±250 deg/s
        - accel full-scale range = ±2 g

        Why these scale factors later?
        - GYRO_CONFIG = 0x00 selects ±250 deg/s
        - ACCEL_CONFIG = 0x00 selects ±2 g
        """
        self.write8(MPU_PWR_MGMT_1, 0x00)   # Wake up sensor
        time.sleep(0.1)

        # Sample rate divider:
        # sample_rate = internal_rate / (1 + SMPLRT_DIV)
        # Here: 1 kHz / (1 + 4) = 200 Hz when DLPF is enabled.
        self.write8(MPU_SMPLRT_DIV, 0x04)

        # Digital low-pass filter setting.
        self.write8(MPU_CONFIG, 0x03)

        # 0x00 => FS_SEL = 0 => gyro full-scale range = ±250 deg/s
        self.write8(MPU_GYRO_CONFIG, 0x00)

        # 0x00 => AFS_SEL = 0 => accel full-scale range = ±2 g
        self.write8(MPU_ACCEL_CONFIG, 0x00)

        self.write8(MPU_ACCEL_CONFIG2, 0x03)

    def read(self):
        """
        Read one full set of IMU data.

        The MPU-9250 returns 14 bytes in this order:
          accel X, accel Y, accel Z,
          temperature,
          gyro X, gyro Y, gyro Z

        Conversion factors used below:

        Accelerometer:
        - For ±2 g range, sensitivity = 16384 LSB/g
        - So: acceleration_g = raw_value / 16384.0

        Gyroscope:
        - For ±250 deg/s range, sensitivity = 131 LSB/(deg/s)
        - So: angular_rate_dps = raw_value / 131.0

        Temperature:
        - temp_c = (temp_raw / 333.87) + 21.0
        """
        # Read 14 bytes starting from ACCEL_XOUT_H
        data = self.read_block(MPU_ACCEL_XOUT_H, 14)

        # Accelerometer raw values -> g
        # 16384 LSB/g because ACCEL_CONFIG = ±2 g
        ax = s16(data[0], data[1]) / 16384.0
        ay = s16(data[2], data[3]) / 16384.0
        az = s16(data[4], data[5]) / 16384.0

        # Internal temperature raw value
        temp_raw = s16(data[6], data[7])

        # Gyroscope raw values -> degrees per second
        # 131 LSB/(deg/s) because GYRO_CONFIG = ±250 deg/s
        gx = s16(data[8], data[9]) / 131.0
        gy = s16(data[10], data[11]) / 131.0
        gz = s16(data[12], data[13]) / 131.0

        # MPU-9250 temperature conversion formula
        temp_c = (temp_raw / 333.87) + 21.0

        return ax, ay, az, gx, gy, gz, temp_c


# ============================================================
# BMP280 register addresses
# ============================================================
# The BMP280 stores calibration constants in internal registers
# beginning at 0x88. Pressure and temperature measurement data
# begins at 0xF7.
BMP280_CALIB00   = 0x88
BMP280_CHIPID    = 0xD0
BMP280_CTRL_MEAS = 0xF4
BMP280_CONFIG    = 0xF5
BMP280_PRESS_MSB = 0xF7


class BMP280:
    """
    Minimal driver for the BMP280 temperature and pressure sensor.
    """

    def __init__(self, bus, addr):
        self.bus = bus
        self.addr = addr
        self.t_fine = 0
        self.cal = {}

    def read8(self, reg):
        """Read one byte from one BMP280 register."""
        return self.bus.read_byte_data(self.addr, reg)

    def write8(self, reg, value):
        """Write one byte to one BMP280 register."""
        self.bus.write_byte_data(self.addr, reg, value)

    def read_block(self, reg, length):
        """Read multiple consecutive bytes from the BMP280."""
        return self.bus.read_i2c_block_data(self.addr, reg, length)

    def u16(self, data, i):
        """Read an unsigned 16-bit little-endian value."""
        return data[i] | (data[i + 1] << 8)

    def s16le(self, data, i):
        """Read a signed 16-bit little-endian value."""
        v = self.u16(data, i)
        if v & 0x8000:
            v -= 65536
        return v

    def init(self):
        """
        Initialize the BMP280.

        Steps:
        1. Read chip ID to confirm sensor identity
        2. Read factory calibration constants
        3. Configure oversampling and normal mode
        4. Configure standby time and digital filtering

        Notes:
        - BMP280 chip ID should be 0x58
        - Calibration values from 0x88..0x9F are required for
          converting raw ADC readings into real temperature and pressure
        """
        chip_id = self.read8(BMP280_CHIPID)
        if chip_id != 0x58:
            raise RuntimeError(f"Unexpected BMP280 chip ID: 0x{chip_id:02X}")

        calib = self.read_block(BMP280_CALIB00, 24)

        # Temperature calibration constants
        self.cal["T1"] = self.u16(calib, 0)
        self.cal["T2"] = self.s16le(calib, 2)
        self.cal["T3"] = self.s16le(calib, 4)

        # Pressure calibration constants
        self.cal["P1"] = self.u16(calib, 6)
        self.cal["P2"] = self.s16le(calib, 8)
        self.cal["P3"] = self.s16le(calib, 10)
        self.cal["P4"] = self.s16le(calib, 12)
        self.cal["P5"] = self.s16le(calib, 14)
        self.cal["P6"] = self.s16le(calib, 16)
        self.cal["P7"] = self.s16le(calib, 18)
        self.cal["P8"] = self.s16le(calib, 20)
        self.cal["P9"] = self.s16le(calib, 22)

        # CTRL_MEAS = 0x57
        #
        # Bit fields:
        #   osrs_t[2:0] = 010 => temperature oversampling x2
        #   osrs_p[2:0] = 101 => pressure oversampling x16
        #   mode[1:0]   = 11  => normal mode
        self.write8(BMP280_CTRL_MEAS, 0x57)

        # CONFIG = 0x90
        #
        # Bit fields:
        #   t_sb[2:0]   = 100 => standby time 500 ms
        #   filter[2:0] = 100 => IIR filter coefficient 16
        #   spi3w_en    = 0   => 4-wire SPI disabled
        self.write8(BMP280_CONFIG, 0x90)

        time.sleep(0.1)

    def compensate_temp(self, adc_T):
        """
        Convert raw BMP280 temperature into degrees C.

        This uses Bosch's compensation formula from the BMP280 datasheet.
        The intermediate value 't_fine' must be saved because it is also
        used later in the pressure compensation formula.
        """
        c = self.cal
        var1 = (((adc_T / 16384.0) - (c["T1"] / 1024.0)) * c["T2"])
        var2 = ((((adc_T / 131072.0) - (c["T1"] / 8192.0)) ** 2) * c["T3"])
        self.t_fine = var1 + var2
        return self.t_fine / 5120.0

    def compensate_press(self, adc_P):
        """
        Convert raw BMP280 pressure into Pascals (Pa).

        This follows Bosch's official compensation formula and uses
        't_fine' computed during temperature compensation.
        """
        c = self.cal

        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * c["P6"] / 32768.0
        var2 = var2 + var1 * c["P5"] * 2.0
        var2 = (var2 / 4.0) + (c["P4"] * 65536.0)

        var1 = (c["P3"] * var1 * var1 / 524288.0 + c["P2"] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * c["P1"]

        if var1 == 0:
            return None  # Avoid division by zero

        p = 1048576.0 - adc_P
        p = ((p - (var2 / 4096.0)) * 6250.0) / var1
        var1 = c["P9"] * p * p / 2147483648.0
        var2 = p * c["P8"] / 32768.0
        p = p + (var1 + var2 + c["P7"]) / 16.0

        return p  # Pressure in Pascals

    def read(self):
        """
        Read one full set of BMP280 data.

        Raw data layout:
          pressure_msb, pressure_lsb, pressure_xlsb,
          temp_msb,     temp_lsb,     temp_xlsb

        The BMP280 stores pressure and temperature as 20-bit raw values.
        """
        data = self.read_block(BMP280_PRESS_MSB, 6)

        # Reconstruct 20-bit raw pressure and temperature ADC values
        adc_P = ((data[0] << 12) | (data[1] << 4) | (data[2] >> 4))
        adc_T = ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4))

        temp_c = self.compensate_temp(adc_T)
        press_pa = self.compensate_press(adc_P)

        # Convert Pascals -> hectoPascals
        # 1 hPa = 100 Pa
        return temp_c, press_pa / 100.0


# ============================================================
# Main program
# ============================================================
# Open the I2C bus, initialize both sensors, then read and print
# values forever until the user stops the program.
with SMBus(BUS_NUM) as bus:
    mpu = MPU9250(bus, MPU_ADDR)
    bmp = BMP280(bus, BMP_ADDR)

    mpu.init()
    bmp.init()

    while True:
        ax, ay, az, gx, gy, gz, imu_temp = mpu.read()
        bmp_temp, pressure_hpa = bmp.read()

        print(f"ACC[g]    {ax:+.3f} {ay:+.3f} {az:+.3f}")
        print(f"GYRO[dps] {gx:+.3f} {gy:+.3f} {gz:+.3f}")
        print(f"IMU temp  {imu_temp:.2f} C")
        print(f"BMP temp  {bmp_temp:.2f} C")
        print(f"Pressure  {pressure_hpa:.2f} hPa")
        print("-" * 50)

        time.sleep(0.5)