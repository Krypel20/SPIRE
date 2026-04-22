"""
SPIRE IMU Driver — MPU6886 (TDK InvenSense)
6-axis IMU: 3-axis accel + 3-axis gyro (no magnetometer)

I2C address: 0x68 or 0x69 (AD0 pin selects)
Internal sample rate: 1000 Hz (configurable via divider)

Status: STUB — awaiting hardware for testing.
Register map based on MPU6886 datasheet. Similar to MPU6050/MPU6500.
"""

import time
import struct
import logging

import smbus2

from . import IMUDriver, register_driver, GYRO_RANGES_DPS, ACCEL_RANGES_G

log = logging.getLogger("spire.imu.mpu6886")

# ---------------------------------------------------------------------------
# Register map
# ---------------------------------------------------------------------------

REG_WHO_AM_I       = 0x75
REG_PWR_MGMT_1     = 0x6B
REG_PWR_MGMT_2     = 0x6C
REG_SMPLRT_DIV      = 0x19
REG_CONFIG          = 0x1A
REG_GYRO_CONFIG     = 0x1B
REG_ACCEL_CONFIG    = 0x1C
REG_ACCEL_CONFIG_2  = 0x1D
REG_ACCEL_XOUT_H    = 0x3B
REG_GYRO_XOUT_H     = 0x43
REG_TEMP_OUT_H      = 0x41

WHO_AM_I_VAL        = 0x19
INTERNAL_RATE_HZ    = 1000.0

ACCEL_SENSITIVITY = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}
GYRO_SENSITIVITY  = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

@register_driver("mpu6886")
class MPU6886Driver(IMUDriver):
    """Hardware driver for MPU6886 6-axis IMU."""

    DEFAULT_ADDRESS = 0x68

    def __init__(self, bus_num=1, address=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.gyro_fs = 1
        self.accel_fs = 1
        self.rate_div = 0

    def check_id(self):
        who = self.bus.read_byte_data(self.addr, REG_WHO_AM_I)
        if who != WHO_AM_I_VAL:
            raise RuntimeError(
                f"MPU6886 not found. WHO_AM_I=0x{who:02X}, "
                f"expected 0x{WHO_AM_I_VAL:02X}"
            )
        log.info(f"MPU6886 detected at 0x{self.addr:02X}")

    def initialize(self, gyro_fs=1, accel_fs=1, rate_div=0):
        self.gyro_fs = gyro_fs
        self.accel_fs = accel_fs
        self.rate_div = rate_div

        # Reset
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        # Wake up, auto-select clock
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        # Enable all axes
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_2, 0x00)

        # Sample rate divider
        self.bus.write_byte_data(self.addr, REG_SMPLRT_DIV, rate_div)

        # DLPF config (bandwidth ~92 Hz)
        self.bus.write_byte_data(self.addr, REG_CONFIG, 0x02)

        # Gyro config
        self.bus.write_byte_data(self.addr, REG_GYRO_CONFIG, gyro_fs << 3)

        # Accel config
        self.bus.write_byte_data(self.addr, REG_ACCEL_CONFIG, accel_fs << 3)
        self.bus.write_byte_data(self.addr, REG_ACCEL_CONFIG_2, 0x02)

        info = self.get_sensor_info()
        log.info(f"Gyro:  ±{info['gyro_range_dps']} °/s "
                 f"@ {info['actual_rate_hz']:.0f} Hz")
        log.info(f"Accel: ±{info['accel_range_g']} g "
                 f"@ {info['actual_rate_hz']:.0f} Hz")

    def read_accel_gyro(self):
        raw = self.bus.read_i2c_block_data(self.addr, REG_ACCEL_XOUT_H, 14)
        # Bytes: ax(2) ay(2) az(2) temp(2) gx(2) gy(2) gz(2)

        ax_raw = struct.unpack(">h", bytes(raw[0:2]))[0]
        ay_raw = struct.unpack(">h", bytes(raw[2:4]))[0]
        az_raw = struct.unpack(">h", bytes(raw[4:6]))[0]
        # raw[6:8] = temperature, skip
        gx_raw = struct.unpack(">h", bytes(raw[8:10]))[0]
        gy_raw = struct.unpack(">h", bytes(raw[10:12]))[0]
        gz_raw = struct.unpack(">h", bytes(raw[12:14]))[0]

        a_scale = ACCEL_SENSITIVITY[self.accel_fs]
        g_scale = GYRO_SENSITIVITY[self.gyro_fs]

        return (
            ax_raw / a_scale,
            ay_raw / a_scale,
            az_raw / a_scale,
            gx_raw / g_scale,
            gy_raw / g_scale,
            gz_raw / g_scale,
        )

    def enable_magnetometer(self):
        log.info("MPU6886 has no magnetometer")
        return False

    def read_magnetometer(self):
        return None

    def read_temperature(self):
        raw = self.bus.read_i2c_block_data(self.addr, REG_TEMP_OUT_H, 2)
        temp_raw = struct.unpack(">h", bytes(raw))[0]
        return temp_raw / 326.8 + 25.0

    def get_sensor_info(self):
        return {
            "name": "MPU6886",
            "gyro_range_dps": GYRO_RANGES_DPS[self.gyro_fs],
            "accel_range_g": ACCEL_RANGES_G[self.accel_fs],
            "actual_rate_hz": INTERNAL_RATE_HZ / (1 + self.rate_div),
            "has_magnetometer": False,
        }

    def close(self):
        self.bus.close()
