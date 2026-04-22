"""
SPIRE IMU Driver — ICM-20948 (TDK InvenSense)
9-axis IMU: 3-axis accel + 3-axis gyro + AK09916 magnetometer

I2C address: 0x68 or 0x69 (default 0x69)
Internal sample rate: 1125 Hz (configurable via divider)
"""

import time
import struct
import logging

import smbus2

from . import IMUDriver, register_driver, GYRO_RANGES_DPS, ACCEL_RANGES_G

log = logging.getLogger("spire.imu.icm20948")

# ---------------------------------------------------------------------------
# Register map
# ---------------------------------------------------------------------------

# Bank 0
REG_WHO_AM_I         = 0x00
REG_USER_CTRL        = 0x03
REG_LP_CONFIG        = 0x05
REG_PWR_MGMT_1       = 0x06
REG_PWR_MGMT_2       = 0x07
REG_INT_PIN_CFG      = 0x0F
REG_INT_ENABLE_1     = 0x11
REG_ACCEL_XOUT_H     = 0x2D
REG_GYRO_XOUT_H      = 0x33
REG_TEMP_OUT_H       = 0x39
REG_BANK_SEL         = 0x7F

# Bank 2
REG_GYRO_SMPLRT_DIV  = 0x00
REG_GYRO_CONFIG_1    = 0x01
REG_ACCEL_SMPLRT_DIV_1 = 0x10
REG_ACCEL_SMPLRT_DIV_2 = 0x11
REG_ACCEL_CONFIG     = 0x14

# Magnetometer (AK09916)
AK_I2C_ADDR          = 0x0C
AK_WHO_AM_I          = 0x01
AK_ST1               = 0x10
AK_HXL               = 0x11
AK_CNTL2             = 0x31
AK_CNTL3             = 0x32

# Constants
WHO_AM_I_VAL         = 0xEA
INTERNAL_RATE_HZ     = 1125.0

ACCEL_SENSITIVITY = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}
GYRO_SENSITIVITY  = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}
MAG_SCALE         = 0.15  # µT/LSB


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

@register_driver("icm20948")
class ICM20948Driver(IMUDriver):
    """Hardware driver for ICM-20948 9-axis IMU."""

    DEFAULT_ADDRESS = 0x69

    def __init__(self, bus_num=1, address=0x69):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.current_bank = -1
        self.gyro_fs = 1
        self.accel_fs = 1
        self.rate_div = 0
        self.mag_enabled = False

    # --- Low-level I2C ---

    def _select_bank(self, bank):
        if bank != self.current_bank:
            self.bus.write_byte_data(self.addr, REG_BANK_SEL, bank << 4)
            self.current_bank = bank

    def _read_byte(self, bank, reg):
        self._select_bank(bank)
        return self.bus.read_byte_data(self.addr, reg)

    def _write_byte(self, bank, reg, val):
        self._select_bank(bank)
        self.bus.write_byte_data(self.addr, reg, val)

    def _read_bytes(self, bank, reg, count):
        self._select_bank(bank)
        return self.bus.read_i2c_block_data(self.addr, reg, count)

    # --- IMUDriver interface ---

    def check_id(self):
        who = self._read_byte(0, REG_WHO_AM_I)
        if who != WHO_AM_I_VAL:
            raise RuntimeError(
                f"ICM-20948 not found. WHO_AM_I=0x{who:02X}, "
                f"expected 0x{WHO_AM_I_VAL:02X}"
            )
        log.info(f"ICM-20948 detected at 0x{self.addr:02X}")

    def initialize(self, gyro_fs=1, accel_fs=1, rate_div=0):
        self.gyro_fs = gyro_fs
        self.accel_fs = accel_fs
        self.rate_div = rate_div

        # Reset
        self._write_byte(0, REG_PWR_MGMT_1, 0x81)
        time.sleep(0.1)
        self._write_byte(0, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        # Enable all axes
        self._write_byte(0, REG_PWR_MGMT_2, 0x00)

        # Gyro config (Bank 2)
        self._write_byte(2, REG_GYRO_SMPLRT_DIV, rate_div)
        self._write_byte(2, REG_GYRO_CONFIG_1, (gyro_fs << 1) | 0x01)

        # Accel config (Bank 2)
        self._write_byte(2, REG_ACCEL_SMPLRT_DIV_1, (rate_div >> 8) & 0x0F)
        self._write_byte(2, REG_ACCEL_SMPLRT_DIV_2, rate_div & 0xFF)
        self._write_byte(2, REG_ACCEL_CONFIG, (accel_fs << 1) | 0x01)

        # Back to bank 0
        self._select_bank(0)

        info = self.get_sensor_info()
        log.info(f"Gyro:  ±{info['gyro_range_dps']} °/s "
                 f"@ {info['actual_rate_hz']:.0f} Hz")
        log.info(f"Accel: ±{info['accel_range_g']} g "
                 f"@ {info['actual_rate_hz']:.0f} Hz")

    def read_accel_gyro(self):
        raw = self._read_bytes(0, REG_ACCEL_XOUT_H, 12)

        ax_raw = struct.unpack(">h", bytes(raw[0:2]))[0]
        ay_raw = struct.unpack(">h", bytes(raw[2:4]))[0]
        az_raw = struct.unpack(">h", bytes(raw[4:6]))[0]
        gx_raw = struct.unpack(">h", bytes(raw[6:8]))[0]
        gy_raw = struct.unpack(">h", bytes(raw[8:10]))[0]
        gz_raw = struct.unpack(">h", bytes(raw[10:12]))[0]

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
        self._write_byte(0, REG_INT_PIN_CFG, 0x02)
        time.sleep(0.01)

        try:
            who = self.bus.read_byte_data(AK_I2C_ADDR, AK_WHO_AM_I)
            log.info(f"AK09916 magnetometer detected "
                     f"(WHO_AM_I=0x{who:02X})")
        except OSError:
            log.warning("AK09916 magnetometer not found")
            return False

        self.bus.write_byte_data(AK_I2C_ADDR, AK_CNTL3, 0x01)
        time.sleep(0.01)
        self.bus.write_byte_data(AK_I2C_ADDR, AK_CNTL2, 0x08)
        time.sleep(0.01)

        self.mag_enabled = True
        log.info("Magnetometer: continuous mode @ 100 Hz")
        return True

    def read_magnetometer(self):
        if not self.mag_enabled:
            return None
        try:
            st1 = self.bus.read_byte_data(AK_I2C_ADDR, AK_ST1)
            if not (st1 & 0x01):
                return None

            raw = self.bus.read_i2c_block_data(AK_I2C_ADDR, AK_HXL, 8)

            mx = struct.unpack("<h", bytes(raw[0:2]))[0] * MAG_SCALE
            my = struct.unpack("<h", bytes(raw[2:4]))[0] * MAG_SCALE
            mz = struct.unpack("<h", bytes(raw[4:6]))[0] * MAG_SCALE

            return mx, my, mz
        except OSError:
            return None

    def read_temperature(self):
        raw = self._read_bytes(0, REG_TEMP_OUT_H, 2)
        temp_raw = struct.unpack(">h", bytes(raw))[0]
        return (temp_raw - 21.0) / 333.87 + 21.0

    def get_sensor_info(self):
        return {
            "name": "ICM-20948",
            "gyro_range_dps": GYRO_RANGES_DPS[self.gyro_fs],
            "accel_range_g": ACCEL_RANGES_G[self.accel_fs],
            "actual_rate_hz": INTERNAL_RATE_HZ / (1 + self.rate_div),
            "has_magnetometer": True,
        }

    def close(self):
        self.bus.close()
