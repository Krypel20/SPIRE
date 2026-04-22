"""
SPIRE IMU Driver — LSM9DS1 (STMicroelectronics)
9-axis IMU: 3-axis accel + 3-axis gyro + 3-axis magnetometer

Unlike ICM-20948 and MPU6886, the LSM9DS1 uses TWO I2C addresses:
  - Accel/Gyro: 0x6A or 0x6B (default 0x6B)
  - Magnetometer: 0x1C or 0x1E (default 0x1E)

Internal sample rate: up to 952 Hz (accel/gyro), 80 Hz (mag)

Status: STUB — awaiting hardware for testing.
Register map based on LSM9DS1 datasheet.
"""

import time
import struct
import logging

import smbus2

from . import IMUDriver, register_driver, GYRO_RANGES_DPS, ACCEL_RANGES_G

log = logging.getLogger("spire.imu.lsm9ds1")

# ---------------------------------------------------------------------------
# Register map — Accel/Gyro (AG)
# ---------------------------------------------------------------------------

REG_WHO_AM_I_AG    = 0x0F
REG_CTRL_REG1_G    = 0x10  # Gyro ODR + FS + BW
REG_CTRL_REG6_XL   = 0x20  # Accel ODR + FS + BW
REG_CTRL_REG8      = 0x22  # Reset
REG_OUT_X_L_G      = 0x18  # Gyro data start
REG_OUT_X_L_XL     = 0x28  # Accel data start
REG_OUT_TEMP_L     = 0x15  # Temperature

WHO_AM_I_AG_VAL    = 0x68

# ---------------------------------------------------------------------------
# Register map — Magnetometer (M)
# ---------------------------------------------------------------------------

REG_WHO_AM_I_M     = 0x0F
REG_CTRL_REG1_M    = 0x20  # Mag ODR + mode
REG_CTRL_REG2_M    = 0x21  # Mag FS
REG_CTRL_REG3_M    = 0x22  # Mag operating mode
REG_STATUS_REG_M   = 0x27
REG_OUT_X_L_M      = 0x28  # Mag data start

WHO_AM_I_M_VAL     = 0x3D

# ---------------------------------------------------------------------------
# Scale factors
# ---------------------------------------------------------------------------

# Gyro: index maps to CTRL_REG1_G FS bits
# 0=245, 1=500, 2=2000 dps (no 1000 option on LSM9DS1)
GYRO_FS_BITS = {0: 0b00, 1: 0b01, 2: 0b11}
GYRO_SENSITIVITY_MDPS = {0: 8.75, 1: 17.50, 2: 70.0}  # mdps/LSB
LSM_GYRO_RANGES = [245, 500, 2000]

# Accel: 0=2g, 1=4g, 2=8g, 3=16g
ACCEL_FS_BITS = {0: 0b00, 1: 0b10, 2: 0b11, 3: 0b01}
ACCEL_SENSITIVITY_MG = {0: 0.061, 1: 0.122, 2: 0.244, 3: 0.732}  # mg/LSB

# Magnetometer: fixed ±4 gauss
MAG_SENSITIVITY = 0.14  # mgauss/LSB


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

@register_driver("lsm9ds1")
class LSM9DS1Driver(IMUDriver):
    """Hardware driver for LSM9DS1 9-axis IMU."""

    DEFAULT_ADDRESS = 0x6B  # Accel/Gyro address

    def __init__(self, bus_num=1, address=0x6B, mag_address=0x1E):
        self.bus = smbus2.SMBus(bus_num)
        self.addr_ag = address
        self.addr_m = mag_address
        self.gyro_fs = 1
        self.accel_fs = 1
        self.rate_div = 0
        self.mag_enabled = False

    def check_id(self):
        who_ag = self.bus.read_byte_data(self.addr_ag, REG_WHO_AM_I_AG)
        if who_ag != WHO_AM_I_AG_VAL:
            raise RuntimeError(
                f"LSM9DS1 accel/gyro not found. "
                f"WHO_AM_I=0x{who_ag:02X}, expected 0x{WHO_AM_I_AG_VAL:02X}"
            )
        log.info(f"LSM9DS1 accel/gyro detected at 0x{self.addr_ag:02X}")

        try:
            who_m = self.bus.read_byte_data(self.addr_m, REG_WHO_AM_I_M)
            if who_m == WHO_AM_I_M_VAL:
                log.info(f"LSM9DS1 magnetometer detected at "
                         f"0x{self.addr_m:02X}")
            else:
                log.warning(f"LSM9DS1 mag WHO_AM_I=0x{who_m:02X}, "
                            f"expected 0x{WHO_AM_I_M_VAL:02X}")
        except OSError:
            log.warning("LSM9DS1 magnetometer not found at "
                        f"0x{self.addr_m:02X}")

    def initialize(self, gyro_fs=1, accel_fs=1, rate_div=0):
        self.gyro_fs = min(gyro_fs, 2)  # LSM9DS1 has only 3 gyro ranges
        self.accel_fs = accel_fs
        self.rate_div = rate_div

        # Software reset
        self.bus.write_byte_data(self.addr_ag, REG_CTRL_REG8, 0x05)
        time.sleep(0.1)

        # Gyro: 476 Hz ODR + full-scale selection
        # ODR bits [7:5]: 100 = 476 Hz
        gyro_fs_bits = GYRO_FS_BITS.get(self.gyro_fs, 0b01)
        self.bus.write_byte_data(
            self.addr_ag, REG_CTRL_REG1_G,
            (0b100 << 5) | (gyro_fs_bits << 3)
        )

        # Accel: 476 Hz ODR + full-scale selection
        accel_fs_bits = ACCEL_FS_BITS.get(self.accel_fs, 0b00)
        self.bus.write_byte_data(
            self.addr_ag, REG_CTRL_REG6_XL,
            (0b100 << 5) | (accel_fs_bits << 3)
        )

        info = self.get_sensor_info()
        log.info(f"Gyro:  ±{info['gyro_range_dps']} °/s "
                 f"@ {info['actual_rate_hz']:.0f} Hz")
        log.info(f"Accel: ±{info['accel_range_g']} g "
                 f"@ {info['actual_rate_hz']:.0f} Hz")

    def read_accel_gyro(self):
        # Read gyro (6 bytes)
        g_raw = self.bus.read_i2c_block_data(
            self.addr_ag, REG_OUT_X_L_G, 6
        )
        gx_raw = struct.unpack("<h", bytes(g_raw[0:2]))[0]
        gy_raw = struct.unpack("<h", bytes(g_raw[2:4]))[0]
        gz_raw = struct.unpack("<h", bytes(g_raw[4:6]))[0]

        # Read accel (6 bytes)
        a_raw = self.bus.read_i2c_block_data(
            self.addr_ag, REG_OUT_X_L_XL, 6
        )
        ax_raw = struct.unpack("<h", bytes(a_raw[0:2]))[0]
        ay_raw = struct.unpack("<h", bytes(a_raw[2:4]))[0]
        az_raw = struct.unpack("<h", bytes(a_raw[4:6]))[0]

        # Convert to physical units
        g_sens = GYRO_SENSITIVITY_MDPS[self.gyro_fs] / 1000.0  # dps/LSB
        a_sens = ACCEL_SENSITIVITY_MG[self.accel_fs] / 1000.0  # g/LSB

        return (
            ax_raw * a_sens,
            ay_raw * a_sens,
            az_raw * a_sens,
            gx_raw * g_sens,
            gy_raw * g_sens,
            gz_raw * g_sens,
        )

    def enable_magnetometer(self):
        try:
            # Continuous conversion mode, 80 Hz ODR
            self.bus.write_byte_data(
                self.addr_m, REG_CTRL_REG1_M,
                0b01111100  # Temp comp + UHP mode + 80 Hz
            )
            self.bus.write_byte_data(
                self.addr_m, REG_CTRL_REG2_M,
                0x00  # ±4 gauss
            )
            self.bus.write_byte_data(
                self.addr_m, REG_CTRL_REG3_M,
                0x00  # Continuous conversion
            )

            self.mag_enabled = True
            log.info("Magnetometer: continuous mode @ 80 Hz")
            return True
        except OSError:
            log.warning("Failed to enable magnetometer")
            return False

    def read_magnetometer(self):
        if not self.mag_enabled:
            return None
        try:
            status = self.bus.read_byte_data(
                self.addr_m, REG_STATUS_REG_M
            )
            if not (status & 0x08):  # ZYXDA bit
                return None

            raw = self.bus.read_i2c_block_data(
                self.addr_m, REG_OUT_X_L_M, 6
            )
            mx = struct.unpack("<h", bytes(raw[0:2]))[0] * MAG_SENSITIVITY
            my = struct.unpack("<h", bytes(raw[2:4]))[0] * MAG_SENSITIVITY
            mz = struct.unpack("<h", bytes(raw[4:6]))[0] * MAG_SENSITIVITY

            # Convert milligauss to µT (1 mgauss = 0.1 µT)
            return mx * 0.1, my * 0.1, mz * 0.1
        except OSError:
            return None

    def read_temperature(self):
        raw = self.bus.read_i2c_block_data(
            self.addr_ag, REG_OUT_TEMP_L, 2
        )
        temp_raw = struct.unpack("<h", bytes(raw))[0]
        return 25.0 + temp_raw / 16.0

    def get_sensor_info(self):
        return {
            "name": "LSM9DS1",
            "gyro_range_dps": LSM_GYRO_RANGES[self.gyro_fs],
            "accel_range_g": ACCEL_RANGES_G[self.accel_fs],
            "actual_rate_hz": 476.0,  # Fixed at 476 Hz ODR
            "has_magnetometer": True,
        }

    def close(self):
        self.bus.close()
