"""
SPIRE IMU Drivers
Abstract base class and factory for hardware-specific IMU drivers.

Supported sensors:
  - ICM-20948 (9-axis, TDK InvenSense)
  - MPU6886  (6-axis, TDK InvenSense) — stub, awaiting hardware
  - LSM9DS1  (9-axis, STMicroelectronics) — stub, awaiting hardware

Usage:
    from imu_drivers import create_driver
    imu = create_driver("icm20948", bus=1, address=0x69)
    imu.initialize()
    ax, ay, az, gx, gy, gz = imu.read_accel_gyro()
"""

from abc import ABC, abstractmethod


class IMUDriver(ABC):
    """Abstract base class for all IMU drivers.

    Every hardware-specific driver must implement these methods.
    This ensures imu_reader.py works identically regardless of
    which physical sensor is connected.
    """

    @abstractmethod
    def initialize(self, gyro_fs=1, accel_fs=1, rate_div=0):
        """Initialize and configure the sensor.

        Args:
            gyro_fs: Gyro full-scale range index
                     (meaning depends on sensor, typically 0-3)
            accel_fs: Accel full-scale range index (typically 0-3)
            rate_div: Sample rate divider (sensor-specific)
        """
        pass

    @abstractmethod
    def check_id(self):
        """Verify sensor identity via WHO_AM_I register.

        Raises:
            RuntimeError: If sensor not found or wrong ID
        """
        pass

    @abstractmethod
    def read_accel_gyro(self):
        """Read accelerometer and gyroscope data.

        Returns:
            tuple: (ax, ay, az, gx, gy, gz)
                   accel in g, gyro in °/s
        """
        pass

    @abstractmethod
    def read_magnetometer(self):
        """Read magnetometer data.

        Returns:
            tuple: (mx, my, mz) in µT, or None if not available/ready
        """
        pass

    @abstractmethod
    def enable_magnetometer(self):
        """Enable magnetometer readings.

        Returns:
            bool: True if magnetometer available and enabled
        """
        pass

    @abstractmethod
    def read_temperature(self):
        """Read die temperature.

        Returns:
            float: Temperature in °C
        """
        pass

    @abstractmethod
    def get_sensor_info(self):
        """Return sensor metadata for logging.

        Returns:
            dict with keys: name, gyro_range_dps, accel_range_g,
                            actual_rate_hz, has_magnetometer
        """
        pass

    @abstractmethod
    def close(self):
        """Release hardware resources (I2C bus, etc.)."""
        pass


# ---------------------------------------------------------------------------
# Gyro/accel range lookup tables (shared across drivers)
# ---------------------------------------------------------------------------

GYRO_RANGES_DPS = [250, 500, 1000, 2000]
ACCEL_RANGES_G = [2, 4, 8, 16]


# ---------------------------------------------------------------------------
# Driver factory
# ---------------------------------------------------------------------------

DRIVER_REGISTRY = {}


def register_driver(name):
    """Decorator to register a driver class by name."""
    def decorator(cls):
        DRIVER_REGISTRY[name.lower()] = cls
        return cls
    return decorator


def create_driver(name, bus=1, address=None):
    """Create an IMU driver instance by sensor name.

    Args:
        name: Sensor name (e.g. "icm20948", "mpu6886", "lsm9ds1")
        bus: I2C bus number
        address: I2C address (None = use sensor default)

    Returns:
        IMUDriver instance

    Raises:
        ValueError: If sensor name not recognized
    """
    key = name.lower().replace("-", "").replace("_", "")
    if key not in DRIVER_REGISTRY:
        available = ", ".join(sorted(DRIVER_REGISTRY.keys()))
        raise ValueError(
            f"Unknown IMU driver: '{name}'. "
            f"Available: {available}"
        )

    cls = DRIVER_REGISTRY[key]
    if address is None:
        address = cls.DEFAULT_ADDRESS
    return cls(bus_num=bus, address=address)


def list_drivers():
    """Return list of registered driver names."""
    return sorted(DRIVER_REGISTRY.keys())


# Import drivers to trigger registration
from . import icm20948
from . import mpu6886
from . import lsm9ds1
