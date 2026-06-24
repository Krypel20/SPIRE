#!/usr/bin/env python3
"""
MPRLS (0-172 kPa) pressure sensor test and discovery
For Adafruit module on RPi I2C bus 1
"""

import smbus2
import time
import sys
import statistics

def discover_mprls(bus_num=1):
    """Scan I2C bus for MPRLS sensor (usually 0x76, but may vary)"""
    bus = smbus2.SMBus(bus_num)
    
    print("Scanning I2C bus 1 for MPRLS sensor...")
    candidates = [0x76, 0x18, 0x1e, 0x77]  # Common MPRLS addresses
    
    found = None
    for addr in candidates:
        try:
            # Try to read status byte at offset 0
            data = bus.read_i2c_block_data(addr, 0, 1)
            print(f"  ✓ Response at 0x{addr:02x}: status=0x{data[0]:02x}")
            found = addr
            break
        except Exception:
            pass
    
    bus.close()
    return found

def read_mprls_raw(addr=0x76, bus_num=1):
    """
    Read raw pressure data from MPRLS.
    
    Protocol:
    - Send command 0xAA to trigger measurement
    - Wait 5ms for conversion
    - Read 3 bytes: [status, pressure_MSB, pressure_LSB]
    - Pressure (Pa) = (pressure_data >> 5) * (172000 / 16384)
    """
    bus = smbus2.SMBus(bus_num)
    
    try:
        # Trigger measurement
        bus.write_byte(addr, 0xAA)
        time.sleep(0.005)  # 5ms conversion time
        
        # Read status + 2 pressure bytes
        data = bus.read_i2c_block_data(addr, 0x00, 3)
        status = data[0]
        pressure_raw = (data[1] << 8) | data[2]
        
        # Convert raw to pressure in Pa
        # Formula from Adafruit: pressure = (raw_value >> 5) * (172000 / 16384)
        pressure_pa = (pressure_raw >> 5) * (172000 / 16384)
        pressure_hpa = pressure_pa / 100
        
        return {
            'status': status,
            'pressure_raw': pressure_raw,
            'pressure_pa': pressure_pa,
            'pressure_hpa': pressure_hpa,
            'ready': (status & 0x40) == 0,  # Bit 6: 0=ready, 1=busy
        }
    finally:
        bus.close()

def test_accuracy(addr=0x76, samples=20, interval=0.1):
    """Run series of measurements to assess accuracy and stability"""
    print(f"\nRunning {samples} measurements (interval {interval}s)...")
    print("=" * 60)
    
    measurements = []
    for i in range(samples):
        try:
            result = read_mprls_raw(addr=addr)
            measurements.append(result['pressure_hpa'])
            
            status_str = "ready" if result['ready'] else "busy"
            print(f"  {i+1:2d}. {result['pressure_hpa']:7.2f} hPa  "
                  f"[status={status_str}, raw=0x{result['pressure_raw']:04x}]")
            
            time.sleep(interval)
        except Exception as e:
            print(f"  {i+1:2d}. ERROR: {e}")
            return None
    
    # Statistics
    mean = statistics.mean(measurements)
    stdev = statistics.stdev(measurements) if len(measurements) > 1 else 0
    min_p = min(measurements)
    max_p = max(measurements)
    
    print("=" * 60)
    print(f"Mean:   {mean:.2f} hPa")
    print(f"StDev:  {stdev:.4f} hPa")
    print(f"Range:  {min_p:.2f} – {max_p:.2f} hPa (Δ={max_p-min_p:.2f})")
    print(f"Relative stability: {100*stdev/mean:.3f}% (should be <1% for stable sensor)")
    
    return measurements

def estimate_altitude_from_pressure(pressure_hpa, sea_level_pressure=1013.25):
    """
    Rough altitude estimate from barometric formula.
    pressure_hpa: measured pressure in hPa
    sea_level_pressure: reference (default 1013.25 hPa)
    Returns: estimated altitude in meters
    """
    # Barometric formula: h ≈ 44330 * (1 - (P/P0)^(1/5.255))
    altitude = 44330 * (1 - (pressure_hpa / sea_level_pressure) ** (1/5.255))
    return altitude

if __name__ == '__main__':
    print("MPRLS Pressure Sensor Test")
    print("=" * 60)
    
    # Step 1: Discover sensor
    addr = discover_mprls()
    if not addr:
        print("ERROR: MPRLS not found on any common address.")
        print("Check wiring (VIN, GND, SCL, SDA) and I2C bus number.")
        sys.exit(1)
    
    print(f"✓ MPRLS found at 0x{addr:02x}\n")
    
    # Step 2: Single test read
    print("Single measurement:")
    result = read_mprls_raw(addr=addr)
    print(f"  Pressure: {result['pressure_hpa']:.2f} hPa")
    print(f"  Status: ready={result['ready']}, raw=0x{result['pressure_raw']:04x}\n")
    
    # Step 3: Accuracy test
    measurements = test_accuracy(addr=addr, samples=20, interval=0.2)
    
    if measurements:
        # Step 4: Altitude estimation
        mean_pressure = sum(measurements) / len(measurements)
        print(f"\nAltitude estimate (from pressure, sea level ref):")
        print(f"  {estimate_altitude_from_pressure(mean_pressure):.1f} m")
        print(f"  (This assumes 1013.25 hPa at sea level on test day)")
        print(f"  Actual altitude depends on local sea-level pressure!")