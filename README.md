# SPIRE
### Stratospheric Payload for Imaging and Rotation Elimination

**Master's Thesis Project** | AGH University of Science and Technology, Kraków  
**Program:** Space Technologies  
**Author:** Piotr Krypel  
**Status:** Active development — flight test planned for 2026

---

## Overview

SPIRE is an integrated high-resolution imaging payload for stratospheric balloon missions. The core research goal is to combine active mechanical yaw stabilization with an IMU-synchronized software pipeline to capture sharp, high-resolution images of Earth's surface from 10 to 30 km altitude using a cost-constrained, single-board-computer platform.

The system is designed around the Sony IMX477 sensor with a 25 mm telephoto lens, which at stratospheric altitude yields a ground sampling distance well below 1 m per pixel. At this focal length, the payload is sensitive to even small residual angular velocities, which makes the stabilization and deblurring pipeline the central engineering challenge of the project.

This work bridges a documented gap in the HAB imaging literature: existing high-resolution balloon platforms either avoid on-board image correction entirely, or active stabilization systems focus on coarse platform control without addressing the full image processing chain. SPIRE targets both problems in a single integrated design.

---

## System Architecture

```
                    ┌─────────────────────────────────────┐
                    │           SPIRE Payload             │
                    │                                     │
  ┌─────────────┐   │  ┌──────────┐    ┌───────────────┐  │
  │  GPS (NMEA) │──►│  │gps_reader│    │  imu_reader   │◄─┼── 5x IMU array
  └─────────────┘   │  └────┬─────┘    │ (1 kHz target │  │   (ICM-20948 +
                    │       │          │  shared mem)  │  │    MPU6886 x4)
                    │       │          └───────┬───────┘  │
                    │  ┌────▼──────────────────▼────────┐ │
                    │  │        data_logger             │ │
                    │  │   (CSV + telemetry stream)     │ │
                    │  └────────────────────────────────┘ │
                    │                                     │
  ┌─────────────┐   │  ┌──────────┐    ┌───────────────┐  │
  │  IMX477 HQ  │──►│  │ capture  │◄──►│  servo_ctrl   │──┼── TD-6622MG
  │  25mm lens  │   │  │ (RAW+JPG)│    │  (PID yaw)    │  │   via PCA9685
  └─────────────┘   │  └──────────┘    └───────────────┘  │
                    │                                     │
                    │  Raspberry Pi 5 (flight computer)   │
                    └─────────────────────────────────────┘
```

Five concurrent processes communicate via shared memory (IMU state) and command queues:

| Process | Function |
|---|---|
| `imu_reader` | Polls IMU array at ~1 kHz; writes to shared memory |
| `capture` | Triggers JPEG + DNG capture with embedded IMU metadata |
| `servo_ctrl` | PID controller driving yaw servo via PCA9685 |
| `data_logger` | Writes timestamped CSV and telemetry |
| `gps_reader` | Parses NMEA, provides UTC reference |

---

## Hardware

### Flight Computer
- **Raspberry Pi 5** running Debian 13 Trixie
- Camera connected via 22-pin FPC cable to CAM/DISP 1 port

### Camera
- ArduCam IMX477 HQ Camera (12.3 MP, 1.55 µm pixel pitch)
- 25 mm C-mount telephoto lens
- Nadir-looking configuration

### IMU Array
- 1x LSM9DS1 (9-DoF: accel + gyro + mag) mounted on camera platform
- 4x MPU6886 (6-DoF) at capsule corners
- TCA9548A I2C multiplexer for bus management
- Achieved sample rate: ~486 Hz per sensor (ICM-20948 validated)

### Stabilization
- TD-6622MG digital servo (20 kg·cm, metal gears)
- PCA9685 16-channel PWM driver (hardware PWM, jitter-free)
- Bearing-supported rotating camera platform
- Single-axis active yaw; passive isolation on remaining axes

### Power
- LAOMAO XL4016-based buck converter (12-36V in, 5A continuous)
- 3S LiPo target for flight with thermal insulation
- Total budget: under 12 W

### Development Board
- Raspberry Pi 4 (Debian 13 Trixie, user: `revan`)
- Python venv at `~/payload/.venv` with `--system-site-packages`

---

## Software

### Stack
- **Python** — capture, IMU reading, servo control, GPS, logging
- **C++** — deblurring pipeline (planned)
- Toolchain: `rpicam-*` suite (`rpicam-still`, `rpicam-hello`)

### Current Status

| Module | Status |
|---|---|
| `capture.py` v2.0 | Working and tested on RPi 5 |
| `imu_reader.py` | Working; ~0.85 ms shared memory latency |
| Two-process sync via shared memory | Validated |
| Adaptive exposure control | Working |
| `servo_ctrl` (PCA9685 PID) | In development |
| `gps_reader` | Planned |
| `data_logger` | Planned |
| C++ deblurring pipeline | Planned |

### Key Design Parameters

| Parameter | Value |
|---|---|
| Exposure range | 1/150 s to 1/1000 s (adaptive) |
| Angular velocity budget | ~0.53 deg/s at 1/150 s for < 1 px blur |
| Image format | JPEG + DNG (RAW16, ~25 MB/frame) |
| Processing latency target | ≤ 200 ms/frame |
| Expected RAW data volume | ~50 GB per 3-hour flight |
| Telemetry downlink | Downsampled JPEG previews via LoRa/APRS |

---

## Repository Structure

```
SPIRE/
├── capture.py          # Image acquisition with IMU metadata embedding
├── imu_reader.py       # IMU polling process, shared memory writer
├── servo_ctrl.py       # PID yaw stabilization via PCA9685 (in progress)
├── data_logger.py      # CSV and telemetry logging (planned)
├── gps_reader.py       # NMEA/UTC parsing (planned)
├── deblur/             # C++ deblurring pipeline (planned)
└── tests/              # Hardware validation scripts
```

Active development is on the `dev-test` branch.

---

## Stabilization Approach

Payload spin is the primary source of image blur at telephoto focal lengths. The literature documents rotation rates up to 1 rev/s during ascent (Flaten et al.) and exceeding ±100 deg/s in the jet stream region (Stark et al., BAMS 2023). Prior systems address either the mechanical problem (CHAOS, HAVOC) or the software problem (Joshi et al., SIGGRAPH 2010; Karpenko et al., Stanford 2011) but not both in an integrated on-board pipeline.

SPIRE's approach:

1. **Mechanical layer** — active yaw servo loop (TD-6622MG + PCA9685 + PID) reduces gross rotation before capture
2. **Exposure control** — adaptive shutter (1/150 to 1/1000 s) based on real-time angular velocity from the IMU array
3. **Software layer** — IMU-synchronized deblurring using motion vectors recorded during each exposure interval, running in C++ on the Raspberry Pi

The 25 mm focal length is a deliberate thesis choice: it halves the tolerable angular velocity budget compared to 50 mm (stricter requirement), increases sensitivity to residual motion, and demands a properly engineered pipeline rather than accepting blur as unavoidable.

---

## Testing Plan

- **Bench tests** — stabilization loop performance under known angular velocity profiles; IMU latency and sync accuracy measurement
- **Environmental tests** — cold chamber (-50°C to -70°C), vibration, and illumination representative of stratospheric conditions
- **Integration test** — full payload end-to-end: capture, IMU logging, servo control, deblurring
- **Flight test** — stratospheric balloon launch to validate the complete pipeline under real conditions

---

## Research Context

This project is submitted as a Master's thesis at AGH UST, Department of Space Technologies. The research gap addressed: no documented Raspberry Pi-compatible imaging system exists that combines active single-axis mechanical stabilization with IMU-synchronized capture, rolling-shutter-aware exposure control, and on-board deblurring, tailored to stratospheric balloon constraints.

Key references informing the design:
- Stark et al., *BAMS* 2023 — active HAB platform stabilization using cold-gas thrusters
- Joshi et al., *SIGGRAPH* 2010 — IMU-aided blind deconvolution for DSLR deblurring
- Karpenko et al., Stanford TR 2011 — real-time gyro-based rolling-shutter correction on mobile GPU
- Wang et al., *Sensors* 2023 — professional near-space high-resolution imaging (0.2 m GSD at 20 km)
- von Ehrenfried & Lim, *AIAA ASCEND* 2022 — CHAOS 3-axis servo stabilization platform

---

## License

Academic research project. Contact the author for usage inquiries.
