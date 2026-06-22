# SPIRE
### Stratospheric Payload for Imaging and Rotation Elimination

**Master's Thesis Project** | AGH University of Science and Technology, Kraków  
**Program:** Space Technologies  
**Author:** Piotr Krypel  
**Status:** Active development - flight test planned for 2026

> **Note:** The capture and stabilization pipeline is consolidated in
> `src/flight_capture.py`. Some architecture notes below describe the
> originally planned multi-process design; the flight build integrates
> capture, IMU reading, rate-integration servo control, and burst frame
> selection into a single coordinated program.

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
  │  GPS (NMEA) │──►│  │gps_reader│    │  imu_reader   │◄─┼── 2x IMU
  └─────────────┘   │  └────┬─────┘    │ (shared mem)  │  │   ICM-20948 (camera)
                    │       │          └───────┬───────┘  │   LSM9DS1 (platform)
                    │  ┌────▼──────────────────▼────────┐ │
                    │  │        data_logger             │ │
                    │  │   (CSV + telemetry stream)     │ │
                    │  └────────────────────────────────┘ │
                    │                                     │
  ┌─────────────┐   │  ┌──────────┐    ┌───────────────┐  │
  │  IMX477 HQ  │──►│  │ capture  │◄──►│  servo_ctrl   │──┼── TD-6622MG
  │  25mm lens  │   │  │ (RAW+JPG)│    │ (rate-integ.) │  │   hardware PWM
  └─────────────┘   │  └──────────┘    └───────────────┘  │   GPIO 12 (pin 32)
                    │                                     │
                    │  Raspberry Pi 5 (flight computer)   │
                    └─────────────────────────────────────┘
```

The flight build consolidates these functions into `src/flight_capture.py`.
Concurrent threads and processes communicate via shared memory (IMU state):

| Function | Role |
|---|---|
| `imu_reader` | Polls camera + platform IMUs; writes to shared memory |
| capture | Burst full-res capture with embedded IMU metadata |
| servo control | Rate-integration yaw stabilization (hardware PWM) |
| `data_logger` | Writes timestamped metadata per frame |
| `gps_reader` | Parses NMEA, provides UTC reference (planned) |

---

## Hardware

### Flight Computer
- **Raspberry Pi 5** running Debian 13 Trixie
- Camera connected via 22-pin FPC cable to CAM/DISP 1 port

### Camera
- ArduCam IMX477 HQ Camera (12.3 MP, 1.55 µm pixel pitch)
- 25 mm C-mount telephoto lens
- Nadir-looking configuration

### IMU Configuration
- **ICM-20948** (9-DoF) rigidly mounted to the RPi/camera module — measures
  camera yaw for exposure-time frame selection
- **LSM9DS1** (9-DoF) mounted on the gondola/platform frame — drives the
  rate-integration stabilization loop
- Validated sample rate: ~486 Hz per sensor

### Stabilization
- TD-6622MG digital servo (20 kg·cm, metal gears)
- **Hardware PWM** directly on GPIO 12 / physical pin 32 (`rpi_hardware_pwm`,
  PWM0 channel 0) — jitter-free, no external PWM driver needed
- Bearing-supported rotating camera platform
- Single-axis active yaw; passive 2-axis gimbal (pitch/roll) on remaining axes

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
| `flight_capture.py` v2.1 | Working and tested on RPi 5 |
| `imu_reader.py` | Working; ~0.85 ms shared memory latency |
| Two-IMU sync via shared memory | Validated |
| Hardware PWM servo (GPIO 12) | Working; jitter eliminated |
| Rate-integration stabilization | Working; validated on bench |
| Burst-and-select capture | Working; selects sharpest frame |
| Exposure-time IMU correlation | Working (SensorTimestamp) |
| `gps_reader` | Planned |
| C++ deblurring pipeline | Python prototype done; C++ planned |

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
├── src/
│   ├── flight_capture.py   # Main capture + rate-integration stabilization
│   ├── imu_reader.py       # IMU polling, shared memory writer
│   ├── imu_drivers/        # ICM-20948, LSM9DS1 drivers
│   ├── imu_calibrate.py    # IMU calibration
│   ├── servo_test.py       # Standalone servo bench test
│   ├── deblur_prototype.py # Wiener deconvolution prototype (Python)
│   ├── gps_reader.py       # NMEA/UTC parsing (planned)
│   └── preview.py          # HTTP preview server
└── docs/                   # Checkpoints and notes
```

Active development is on the `dev-test` branch; tested checkpoints land on
`main`. Generated flight data lives on a separate `data` branch.

---

## Usage

Default run uses the validated configuration (rate-integration, burst-early,
6-frame burst, rate-gain 1.5):

```bash
python3 src/flight_capture.py --interval 8 -n 10 --preview
```

Open `http://<pi-ip>:8080` for the live preview and a gallery of saved frames.
Add `--diag` to log per-frame exposure-time camera angular velocity and the
burst selection record.

### Hardware PWM setup (one-time)

The servo uses jitter-free hardware PWM via `rpi-hardware-pwm`:

1. Add to `/boot/firmware/config.txt`:
   `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4`
2. Reboot.
3. `sudo pip3 install rpi-hardware-pwm --break-system-packages`
4. Verify: `pinctrl get 12` reports `PWM0_CHAN0`.

GPIO 12 = PWM0 channel 0. On kernel 6.12+ (Debian Trixie) use `chip=0`.

### Key options

| Option | Default | Description |
|---|---|---|
| `--interval` | 10 | Seconds between capture cycles |
| `--burst-count` | 6 | Frames per burst; keep lowest \|cam_gz\| |
| `--no-burst-early` | (off) | Wait for STABLE gate instead of arc burst |
| `--rate-gain` | 1.5 | Servo counter-rotation gain (servo/voltage-calibrated) |
| `--gyro-deadband` | 0.5 | Zero servo rate below this \|gyro\| deg/s |
| `--d-alpha` | 0.3 | Rate low-pass factor; lower = more smoothing |
| `--min-rotation` | 5 | Below this rotation, skip stabilization (CALM) |
| `--preview` | (off) | Enable HTTP preview server |
| `--diag` | (off) | Exposure-time gyro correlation diagnostics |

Run `python3 src/flight_capture.py -h` for the full list.

> **Note:** `--rate-gain` is calibrated for the current servo at ~6.2 V.
> Supply voltage changes (battery state, stratospheric cold) shift the
> effective gain; re-check before flight.

---

## Stabilization Approach

Payload spin is the primary source of image blur at telephoto focal lengths. The literature documents rotation rates up to 1 rev/s during ascent (Flaten et al.) and exceeding ±100 deg/s in the jet stream region (Stark et al., BAMS 2023). Prior systems address either the mechanical problem (CHAOS, HAVOC) or the software problem (Joshi et al., SIGGRAPH 2010; Karpenko et al., Stanford 2011) but not both in an integrated on-board pipeline.

SPIRE's approach:

1. **Mechanical layer** — active yaw servo (TD-6622MG on hardware PWM)
   counter-rotates the camera against gondola spin. The control law is
   **rate-integration**: the servo angle is the running integral of the
   negated platform angular velocity, so the camera stays inertially fixed in
   yaw. This replaced an earlier position-based PID, which exhibited a growing
   back-and-forth oscillation at low rotation rates because the position loop
   overshot through its heading reference. Rate-integration tracks velocity
   with no position reference, so a near-still platform leaves the servo still.
2. **Burst-and-select capture** — at each cycle a short burst of full-resolution
   frames is captured across the active servo arc. Each frame's hardware
   `SensorTimestamp` is correlated against a high-rate camera-gyro ring buffer,
   and the frame with the lowest angular velocity at its actual exposure
   instant is kept; the rest are discarded. This decouples the moment of
   sharpness from capture latency: the camera is momentarily still mid-arc
   (when servo velocity matches platform rotation), not after the servo stops.
3. **Software layer** — IMU-synchronized deblurring using motion vectors
   recorded during each exposure interval, running in C++ on the Raspberry Pi
   (Python prototype validated; C++ implementation pending).

The 25 mm focal length is a deliberate thesis choice: it halves the tolerable
angular velocity budget compared to 50 mm (stricter requirement), increases
sensitivity to residual motion, and demands a properly engineered pipeline
rather than accepting blur as unavoidable.

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
