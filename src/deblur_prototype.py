#!/usr/bin/env python3
"""
SPIRE Deblurring Prototype — Synthetic Data Validation
Validates IMU-based motion deblurring pipeline using controlled synthetic blur.

Pipeline:
  1. Load a sharp reference image
  2. Generate synthetic motion PSF (simulating known camera motion)
  3. Apply blur via convolution (simulating real motion blur)
  4. Reconstruct sharp image via Wiener deconvolution
  5. Evaluate quality (PSNR, SSIM, edge sharpness)

This validates the algorithm before testing with real IMU+camera data.

Usage:
  python3 deblur_prototype.py -i data/test_capture/sharp.jpg
  python3 deblur_prototype.py -i sharp.jpg --angle 15 --length 20
  python3 deblur_prototype.py -i sharp.jpg --trajectory arc --omega 2.0
  python3 deblur_prototype.py -i sharp.jpg --imu-csv data/imu_log.csv --frame-start 1000 --exposure-us 6667
"""

import os
import sys
import argparse
import logging
import json
import csv
import numpy as np
from datetime import datetime, timezone

log = logging.getLogger("spire.deblur")

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

def setup_logging(verbose=False):
    level = logging.DEBUG if verbose else logging.INFO
    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S"
    )
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level)
    console.setFormatter(fmt)
    log.setLevel(logging.DEBUG)
    log.addHandler(console)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PIXEL_PITCH_UM = 1.55
DEFAULT_FOCAL_LENGTH_MM = 6 #ultimately will be set by the lens focal length (25 mm or 16 mm)

# ---------------------------------------------------------------------------
# PSF Generation
# ---------------------------------------------------------------------------

def angular_velocity_to_pixels(omega_dps, exposure_s,
                                focal_length_mm=DEFAULT_FOCAL_LENGTH_MM):
    """Convert angular velocity to pixel displacement during exposure.

    Args:
        omega_dps: Angular velocity in degrees/second
        exposure_s: Exposure time in seconds 
        focal_length_mm: Lens focal length in mm

    Returns:
        Pixel displacement (float)
    """
    omega_rad = np.radians(omega_dps)
    displacement_mm = omega_rad * exposure_s * focal_length_mm
    displacement_px = displacement_mm / (PIXEL_PITCH_UM * 1e-3)
    return displacement_px


def make_linear_psf(length_px, angle_deg, size=None):
    """Generate a linear motion blur PSF.

    Simulates uniform linear camera motion during exposure.

    Args:
        length_px: Blur length in pixels
        angle_deg: Blur direction in degrees (0=horizontal right)
        size: Output kernel size (default: auto)

    Returns:
        2D numpy array (normalized PSF kernel)
    """
    length_px = max(1, int(round(length_px)))
    if size is None:
        size = length_px + 2
    if size % 2 == 0:
        size += 1

    psf = np.zeros((size, size), dtype=np.float64)
    center = size // 2

    angle_rad = np.radians(angle_deg)
    dx = np.cos(angle_rad)
    dy = np.sin(angle_rad)

    for i in range(length_px):
        t = (i - length_px / 2.0 + 0.5)
        x = int(round(center + t * dx))
        y = int(round(center + t * dy))
        if 0 <= x < size and 0 <= y < size:
            psf[y, x] += 1.0

    total = psf.sum()
    if total > 0:
        psf /= total

    return psf


def make_arc_psf(omega_dps, exposure_s, focal_length_mm=DEFAULT_FOCAL_LENGTH_MM,
                 size=None, axis='z'):
    """Generate an arc/rotational motion blur PSF.

    Simulates camera rotation at constant angular velocity during exposure.

    Args:
        omega_dps: Angular velocity in °/s
        exposure_s: Exposure time in seconds
        focal_length_mm: Focal length in mm
        size: Output kernel size
        axis: Rotation axis ('z'=yaw, 'x'=pitch, 'y'=roll)

    Returns:
        2D numpy array (normalized PSF kernel)
    """
    total_px = angular_velocity_to_pixels(omega_dps, exposure_s,
                                           focal_length_mm)
    n_steps = max(int(abs(total_px) * 2), 10)

    if size is None:
        size = int(abs(total_px)) + 4
    if size % 2 == 0:
        size += 1

    psf = np.zeros((size, size), dtype=np.float64)
    center = size // 2

    for i in range(n_steps):
        t = (i / n_steps - 0.5) * exposure_s
        angle = np.radians(omega_dps * t)

        if axis == 'z':  # Yaw — horizontal displacement
            dx = angle * focal_length_mm / (PIXEL_PITCH_UM * 1e-3)
            dy = 0
        elif axis == 'x':  # Pitch — vertical displacement
            dx = 0
            dy = angle * focal_length_mm / (PIXEL_PITCH_UM * 1e-3)
        elif axis == 'y':  # Roll — rotational
            r = size / 4
            dx = r * np.sin(angle)
            dy = r * (1 - np.cos(angle))

        x = int(round(center + dx))
        y = int(round(center + dy))
        if 0 <= x < size and 0 <= y < size:
            psf[y, x] += 1.0

    total = psf.sum()
    if total > 0:
        psf /= total

    return psf


def make_psf_from_imu_data(gyro_samples, timestamps_ns, exposure_start_ns,
                            exposure_us, focal_length_mm=DEFAULT_FOCAL_LENGTH_MM,
                            size=None):
    """Generate PSF from real IMU gyroscope data.

    Integrates angular velocity samples over the exposure window
    to compute the camera motion trajectory, then maps to pixel coordinates.

    Args:
        gyro_samples: List of (gx, gy, gz) in °/s
        timestamps_ns: List of timestamps in nanoseconds
        exposure_start_ns: Start of exposure (monotonic ns)
        exposure_us: Exposure duration in microseconds
        focal_length_mm: Focal length in mm
        size: Output kernel size

    Returns:
        2D numpy array (normalized PSF kernel)
    """
    exposure_ns = exposure_us * 1000
    exposure_end_ns = exposure_start_ns + exposure_ns

    # Filter samples within exposure window
    window_samples = []
    for i, t in enumerate(timestamps_ns):
        if exposure_start_ns <= t <= exposure_end_ns:
            window_samples.append((t, gyro_samples[i]))

    if len(window_samples) < 2:
        log.warning(f"Only {len(window_samples)} IMU samples in exposure "
                    f"window — PSF may be inaccurate")
        if len(window_samples) == 0:
            # Return delta PSF (no blur)
            psf = np.zeros((3, 3), dtype=np.float64)
            psf[1, 1] = 1.0
            return psf

    # Integrate angular velocity → angular displacement
    trajectory_px = []
    cumulative_x = 0.0  # Yaw displacement (pixels)
    cumulative_y = 0.0  # Pitch displacement (pixels)

    for i in range(len(window_samples)):
        t, (gx, gy, gz) = window_samples[i]

        if i > 0:
            dt = (t - window_samples[i - 1][0]) / 1e9  # seconds
            # Yaw (gz) → horizontal pixel shift
            cumulative_x += np.radians(gz) * dt * focal_length_mm / (
                PIXEL_PITCH_UM * 1e-3
            )
            # Pitch (gx) → vertical pixel shift
            cumulative_y += np.radians(gx) * dt * focal_length_mm / (
                PIXEL_PITCH_UM * 1e-3
            )

        trajectory_px.append((cumulative_x, cumulative_y))

    # Center trajectory
    trajectory_px = np.array(trajectory_px)
    trajectory_px[:, 0] -= trajectory_px[:, 0].mean()
    trajectory_px[:, 1] -= trajectory_px[:, 1].mean()

    # Determine kernel size
    max_displacement = max(
        np.abs(trajectory_px[:, 0]).max(),
        np.abs(trajectory_px[:, 1]).max()
    )
    if size is None:
        size = int(max_displacement * 2) + 5
    if size % 2 == 0:
        size += 1
    size = max(size, 3)

    # Build PSF from trajectory
    psf = np.zeros((size, size), dtype=np.float64)
    center = size // 2

    for dx, dy in trajectory_px:
        x = int(round(center + dx))
        y = int(round(center + dy))
        if 0 <= x < size and 0 <= y < size:
            psf[y, x] += 1.0

    total = psf.sum()
    if total > 0:
        psf /= total

    log.info(f"PSF from IMU: {len(window_samples)} samples, "
             f"max displacement: {max_displacement:.1f} px, "
             f"kernel: {size}x{size}")

    return psf


# ---------------------------------------------------------------------------
# Blur / Deblur Operations
# ---------------------------------------------------------------------------

def apply_blur(image, psf, noise_sigma=0.0):
    """Apply motion blur to image using convolution.

    Args:
        image: Input image (HxW or HxWxC, float64, range 0-1)
        psf: Blur kernel (normalized)
        noise_sigma: Additive Gaussian noise std (0 = no noise)

    Returns:
        Blurred image (same shape as input)
    """
    from scipy.signal import fftconvolve

    if image.ndim == 3:
        # Process each channel independently
        result = np.zeros_like(image)
        for c in range(image.shape[2]):
            result[:, :, c] = fftconvolve(
                image[:, :, c], psf, mode='same'
            )
    else:
        result = fftconvolve(image, psf, mode='same')

    if noise_sigma > 0:
        noise = np.random.normal(0, noise_sigma, result.shape)
        result = result + noise

    return np.clip(result, 0, 1)


def wiener_deconvolution(blurred, psf, snr=30.0):
    """Restore image using Wiener deconvolution in frequency domain.

    The Wiener filter minimizes MSE between restored and original:
      F_hat = (H* / (|H|^2 + 1/SNR)) * G

    where H is the PSF in frequency domain, G is blurred image,
    and SNR controls noise regularization.

    Args:
        blurred: Blurred image (HxW or HxWxC, float64, range 0-1)
        psf: Blur kernel (must match the blur that was applied)
        snr: Signal-to-noise ratio (higher = less regularization)

    Returns:
        Restored image (same shape)
    """
    def _wiener_2d(blurred_ch, psf_padded_fft, snr):
        G = np.fft.fft2(blurred_ch)
        H = psf_padded_fft
        H_conj = np.conj(H)
        H_sq = np.abs(H) ** 2

        # Wiener filter
        W = H_conj / (H_sq + 1.0 / snr)
        restored = np.real(np.fft.ifft2(G * W))
        return restored

    # Pad PSF to image size
    psf_padded = np.zeros(blurred.shape[:2], dtype=np.float64)
    kh, kw = psf.shape
    psf_padded[:kh, :kw] = psf
    # Shift PSF so center is at (0,0) for correct FFT
    psf_padded = np.roll(psf_padded, -(kh // 2), axis=0)
    psf_padded = np.roll(psf_padded, -(kw // 2), axis=1)
    PSF_FFT = np.fft.fft2(psf_padded)

    if blurred.ndim == 3:
        result = np.zeros_like(blurred)
        for c in range(blurred.shape[2]):
            result[:, :, c] = _wiener_2d(blurred[:, :, c], PSF_FFT, snr)
    else:
        result = _wiener_2d(blurred, PSF_FFT, snr)

    return np.clip(result, 0, 1)


def richardson_lucy(blurred, psf, iterations=20):
    """Restore image using Richardson-Lucy iterative deconvolution.

    Non-linear, iterative algorithm that preserves positivity.
    Better for Poisson noise (common in imaging), but slower.

    Args:
        blurred: Blurred image (HxW or HxWxC, float64, range 0-1)
        psf: Blur kernel
        iterations: Number of RL iterations

    Returns:
        Restored image (same shape)
    """
    from scipy.signal import fftconvolve

    psf_mirror = psf[::-1, ::-1]

    def _rl_2d(blurred_ch, psf, psf_mirror, iterations):
        estimate = np.copy(blurred_ch)
        for i in range(iterations):
            reblurred = fftconvolve(estimate, psf, mode='same')
            reblurred = np.maximum(reblurred, 1e-10)  # Avoid division by zero
            ratio = blurred_ch / reblurred
            correction = fftconvolve(ratio, psf_mirror, mode='same')
            estimate *= correction
        return estimate

    if blurred.ndim == 3:
        result = np.zeros_like(blurred)
        for c in range(blurred.shape[2]):
            result[:, :, c] = _rl_2d(
                blurred[:, :, c], psf, psf_mirror, iterations
            )
    else:
        result = _rl_2d(blurred, psf, psf_mirror, iterations)

    return np.clip(result, 0, 1)


# ---------------------------------------------------------------------------
# Quality Metrics
# ---------------------------------------------------------------------------

def compute_psnr(original, restored):
    """Compute Peak Signal-to-Noise Ratio in dB."""
    mse = np.mean((original - restored) ** 2)
    if mse == 0:
        return float('inf')
    return 10 * np.log10(1.0 / mse)


def compute_ssim(original, restored):
    """Compute Structural Similarity Index (simplified).

    Operates on grayscale. For color images, converts to grayscale first.
    """
    def _to_gray(img):
        if img.ndim == 3:
            return 0.299 * img[:,:,0] + 0.587 * img[:,:,1] + 0.114 * img[:,:,2]
        return img

    img1 = _to_gray(original)
    img2 = _to_gray(restored)

    C1 = (0.01) ** 2
    C2 = (0.03) ** 2

    mu1 = img1.mean()
    mu2 = img2.mean()
    sigma1_sq = img1.var()
    sigma2_sq = img2.var()
    sigma12 = ((img1 - mu1) * (img2 - mu2)).mean()

    ssim = ((2 * mu1 * mu2 + C1) * (2 * sigma12 + C2)) / (
        (mu1**2 + mu2**2 + C1) * (sigma1_sq + sigma2_sq + C2)
    )
    return ssim


def compute_edge_sharpness(image):
    """Compute average edge gradient magnitude (Sobel-based).

    Higher value = sharper edges.
    """
    def _to_gray(img):
        if img.ndim == 3:
            return 0.299 * img[:,:,0] + 0.587 * img[:,:,1] + 0.114 * img[:,:,2]
        return img

    gray = _to_gray(image)

    # Simple Sobel-like gradient
    gx = np.diff(gray, axis=1)
    gy = np.diff(gray, axis=0)

    # Trim to same size
    min_h = min(gx.shape[0], gy.shape[0])
    min_w = min(gx.shape[1], gy.shape[1])
    gx = gx[:min_h, :min_w]
    gy = gy[:min_h, :min_w]

    magnitude = np.sqrt(gx**2 + gy**2)
    return magnitude.mean()


# ---------------------------------------------------------------------------
# Image I/O
# ---------------------------------------------------------------------------

def load_image(path):
    """Load image as float64 array, range 0-1."""
    try:
        from PIL import Image
        img = Image.open(path).convert("RGB")
        return np.array(img, dtype=np.float64) / 255.0
    except ImportError:
        log.error("Pillow required: pip install Pillow")
        sys.exit(1)


def save_image(image, path):
    """Save float64 image (range 0-1) as JPEG."""
    from PIL import Image
    img_uint8 = (np.clip(image, 0, 1) * 255).astype(np.uint8)
    Image.fromarray(img_uint8).save(path, quality=95)
    log.info(f"Saved: {path}")


# ---------------------------------------------------------------------------
# IMU CSV Loader
# ---------------------------------------------------------------------------

def load_imu_csv(csv_path, start_sample=0, num_samples=None):
    """Load gyro data from IMU CSV log.

    Returns:
        tuple: (gyro_samples, timestamps_ns)
            gyro_samples: list of (gx, gy, gz) in °/s
            timestamps_ns: list of monotonic timestamps
    """
    gyro_samples = []
    timestamps = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            if i < start_sample:
                continue
            if num_samples and len(gyro_samples) >= num_samples:
                break
            try:
                gyro_samples.append((
                    float(row["gyro_x"]),
                    float(row["gyro_y"]),
                    float(row["gyro_z"]),
                ))
                timestamps.append(int(row["timestamp_mono_ns"]))
            except (KeyError, ValueError) as e:
                continue

    log.info(f"Loaded {len(gyro_samples)} IMU samples from {csv_path}")
    return gyro_samples, timestamps


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Deblurring Prototype — Synthetic Validation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Synthetic linear blur, 15px at 45 degrees
  %(prog)s -i sharp.jpg --length 15 --angle 45

  # Synthetic rotational blur, 2°/s yaw during 1/150s exposure
  %(prog)s -i sharp.jpg --trajectory arc --omega 2.0

  # Real IMU data
  %(prog)s -i sharp.jpg --imu-csv data/imu_log.csv --frame-start 1000

  # Compare Wiener vs Richardson-Lucy
  %(prog)s -i sharp.jpg --length 20 --method both
        """
    )

    # Input
    parser.add_argument("-i", "--input", required=True,
                        help="Path to sharp reference image")
    parser.add_argument("-o", "--output", default="./data/deblur_results",
                        help="Output directory")

    # Synthetic PSF params
    parser.add_argument("--length", type=float, default=15,
                        help="Linear blur length in pixels (default: 15)")
    parser.add_argument("--angle", type=float, default=0,
                        help="Linear blur angle in degrees (default: 0)")
    parser.add_argument("--trajectory", type=str, default="linear",
                        choices=["linear", "arc"],
                        help="PSF trajectory type (default: linear)")
    parser.add_argument("--omega", type=float, default=2.0,
                        help="Angular velocity for arc PSF in °/s (default: 2.0)")
    parser.add_argument("--exposure", type=int, default=6667,
                        help="Exposure time in µs (default: 6667 = 1/150s)")
    parser.add_argument("--focal-length", type=float,
                        default=DEFAULT_FOCAL_LENGTH_MM,
                        help=f"Focal length in mm (default: {DEFAULT_FOCAL_LENGTH_MM})")

    # Real IMU data
    parser.add_argument("--imu-csv", type=str, default=None,
                        help="Path to IMU CSV log (overrides synthetic PSF)")
    parser.add_argument("--frame-start", type=int, default=0,
                        help="First IMU sample index for exposure window")
    parser.add_argument("--frame-samples", type=int, default=None,
                        help="Number of IMU samples in exposure (auto from rate)")

    # Deconvolution params
    parser.add_argument("--method", type=str, default="both",
                        choices=["wiener", "rl", "both"],
                        help="Deconvolution method (default: both)")
    parser.add_argument("--snr", type=float, default=30.0,
                        help="Wiener SNR parameter (default: 30)")
    parser.add_argument("--rl-iter", type=int, default=20,
                        help="Richardson-Lucy iterations (default: 20)")
    parser.add_argument("--noise", type=float, default=0.001,
                        help="Additive noise sigma (default: 0.001)")

    # Misc
    parser.add_argument("--downscale", type=int, default=4,
                        help="Downscale factor for faster processing (default: 4)")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()
    setup_logging(args.verbose)

    log.info("=" * 50)
    log.info("SPIRE Deblurring Prototype")
    log.info("=" * 50)

    # Load image
    log.info(f"Loading: {args.input}")
    original = load_image(args.input)
    log.info(f"Image size: {original.shape[1]}x{original.shape[0]}")

    # Downscale for faster iteration
    if args.downscale > 1:
        from PIL import Image
        h, w = original.shape[:2]
        new_h, new_w = h // args.downscale, w // args.downscale
        img_pil = Image.fromarray((original * 255).astype(np.uint8))
        img_pil = img_pil.resize((new_w, new_h), Image.LANCZOS)
        original = np.array(img_pil, dtype=np.float64) / 255.0
        log.info(f"Downscaled to: {new_w}x{new_h} (factor {args.downscale})")

    # Generate PSF
    exposure_s = args.exposure / 1e6

    if args.imu_csv:
        # PSF from real IMU data
        log.info(f"Loading IMU data: {args.imu_csv}")
        gyro_samples, timestamps = load_imu_csv(args.imu_csv)

        if args.frame_samples is None:
            # Estimate samples from IMU rate and exposure
            if len(timestamps) > 1:
                dt = (timestamps[-1] - timestamps[0]) / (len(timestamps) - 1)
                rate = 1e9 / dt
                args.frame_samples = max(2, int(rate * exposure_s))
                log.info(f"Estimated {args.frame_samples} IMU samples "
                         f"per exposure @ {rate:.0f} Hz")

        start_ns = timestamps[args.frame_start]
        psf = make_psf_from_imu_data(
            gyro_samples[args.frame_start:args.frame_start + args.frame_samples * 2],
            timestamps[args.frame_start:args.frame_start + args.frame_samples * 2],
            start_ns, args.exposure, args.focal_length
        )
    elif args.trajectory == "arc":
        # Rotational PSF
        pixel_disp = angular_velocity_to_pixels(
            args.omega, exposure_s, args.focal_length
        )
        log.info(f"Arc PSF: omega={args.omega}°/s, exposure={exposure_s*1000:.1f}ms, "
                 f"displacement={pixel_disp:.1f}px")
        psf = make_arc_psf(args.omega, exposure_s, args.focal_length)
    else:
        # Linear PSF
        log.info(f"Linear PSF: length={args.length}px, angle={args.angle}°")
        psf = make_linear_psf(args.length, args.angle)

    log.info(f"PSF kernel: {psf.shape[0]}x{psf.shape[1]}")

    # Apply synthetic blur
    log.info(f"Applying blur (noise sigma={args.noise})...")
    blurred = apply_blur(original, psf, noise_sigma=args.noise)

    # Deconvolution
    results = {}

    if args.method in ("wiener", "both"):
        log.info(f"Wiener deconvolution (SNR={args.snr})...")
        restored_wiener = wiener_deconvolution(blurred, psf, snr=args.snr)
        results["wiener"] = restored_wiener

    if args.method in ("rl", "both"):
        log.info(f"Richardson-Lucy deconvolution ({args.rl_iter} iterations)...")
        restored_rl = richardson_lucy(blurred, psf, iterations=args.rl_iter)
        results["richardson_lucy"] = restored_rl

    # Quality metrics
    log.info("")
    log.info("=" * 50)
    log.info("QUALITY METRICS")
    log.info("=" * 50)

    edge_original = compute_edge_sharpness(original)
    edge_blurred = compute_edge_sharpness(blurred)

    log.info(f"{'':20s} {'PSNR (dB)':>10s} {'SSIM':>10s} {'Edge':>10s}")
    log.info(f"{'Original':20s} {'—':>10s} {'1.000':>10s} {edge_original:>10.4f}")
    log.info(f"{'Blurred':20s} "
             f"{compute_psnr(original, blurred):>10.2f} "
             f"{compute_ssim(original, blurred):>10.4f} "
             f"{edge_blurred:>10.4f}")

    for name, restored in results.items():
        psnr = compute_psnr(original, restored)
        ssim = compute_ssim(original, restored)
        edge = compute_edge_sharpness(restored)
        log.info(f"{name:20s} {psnr:>10.2f} {ssim:>10.4f} {edge:>10.4f}")

    log.info("=" * 50)

    # Save outputs
    os.makedirs(args.output, exist_ok=True)
    save_image(original, os.path.join(args.output, "01_original.jpg"))
    save_image(blurred, os.path.join(args.output, "02_blurred.jpg"))

    for name, restored in results.items():
        save_image(restored, os.path.join(
            args.output, f"03_restored_{name}.jpg"
        ))

    # Save PSF visualization (upscaled)
    psf_vis = psf / psf.max()
    from PIL import Image
    psf_img = Image.fromarray((psf_vis * 255).astype(np.uint8), mode='L')
    psf_img = psf_img.resize(
        (psf_img.width * 10, psf_img.height * 10), Image.NEAREST
    )
    psf_img.save(os.path.join(args.output, "00_psf.png"))
    log.info(f"PSF saved: {os.path.join(args.output, '00_psf.png')}")

    # Save metrics to JSON
    metrics = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "input_image": args.input,
        "psf_type": args.trajectory if not args.imu_csv else "imu",
        "exposure_us": args.exposure,
        "focal_length_mm": args.focal_length,
        "noise_sigma": args.noise,
        "psf_size": psf.shape[0],
        "blurred_psnr": float(compute_psnr(original, blurred)),
        "blurred_ssim": float(compute_ssim(original, blurred)),
    }
    for name, restored in results.items():
        metrics[f"{name}_psnr"] = float(compute_psnr(original, restored))
        metrics[f"{name}_ssim"] = float(compute_ssim(original, restored))
        metrics[f"{name}_edge"] = float(compute_edge_sharpness(restored))

    metrics_path = os.path.join(args.output, "metrics.json")
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)
    log.info(f"Metrics saved: {metrics_path}")

    log.info("\nDone. Review results in: " + args.output)


if __name__ == "__main__":
    main()