# SPIRE — Project Checkpoint
**Data: 2026-04-17**
**Status: Early Development**

---

## 1. Architektura systemu

### Procesy (Python, multiprocessing)

| Proces | Funkcja | Częstotliwość | IPC |
|--------|---------|---------------|-----|
| `imu_reader` | Odczyt IMU (gyro+accel) | 1 kHz target | → shared memory |
| `capture` | Zdjęcia JPEG+DNG + metadane | co 2s (konfigurowalne) | ← shared memory, → pliki+CSV |
| `servo_ctrl` | PID kompensacja yaw | 100 Hz | ← shared memory, → PWM |
| `data_logger` | Logowanie CSV + telemetria | 10 Hz | ← shared memory |
| `gps_reader` | Odczyt NMEA (czas UTC + pozycja) | 1 Hz | → shared memory |

### Komunikacja międzyprocesowa
- **Shared memory** (`multiprocessing.shared_memory`) — stan IMU, timestamp, pozycja GPS
- **Command queue** (`multiprocessing.Queue`) — start/stop, zmiana parametrów

### Języki
- **Python** — capture pipeline, sterowanie, logowanie, orkiestracja
- **C++** — zarezerwowany dla deblurringu (przetwarzanie obrazu real-time)

---

## 2. Hardware

### Platformy
| Płytka | Rola | System | Status |
|--------|------|--------|--------|
| RPi 5 Model B Rev 1.0 | Flight computer (docelowy) | RPi OS Lite (Trixie) | Kamera OK (CAM/DISP 1, 22-pin) |
| RPi 4 Model B | Dev/test | RPi OS Lite (Debian 13 Trixie, kernel 6.12) | Kamera OK, venv OK, user `revan` |

### Komponenty dostępne
- 2x RPi HQ Camera V1.0 (IMX477, 6mm lens)
- 1x MPU6050 (6-axis IMU, I2C 0x68)
- 1x Waveshare L76K GPS HAT (UART, NMEA)
- 2x DFRobot 16-bit ADC Gravity v1.0 (to NIE jest IMU)

### Komponenty zamówione (w drodze)
- 4x MPU6886 (6-axis IMU, M5Stack)
- 1x LSM9DS1 (9-DoF IMU, Adafruit)
- 1x TCA9548A (I2C multiplexer, Grove 8-port)
- 1x TD-6622MG (serwo yaw, 20 kg·cm)
- 1x PCA9685 (16-ch PWM driver, I2C)
- 1x IMX477 + obiektyw 25mm C-mount

---

## 3. Software — stan obecny

### Repozytorium
- **GitHub:** https://github.com/Krypel20/SPIRE
- **Branch dev:** `dev-test` (testowanie na RPi)
- **Branch prod:** `main` (sprawdzony kod)

### Struktura katalogów
```
SPIRE/
├── src/
│   └── capture.py          # ✅ Działa — capture pipeline
├── tests/
├── data/                   # .gitignore — nie trafia do repo
├── docs/
└── .gitignore
```

### capture.py — status
- ✅ Cykliczne zdjęcia z interwałem
- ✅ Ręczna kontrola ekspozycji (1/150s – 1/1000s)
- ✅ Zapis JPEG + opcjonalnie DNG (RAW)
- ✅ Logowanie metadanych do CSV (timestamp, exposure, gain, lux)
- ✅ Monotonic timestamp do synchronizacji z IMU
- ⚠️ Do poprawki: `datetime.utcnow()` → `datetime.now(datetime.UTC)`

### Komendy na RPi
```bash
# Aktywacja venv
source ~/payload/.venv/bin/activate

# Test capture (3 zdjęcia, bez RAW)
cd ~/SPIRE
python3 src/capture.py -n 3 -i 2 --no-raw -o data/test_capture

# Sprawdzenie kamery
rpicam-hello --list-cameras

# Pull z GitHub
git pull
```

---

## 4. Kluczowe parametry techniczne

| Parametr | Wartość | Źródło |
|----------|---------|--------|
| Blur budget | < 1 pixel | Concept doc |
| Max angular velocity (1/150s) | ~0.53°/s | Obliczone |
| Pixel pitch IMX477 | 1.55 µm | Datasheet |
| Ogniskowa docelowa | 25 mm | Concept doc |
| Sensor resolution | 4056 × 3040 (12.3 MP) | Datasheet |
| I2C max realistic (5 IMU) | ~400 Hz | Obliczone (TCA9548A overhead) |
| Processing latency target | ≤ 200 ms/frame | Concept doc |
| Power budget | < 12 W | Concept doc |
| System mass target | < 1000 g | Concept doc |

---

## 5. Znane problemy i ryzyka

| Problem | Status | Rozwiązanie |
|---------|--------|-------------|
| `vcgencmd get_camera` nie działa na Trixie | ✅ Rozwiązany | Użyj `rpicam-hello --list-cameras` |
| USB gadget mode powoduje boot loop | ✅ Rozwiązany | Nie używać, SSH przez hotspot |
| RPi 4 vs RPi 5 mylone przez hostname | ✅ Rozwiązany | Weryfikuj: `cat /proc/device-tree/model` |
| RPi 5 wymaga 22-pin taśmy CSI | ✅ Rozwiązany | Osobna taśma 22-pin |
| I2C 5 IMU @ 1 kHz nieosiągalne | ⚠️ Otwarte | Rozdzielić na 2 busy I2C lub obniżyć rate |
| Serwo TD-6622MG bez encodera | ⚠️ Otwarte | Feedback z IMU zamiast encodera |

---

## 6. Kolejne kroki (priorytet)

1. **`imu_reader`** — odczyt MPU6050 przez I2C, zapis do shared memory
2. **Integracja IMU ↔ capture** — synchronizacja timestampów
3. **`gps_reader`** — odczyt NMEA z L76K, czas UTC
4. **`servo_ctrl`** — sterowanie TD-6622MG przez PCA9685, PID
5. **Deblurring pipeline** — C++, na podstawie trajektorii IMU
6. **Testy naziemne** — symulacja ruchu, weryfikacja stabilizacji

---

## 7. Useful commands reference

```bash
# Identyfikacja płytki
cat /proc/device-tree/model

# Konfiguracja kamery
cat /boot/firmware/config.txt | grep camera  # → camera_auto_detect=1
rpicam-hello --list-cameras
rpicam-still --nopreview -o test.jpg

# I2C diagnostyka
sudo i2cdetect -y 1    # RPi 4 main bus
sudo i2cdetect -y 4    # RPi 5 CAM bus

# Sieć
sudo nmtui              # konfiguracja Wi-Fi/hotspot
hostname -I              # sprawdź IP

# Git workflow
git checkout dev-test
git pull
# ... test na RPi ...
git checkout main
git merge dev-test
```
