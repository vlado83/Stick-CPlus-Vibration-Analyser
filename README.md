# M5StickC Plus Vibration Analyzer (M5Unified)

A portable **3-axis vibration analyzer** for **M5StickC Plus (ESP32)**.  
Records accelerometer time series, computes **FFT + peak list**, shows **time series / spectrum / correlation / spectrogram**, estimates **logarithmic decrement** from an exponential decay fit, and stores runs to **LittleFS** with an **RTC timestamp**.

![Firmware](https://img.shields.io/badge/firmware-1.14%20(M5Unified)-blue.svg)
![Platform](https://img.shields.io/badge/platform-M5StickC%20Plus-orange.svg)
![Storage](https://img.shields.io/badge/storage-LittleFS-purple.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

> Note: The sketch filename is `VibrationAnalyzer_v1_15.ino`, while the on-device startup screen prints:  
> `ver. 1.14 (M5Unified)` (this is the firmware string inside the code).

---

## 📋 Table of Contents

- [✨ Features](#-features)
- [🔧 Hardware Requirements](#-hardware-requirements)
- [📚 Software Dependencies](#-software-dependencies)
- [📥 Installation](#-installation)
- [🚀 Quick Start](#-quick-start)
- [🎮 User Interface](#-user-interface)
- [💻 Serial Commands](#-serial-commands)
- [📊 Technical Specifications](#-technical-specifications)
- [📁 Data Export Format](#-data-export-format)
- [🔌 Circuit Connections](#-circuit-connections)
- [❗ Troubleshooting](#-troubleshooting)
- [📜 Version History](#-version-history)
- [🔮 Planned Features](#-planned-features)
- [🤝 Contributing](#-contributing)
- [📄 License](#-license)
- [👤 Author](#-author)
- [🙏 Acknowledgments](#-acknowledgments)

---

## ✨ Features

### Core Functionality
- **3-Axis Acceleration Measurement** (X, Y, Z)
- **Configurable acquisition**
  - Samples: **512 / 1024 / 2048**
  - Nominal Fs: **100 / 200 / 400 / 800 / 1000 Hz** (actual Fs is measured from timestamps)
- **On-device analysis**
  - MIN / MAX / MEAN / SD
  - FFT (Hamming window + DC removal)
  - Automatic peak detection & peak list
  - Envelope + exponential decay fit → **log decrement** and **R²**
- **Persistent storage**
  - LittleFS circular buffer (default: **40 records**)
  - Auto-overwrite oldest record when full
  - Each record tagged with **RTC timestamp**
- **External trigger mode** (optional analog trigger on GPIO36)
- **Intro animation** (tilt-biased multi-walker) from `vibe_intro.cpp/.h`

### Visualization (10 Display Screens + Startup)
0. **Startup** – version, RTC time, storage status
1. **Trigger Setup**
2. **Acquisition Setup**
3. **Statistics**
4. **Time Series**
5. **Envelope / Log Decrement**
6. **Spectrum**
7. **FFT Peaks**
8. **Correlation (X-Y and X-Z)**
9. **Spectrogram**
10. **Storage Browser**

### Connectivity
- **USB Serial (115200 baud)** – export data, set RTC, select stored records, etc.
- **Bluetooth is NOT used in this version** (Serial-only).

---

## 🔧 Hardware Requirements

### Required

| Component | Specification |
|---|---|
| **Microcontroller** | M5StickC Plus (ESP32-based) |
| **Built-in IMU** | MPU6886 (accel + gyro; accel used here) |
| **Display** | 1.14" TFT LCD (135×240) |
| **Flash** | 4MB (LittleFS partition required) |
| **RTC** | BM8563 (backup battery supported) |

### Optional

| Component | Purpose | Pin |
|---|---|---|
| External Trigger Signal | Synchronized measurement start | GPIO 36 (ADC) |
| Oscilloscope Probe | Sample-rate verification | GPIO 26 (CALIB_T_PIN) |

---

## 📚 Software Dependencies

### Arduino IDE Setup
1. Install Arduino IDE (1.8.x or 2.x)
2. Install ESP32 board support:
   - **File → Preferences → Additional Board Manager URLs**
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```
3. Install a board entry that supports **M5StickC Plus** (or use a compatible ESP32 target)

### Required Libraries

| Library | Notes | Installation |
|---|---|---|
| **M5Unified** | Display + IMU + RTC abstraction | Library Manager: “M5Unified” |
| **arduinoFFT** | FFT computation | Library Manager: “arduinoFFT” |
| **LittleFS** | Filesystem | Included in ESP32 core |
| **Preferences** | NVS storage for user settings | Included in ESP32 core |

---

## 📥 Installation

### Step 1: Clone Repository
```bash
git clone https://github.com/yourusername/m5stick-vibration-analyzer.git
cd m5stick-vibration-analyzer
```

### Step 2: Open in Arduino IDE
Open:
```
VibrationAnalyzer_v1_15.ino
```

### Step 3: Configure Board
Typical settings (may vary by ESP32 core version):
- **Board:** M5StickC Plus (or compatible)
- **Upload Speed:** 1500000
- **Port:** select your COM port
- **Partition Scheme:** choose one that includes **SPIFFS/LittleFS**
  - Important: this firmware stores data in **LittleFS**

### Step 4: Upload
```
Sketch → Upload (Ctrl+U)
```

### Step 5: Set RTC Time
Open Serial Monitor at **115200 baud** and send:
```
T2026-03-01 12:00:00
```
(Replace with current date/time)

---

## 🚀 Quick Start

### Basic Recording
1. Power on the device
2. Wait for **Startup Screen** (shows RTC + storage)
3. Press **BtnB** to go to:
   - **Screen 1 (Trigger Setup)** and select trigger mode
   - **Screen 2 (Acquisition Setup)** and select N + nominal Fs
4. Press **BtnA** to arm/start a new recording (depends on trigger mode)
5. Use **BtnB** to browse result screens

### Viewing Stored Records
1. Navigate to **Screen 10 (Storage Browser)** using BtnB
2. Press **BtnA** to cycle through stored records
3. The selected record is displayed on all analysis screens

### External Trigger Mode (Boot-time)
- Hold **BtnB during boot** to enable external trigger mode
- Arm with **BtnA**
- Recording starts when **GPIO36 ADC > 4000**

---

## 🎮 User Interface

### Button Controls (global)

| Button | Press Type | Action |
|---|---|---|
| **BtnB** | Short | Next screen |
| **BtnB** | Long (~700ms) | Context action / serial dump (varies by screen) |
| **BtnA** | Short | Action (arm/record, change value, browse record) |
| **BtnA** | Long | Cursor/selection move (setup screens) |
| **BtnB (boot)** | Hold | External trigger mode |

### Screen-Specific Notes

#### Screen 1: Trigger Setup
- **BtnA short:** change current value  
- **BtnA long:** move cursor to next item  
Trigger modes:
- Immediate
- Delay (1 / 3 / 10 s)
- Accel threshold (|a| in g, adjustable)

#### Screen 2: Acquisition Setup
- **BtnA short:** change selected value  
- **BtnA long:** switch between “Samples” and “Fs (nominal)”

#### Screen 10: Storage Browser
- **BtnA:** browse records
- **BtnB long:** clear all records

---

## 💻 Serial Commands

Connect via USB Serial at **115200 baud**.

| Command | Description | Example |
|---|---|---|
| `R` | Arm recording (uses current Trigger settings) | `R` |
| `A` | Dump current time series (`idx t_us ax ay az`) | `A` |
| `F` | Dump current FFT (`f_Hz X Y Z`) | `F` |
| `S` | Dump summary: stats + FFT peaks + log decrement | `S` |
| `Cxx` | Load record `xx` (0..N-1); `C-1` for LIVE | `C0` |
| `T` | Print current RTC time | `T` |
| `Tyyyy-MM-dd HH:mm:ss` | Set RTC time | `T2026-03-01 12:00:00` |
| `E` | Export all stored records (ASCII) | `E` |
| `I` | Print storage info | `I` |
| `H` or `?` | Help | `H` |

---

## 📊 Technical Specifications

### Measurement Parameters

| Parameter | Value |
|---|---|
| Sample Size | 512 / 1024 / 2048 |
| Nominal Sampling Rate | 100 / 200 / 400 / 800 / 1000 Hz |
| Actual Sampling Rate | computed from timestamps |
| Accelerometer Units | g |
| FFT Window | Hamming |
| DC Offset | removed before FFT |
| FFT Output | magnitude spectrum (stored per axis) |

### Storage Specifications

| Parameter | Value |
|---|---|
| Storage | LittleFS |
| Max Records | 40 (circular buffer) |
| Record Payload | Header + accX/accY/accZ + specX/specY/specZ |
| Array Length | `N` floats per array (N = sample count) |
| Metadata | RTC timestamp + statistics + peak frequencies |

### Spectrogram Parameters

| Parameter | Value |
|---|---|
| Segment Length | 128 samples |
| Overlap | 50% |
| Max Segments | 17 |
| Frequency Bins | 65 (0..Nyquist for SEG_LEN=128) |

---

## 📁 Data Export Format

### Export-All (Serial, `E` command)

```
=== BEGIN EXPORT ===
RECORD_COUNT:<N>
EXPORT_TIME:<yyyy-MM-dd HH:mm:ss>

=== RECORD 0 ===
TIMESTAMP:<yyyy-MM-dd HH:mm:ss>
SAMPLING_FREQ:<float>
PEAK_FREQ:<peakX>,<peakY>,<peakZ>
TIME_DATA:
<ax>,<ay>,<az>
<ax>,<ay>,<az>
...
SPECTRUM:
<f>,<magX>,<magY>,<magZ>
<f>,<magX>,<magY>,<magZ>
...

=== END EXPORT ===
```

### Field Details

| Field | Format | Description |
|---|---|---|
| TIMESTAMP | yyyy-MM-dd HH:mm:ss | Recording start time (RTC) |
| SAMPLING_FREQ | float | Measured sampling frequency (Hz) |
| PEAK_FREQ | float,float,float | Peak frequency X,Y,Z (Hz) |
| TIME_DATA | float,float,float | Acceleration X,Y,Z (g) |
| SPECTRUM | float,float,float,float | Frequency, Magnitude X,Y,Z |

---

## 🔌 Circuit Connections

### External Trigger (Optional)
```
External Signal ──┬── GPIO 36 (EXT_TRIG_PIN, ADC)
                  │
                 GND

Trigger condition: analogRead(GPIO36) > 4000
```

### Calibration Output (Optional)
```
GPIO 26 (CALIB_T_PIN) ──── Oscilloscope

Toggles during sampling - use to verify sampling rate
```

### Pin Summary

| GPIO | Function | Direction |
|---|---|---|
| 36 | External Trigger (ADC) | Input |
| 26 | Calibration Toggle | Output |
| 19 | M5StickC Plus LED (M5_LED) | Output |

---

## ❗ Troubleshooting

### Device won’t start
- Ensure battery is charged (connect USB for ~30 min)
- Try holding the power button for ~6 seconds

### RTC shows wrong time (e.g. 2000-01-01)
- Set time via Serial:
  - `T2026-03-01 12:00:00`
- RTC backup battery may be depleted (keep device powered to recharge, if applicable)

### Storage initialization failed / no records saved
- Ensure your **Partition Scheme includes SPIFFS/LittleFS**
- Re-upload firmware (filesystem may be formatted on first mount)

### External trigger never fires
- Confirm wiring to **GPIO36**
- Ensure signal amplitude reaches the ADC threshold (`> 4000`)

### Data looks noisy
- Mount device firmly to the structure
- Reduce cable tug / handling during acquisition
- Verify measurement axis orientation and mounting stiffness

### FFT shows unexpected peaks
- DC offset is removed automatically (but mounting resonance still shows up)
- Check for resonant mounting fixtures / loose screws
- Confirm sampling settings and record length

---

## 📜 Version History

### v1.14 (M5Unified) — current firmware string
- M5Unified-based firmware
- Trigger + acquisition setup screens
- Envelope + log decrement fit + R² display
- Spectrogram screen
- LittleFS circular buffer storage with RTC timestamp
- Serial export and record selection (`Cxx`)
- Bluetooth removed (Serial-only)

> Older pre-M5Unified versions existed (Bluetooth/earlier UI), see repository tags/branches if present.

---

## 🔮 Planned Features

- [ ] User-configurable TTNS / calibrated sampling loop
- [ ] Optional FIR/IIR filtering + decimation workflow
- [ ] PC-side parser scripts (Python/MATLAB) for `E` export
- [ ] Multi-device synchronization (ESP-NOW / wired trigger)
- [ ] More robust peak picking (prominence, harmonic grouping, mode tracking)

---

## 🤝 Contributing

Contributions are welcome!

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add AmazingFeature'`)
4. Push (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Coding Style
- Use 4-space indentation
- Keep functions small and readable
- Comment signal-processing logic and file formats
- Prefer explicit constants for UI dimensions and scaling

---

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2025-2026 Vladimir Divić

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 👤 Author

**Vladimir Divić**  
- Email: v.divic@gmail.com

---

## 🙏 Acknowledgments

- M5Stack for the hardware platform
- ArduinoFFT library by Enrique Condes (kosme)
- ESP32 Arduino Core developers
- Open-source community

---

<p align="center">
  Made with ❤️ for the vibration / modal analysis community
</p>
