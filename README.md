# M5StickC Plus Vibration Analyzer

A portable 3-axis vibration analyzer built on the M5StickC Plus platform. Features real-time FFT analysis, spectrogram visualization, persistent data storage, and Bluetooth connectivity.

![Version](https://img.shields.io/badge/version-1.7-blue.svg)
![Platform](https://img.shields.io/badge/platform-M5StickC%20Plus-orange.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

## 📋 Table of Contents

- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Software Dependencies](#-software-dependencies)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [User Interface](#-user-interface)
- [Serial Commands](#-serial-commands)
- [Technical Specifications](#-technical-specifications)
- [Data Export Format](#-data-export-format)
- [Bluetooth Protocol](#-bluetooth-protocol)
- [Circuit Connections](#-circuit-connections)
- [Troubleshooting](#-troubleshooting)
- [Version History](#-version-history)
- [Contributing](#-contributing)
- [License](#-license)
- [Author](#-author)

## ✨ Features

### Core Functionality
- **3-Axis Acceleration Measurement** - Simultaneous X, Y, Z axis vibration capture
- **Real-time FFT Analysis** - 1024-point FFT with Hamming window
- **Peak Frequency Detection** - Automatic identification of dominant frequencies per axis
- **Statistical Analysis** - MIN, MAX, MEAN, and Standard Deviation calculations

### Visualization (6 Display Screens)
1. **Startup Screen** - Device info, RTC time, storage status
2. **Statistics** - Numerical summary of recorded data
3. **Time Series** - Waveform plot of all three axes
4. **Spectrum** - Frequency domain representation
5. **Correlation** - X-Y and X-Z phase plots
6. **Spectrogram** - Time-frequency heat map visualization

### Data Management
- **Persistent Storage** - Up to 40 records stored in flash memory
- **Circular Buffer** - Automatic overwrite of oldest records when full
- **RTC Timestamps** - Each record tagged with real-time clock timestamp
- **Data Export** - Full data export via Serial port

### Connectivity
- **Bluetooth Serial** - Wireless data transmission to Android apps
- **USB Serial** - Direct PC connection for data export and RTC setting
- **External Trigger** - Optional external trigger input for synchronized measurements

## 🔧 Hardware Requirements

### Required
| Component | Specification |
|-----------|---------------|
| **Microcontroller** | M5StickC Plus (ESP32-based) |
| **Built-in IMU** | MPU6886 (3-axis accelerometer + gyroscope) |
| **Display** | 1.14" TFT LCD (135×240 pixels) |
| **Flash Storage** | 4MB (≈1.5MB available for data) |
| **RTC** | BM8563 with backup battery |

### Optional
| Component | Purpose | Pin |
|-----------|---------|-----|
| External Trigger | Synchronized measurement start | GPIO 36 |
| Oscilloscope Probe | Sample rate verification | GPIO 26 (CALIB_T_PIN) |

## 📚 Software Dependencies

### Arduino IDE Setup
1. Install [Arduino IDE](https://www.arduino.cc/en/software) (1.8.x or 2.x)
2. Add ESP32 board support:
   - File → Preferences → Additional Board Manager URLs:
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```
3. Install M5StickC Plus board package

### Required Libraries

| Library | Version | Installation |
|---------|---------|--------------|
| **M5StickCPlus** | ≥0.1.0 | Library Manager: "M5StickCPlus" |
| **arduinoFFT** | ≥2.0.0 | Library Manager: "arduinoFFT" |
| **BluetoothSerial** | (included) | Part of ESP32 core |
| **LittleFS** | (included) | Part of ESP32 core |
| **Preferences** | (included) | Part of ESP32 core |

### Installation via Library Manager
```
Sketch → Include Library → Manage Libraries...
Search and install: "M5StickCPlus", "arduinoFFT"
```

## 📥 Installation

### Step 1: Clone Repository
```bash
git clone https://github.com/yourusername/m5stick-vibration-analyzer.git
cd m5stick-vibration-analyzer
```

### Step 2: Open in Arduino IDE
```
File → Open → VibrationAnalyzer_v1.7_RTC.ino
```

### Step 3: Configure Board
```
Tools → Board → ESP32 Arduino → M5Stick-C-Plus
Tools → Upload Speed → 1500000
Tools → Port → [Select your COM port]
```

### Step 4: Upload
```
Sketch → Upload (Ctrl+U)
```

### Step 5: Set RTC Time
After upload, open Serial Monitor (115200 baud) and send:
```
T2025-01-15 14:30:00
```
(Replace with current date/time)

## 🚀 Quick Start

### Basic Recording
1. **Power on** the device
2. Wait for startup screen (shows RTC time and storage status)
3. Press **BtnA** to start recording
4. LED turns on during recording (~4 seconds for 2048 samples)
5. Press **BtnB** to cycle through result screens

### Viewing Stored Records
1. Navigate to **Screen 6** (Storage Browser) using BtnB
2. Press **BtnA** to browse through stored records
3. Selected record data is shown on all other screens

## 🎮 User Interface

### Button Controls

| Button | Press Type | Action |
|--------|-----------|--------|
| **BtnA** | Short | Start new recording |
| **BtnA** | Short (on Screen 6) | Browse previous record |
| **BtnB** | Short | Cycle to next screen |
| **BtnB** | Long (700ms) | Serial data dump / Clear all (Screen 6) |
| **BtnB** | Hold at boot | Enable external trigger mode |

### Screen Descriptions

#### Screen 0: Startup
```
┌────────────────────────────┐
│ Vibration analyser         │
│ ver. 1.7 + RTC             │
│ Vladimir Divic, 2025       │
│ BT: Vibe002                │
│ RTC: 2025-01-15 14:30:00   │
│ Storage: 12/50 rec         │
│ BtnA-rec BtnB-screen       │
│ Self-trigger mode          │
└────────────────────────────┘
```

#### Screen 1: Statistics
Displays numerical analysis:
- Sample count (N) and sampling frequency (Fs)
- MAX, MEAN, SD, MIN for each axis
- Peak frequency for each axis
- Color coded: X=Red, Y=Green, Z=Blue

#### Screen 2: Time Series
- Real-time waveform of all 3 axes
- Auto-scaled Y-axis (acceleration in g)
- Zero reference line

#### Screen 3: Spectrum
- FFT magnitude spectrum (0 to Fs/2)
- Peak frequency annotation on dominant axis
- Logarithmic-like display

#### Screen 4: Correlation
- Left plot: X-Y correlation (orange)
- Right plot: X-Z correlation (magenta)
- Useful for identifying vibration modes

#### Screen 5: Spectrogram
- Time-frequency representation
- Color scale: Blue (low) → Cyan → Green → Yellow → Red (high)
- Total acceleration magnitude
- 128-point segments, 50% overlap

#### Screen 6: Storage Browser
- Current RTC time
- Record count and flash usage
- Currently selected record with timestamp
- Navigation instructions

### LED Indicator

| LED State | Meaning |
|-----------|---------|
| OFF | Idle / Ready |
| ON | Recording in progress |

## 💻 Serial Commands

Connect via USB Serial at **115200 baud**.

| Command | Description | Example |
|---------|-------------|---------|
| `R` | Trigger recording | `R` |
| `T` | Print current RTC time | `T` |
| `Tyyyy-MM-dd HH:mm:ss` | Set RTC time | `T2025-01-15 14:30:00` |
| `E` | Export all stored records | `E` |
| `I` | Print storage info | `I` |
| `H` or `?` | Show help | `H` |

### Setting RTC Time
```bash
# Format: Tyyyy-MM-dd HH:mm:ss
# Example:
T2025-01-15 14:30:00
```

### Expected Response
```
RTC set to: 2025-01-15 14:30:00
```

## 📊 Technical Specifications

### Measurement Parameters

| Parameter | Value |
|-----------|-------|
| Sample Size | 1024 samples |
| Sampling Rate | ~200 Hz (variable, measured) |
| Recording Duration | ~5 seconds |
| Accelerometer Range | ±2g (default) |
| Accelerometer Resolution | 16-bit |
| FFT Window | Hamming |
| FFT Size | 1024 points |
| Frequency Resolution | Fs/1024 ≈ 0.2 Hz |
| Max Frequency | Fs/2 ≈ 100 Hz |

### Storage Specifications

| Parameter | Value |
|-----------|-------|
| Flash Available | ~1.5 MB |
| Record Size | ~30 KB |
| Max Records | 50 |
| Data per Record | 3×1024 floats (time) + 3×1024 floats (spectrum) |
| Metadata | Timestamp, statistics, peak frequencies |

### Spectrogram Parameters

| Parameter | Value |
|-----------|-------|
| Segment Length | 128 samples |
| Overlap | 50% |
| Max Segments | 17 |
| Frequency Bins | 64 |
| Dynamic Range | 60 dB (3 decades) |

## 📁 Data Export Format

### Serial Export Structure
When using the `E` command:

```
=== BEGIN EXPORT ===
RECORD_COUNT:12
EXPORT_TIME:2025-01-15 14:35:00

=== RECORD 0 ===
TIMESTAMP:2025-01-15 10:23:45
SAMPLING_FREQ:198.5
PEAK_FREQ:12.345,8.765,15.432
TIME_DATA:
0.00123,0.00456,0.98765
0.00234,0.00567,0.98654
... (2048 lines)
SPECTRUM:
0.000,0.00012,0.00023,0.00034
0.097,0.00123,0.00234,0.00345
... (1024 lines: freq,X,Y,Z)

=== RECORD 1 ===
...

=== END EXPORT ===
```

### Data Format Details

| Field | Format | Description |
|-------|--------|-------------|
| TIMESTAMP | yyyy-MM-dd HH:mm:ss | Recording start time |
| SAMPLING_FREQ | float | Actual sampling frequency (Hz) |
| PEAK_FREQ | float,float,float | Peak frequency X,Y,Z (Hz) |
| TIME_DATA | float,float,float | Acceleration X,Y,Z (g) |
| SPECTRUM | float,float,float,float | Frequency, Magnitude X,Y,Z |

## 📡 Bluetooth Protocol

### Connection
- Device Name: `Vibe002`
- Protocol: Bluetooth Classic SPP
- Compatible with: "Bluetooth Electronics" Android app

### Data Packets

#### Timestamp Packet
```
*Tyyyy-MM-dd HH:mm:ss*
```

#### Time Domain Data
```
*KC*                          // Clear command
*KX[time]Y[accX],X[time]Y[accY],X[time]Y[accZ]*
```

#### Frequency Domain Data
```
*HC*                          // Clear command
*HX[freq]Y[magX],X[freq]Y[magY],X[freq]Y[magZ]*
```

#### Peak Frequencies
```
*X[peakX]*
*Y[peakY]*
*Z[peakZ]*
```

### Bluetooth Commands

| Command | Action |
|---------|--------|
| `R` | Trigger recording |
| `T` | Get current time |
| `I` | Get storage info |

## 🔌 Circuit Connections

### External Trigger (Optional)
```
External Signal ──┬── GPIO 36 (EXT_TRIG_PIN)
                  │
                 GND
                  
Trigger Level: > 4000 (ADC value, ~3.2V)
```

### Calibration Output (Optional)
```
GPIO 26 (CALIB_T_PIN) ──── Oscilloscope
                           
Toggles on each sample - use to verify sampling rate
```

### Pin Summary

| GPIO | Function | Direction |
|------|----------|-----------|
| 36 | External Trigger | Input |
| 26 | Calibration Toggle | Output |
| 10 | Built-in LED | Output |

## ❗ Troubleshooting

### Device won't start
- Ensure battery is charged (connect USB for 30 min)
- Try holding power button for 6 seconds

### RTC shows wrong time (2000-01-01)
- RTC backup battery may be depleted
- Set time via Serial: `T2025-01-15 14:30:00`
- Keep device powered to charge backup battery

### Storage initialization failed
- Flash may be corrupted
- Re-upload firmware (this will format flash)
- Check if LittleFS partition is configured

### Low sampling rate
- Bluetooth transmission can slow sampling
- Set `BTCONNECT = false` for maximum rate
- Check TTNS value (minimum sample interval)

### Bluetooth not connecting
- Ensure Bluetooth is enabled on phone
- Unpair and re-pair device
- Device name: "Vibe002"

### Data looks noisy
- Ensure device is firmly mounted
- Check for electrical interference
- Verify accelerometer calibration

### FFT shows unexpected peaks
- DC offset is removed automatically
- Check for mechanical resonances in mounting
- Verify Hamming window is applied

## 📜 Version History

### v1.7 (Current)
- Added RTC timestamp for each record
- Serial command to set RTC time
- Timestamp display on all screens
- Improved serial command parsing

### v1.6
- Added persistent storage (LittleFS)
- Circular buffer for up to 50 records
- Storage browser screen
- Record browsing functionality
- Export all records command

### v1.5
- Added spectrogram visualization
- Jet colormap for spectrogram
- Improved screen drawing functions

### v1.2
- Initial public release
- Basic FFT analysis
- Statistics screen
- Bluetooth connectivity

## 🔮 Planned Features

- [ ] Configurable sample rate
- [ ] Configurable record length
- [ ] Self-triggering (threshold-based)
- [ ] Multi-device synchronization
- [ ] M5Stack Core2 port

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Coding Style
- Use 4-space indentation
- Comment complex algorithms
- Keep functions under 50 lines where possible
- Use descriptive variable names

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2025 Vladimir Divic

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

## 👤 Author

**Vladimir Divic**
- Email: v.divic@gmail.com
- Year: 2025

## 🙏 Acknowledgments

- [M5Stack](https://m5stack.com/) for the excellent hardware platform
- [ArduinoFFT](https://github.com/kosme/arduinoFFT) library by Enrique Condes
- ESP32 Arduino Core developers
- Open source community

---

<p align="center">
  Made with ❤️ for the vibration analysis community
</p>
