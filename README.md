# Fall Prediction Device

A wearable pre-impact fall prediction system combining **EEG** (electroencephalography) and **IMU** (inertial measurement) sensing on an **STM32** microcontroller. Designed to detect the early neural and biomechanical signatures of an oncoming fall — providing lead time for intervention before ground impact.

![Platform](https://img.shields.io/badge/platform-STM32-blue)
![Language](https://img.shields.io/badge/language-Arduino_C++-orange)
![Status](https://img.shields.io/badge/status-research_prototype-yellow)
![License](https://img.shields.io/badge/license-MIT-green)

---

## ⚠️ Disclaimer

**This is a research and educational prototype, not a medical device.** It has not been validated, certified, or approved for clinical use, fall prevention, or any safety-critical application. Do not rely on this system for the health or safety of any person.

---

## Overview

Most fall detection systems trigger *after* impact has occurred. This project takes a different approach: it predicts falls **before** they happen by combining two physiological signals:

- **EEG** — detects theta/delta slow-wave intrusion, an early indicator of pre-syncope (vasovagal / orthostatic falls)
- **IMU** — detects pre-impact biomechanics: freefall acceleration, body rotation, and trunk tilt

A pre-impact alert can drive downstream safety actions such as a wearable airbag, muscle bracing signal, or caregiver notification.

```
┌──────────┐    SPI    ┌─────────┐
│ ADS1299  ├──────────►│         │
│ (EEG x3) │           │         │     ┌────────────┐
└──────────┘           │  STM32  ├────►│ ALERT (PA0)│
                       │         │     └────────────┘
┌──────────┐    I²C    │         │     ┌────────────┐
│  BNO055  ├──────────►│         ├────►│ USB Serial │
│  (IMU)   │           └─────────┘     └────────────┘
└──────────┘
```

---

## Features

### Signal Acquisition
- 3-channel EEG at 250 SPS, 24-bit resolution (gain ×24)
- 9-DOF IMU at 100 Hz with onboard sensor fusion
- Hardware lead-off detection per electrode

### Signal Processing
- 60 Hz (or 50 Hz) IIR notch filter for powerline rejection
- 0.5–30 Hz EEG bandpass with DC removal
- 5 Hz Butterworth low-pass on IMU axes
- Complementary filter for accurate tilt angle
- EMG burst detection to suppress muscle artifacts

### Fall Prediction Algorithm
- Theta/delta slow-wave intrusion detection (MAD-scaled z-score)
- Multi-electrode agreement voting (≥2 of 3 channels)
- 5-state IMU pre-impact machine: `STABLE → DISTURBED → FREEFALL → FALLING → IMPACT → POST_FALL`
- 200 ms three-stage IMU gate (ASVM → GSVM → tilt)
- ±2 s coincidence window between EEG and IMU triggers
- Motion-gated EEG suppression (freezes when GSVM > 100 dps)
- Adaptive baseline with 60 s time constant
- 5 s refractory lockout
- IMU-only fallback mode when electrodes are offline

### Output
- CSV stream over USB serial @ 115200 baud
- Tagged `ALERT` messages with confidence level
- Real-time electrode status reporting

---

## Hardware

### Bill of Materials

| Component | Notes |
|-----------|-------|
| STM32F103C8T6 (Blue Pill) | 72 MHz Cortex-M3, 64 KB Flash, 20 KB RAM |
| ADS1299EEGFE-PDK | TI 8-channel 24-bit EEG/AFE evaluation board |
| MMB0 Modular EVM Motherboard | Powers ADS1299 (USB-B) |
| BNO055 | 9-DOF orientation sensor (Adafruit or generic breakout) |
| EEG electrodes | Wet or dry; cervical / mastoid / occipital placement |
| Jumper wires | Female-female recommended for testing |
| 2× USB cables | One for MMB0, one for STM32 |

### Wiring

**ADS1299 ↔ STM32 (SPI1)**

| ADS1299 | STM32 | Function |
|---------|-------|----------|
| DIN | PA7 | SPI MOSI |
| DOUT | PA6 | SPI MISO |
| SCLK | PA5 | SPI Clock |
| CS | PB0 | Chip Select |
| DRDY | PB1 | Data Ready (interrupt) |
| DVDD | 3.3V | Digital power |
| DGND | GND | **Common ground (critical)** |

**BNO055 ↔ STM32 (I2C1)**

| BNO055 | STM32 |
|--------|-------|
| SDA | PB7 |
| SCL | PB8 |
| VCC | 3.3V |
| GND | GND |
| PS0, PS1, BOOT | GND (selects I²C mode) |
| ADR | GND (sets I²C address 0x28) |

**Notes**
- AVDD on the ADS1299 evaluation board is generated internally — no external connection needed
- I²C lines require 4.7 kΩ pull-ups to 3.3 V (most BNO055 breakouts include them)
- Power the ADS1299 through the MMB0's USB-B port; power the STM32 through its own USB

---

## Software Setup

### 1. Arduino IDE
Install [Arduino IDE 2.x](https://www.arduino.cc/en/software).

### 2. STM32duino Core
In **File → Preferences**, add this URL to *Additional Boards Manager URLs*:
```
https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
```
Then in **Tools → Board → Boards Manager**, search "STM32" and install *STM32 MCU based boards*.

### 3. Required Libraries
Install via **Tools → Manage Libraries**:
- Adafruit BNO055
- Adafruit Unified Sensor

### 4. Board Configuration

| Setting | Value |
|---------|-------|
| Board | Generic STM32F1 series |
| Board part number | BluePill F103C8 |
| USB support | CDC (generic Serial supersedes U(S)ART) |
| Upload method | STM32CubeProgrammer (SWD) — recommended with ST-Link |

### 5. Upload
Open `FallPrediction_v2.ino`, select port, click **Upload**. Open **Serial Monitor** at 115200 baud to see streaming output.

---

## Algorithm

### EEG Pipeline (per channel, per sample)

```
raw 24-bit ──► notch (60 Hz) ──► DC removal (HP @ 0.5 Hz)
                                       │
                ┌──────────────────────┼───────────────────┐
                ▼                      ▼                   ▼
          slow band LP @ 8 Hz   fast HP @ 8 Hz       EMG HP @ 30 Hz
                │                LP @ 30 Hz                │
                ▼                      ▼                   ▼
            x² → LP                x² → LP             x² → LP
                │                      │                   │
                └──────► slow/fast ratio ─► z-score   burst gate
                                                │
                                                ▼
                                       sustained > 500 ms? → channel trigger
```

A channel triggers when the slow/fast power ratio z-score exceeds **k × MAD** (default k=3) sustained for ≥500 ms, while not motion- or EMG-gated.

### IMU State Machine

```
STABLE ─[disturbance OR EEG slow-wave]─► DISTURBED
                                              │
                              ASVM < 0.82 g for 50 ms
                                              ▼
                                          FREEFALL
                                              │
                            GSVM > 50 dps within 200 ms
                                              ▼
                            tilt > 25° within 200 ms
                                              ▼
                                  ╔═══════════════════╗
                                  ║  PRE-IMPACT ALERT ║  (FALLING)
                                  ╚═══════════════════╝
                                              │
                                   ASVM > 2 g (impact)
                                              ▼
                                           IMPACT
                                              │
                                  300 ms — settle
                                              ▼
                                        POST_FALL
                                              │
                                  5 s refractory
                                              ▼
                                          STABLE
```

### Alert Confidence Levels

| Confidence | Meaning |
|-----------|---------|
| HIGH | EEG + IMU coincidence within ±2 s — likely syncope-type fall |
| MED | IMU-only, EEG online but quiet — likely trip/slip |
| LOW | IMU-only, EEG offline — fallback mode, stricter thresholds |

---

## Output Format

**Streaming CSV (~50 Hz):**
```
t_ms, state, eegMode, asvm_g, gsvm_dps, tilt_deg,
ch1_uv, ch2_uv, ch3_uv, stat1, stat2, stat3,
z1, z2, z3, trig
```

**Alert line (event-driven):**
```
ALERT, t=12345, conf=3, eegMode=0, asvm=0.61, gsvm=87.3, tilt=42.1, eeg_recent=1, elec=000
```

| State | Meaning | EEG Mode | Meaning | Electrode Status |
|-------|---------|----------|---------|------------------|
| 0 | STABLE | 0 | NORMAL (≥2 good) | 0 = GOOD |
| 1 | DISTURBED | 1 | DEGRADED (1 good) | 1 = FLATLINE |
| 2 | FREEFALL | 2 | OFFLINE (0 good) | 2 = SATURATED |
| 3 | FALLING | | | 3 = LEAD-OFF |
| 4 | IMPACT | | | |
| 5 | POST_FALL | | | |

---

## Tuning

All thresholds are `#define`d at the top of the firmware for easy adjustment:

```cpp
// IMU pre-impact thresholds
#define FREEFALL_G         0.82f
#define FREEFALL_MS        50
#define GYRO_THRESHOLD     50.0f
#define TILT_THRESHOLD     25.0f
#define STAGE_GATE_MS      200

// EEG thresholds
#define SLOW_RATIO_K       3.0f
#define SLOW_SUSTAIN_MS    500
#define MOTION_GATE_DPS    100.0f

// Coincidence window
#define COINCIDENCE_MS     2000

// Powerline frequency
#define POWERLINE_HZ       60   // 50 for EU/Asia, 60 for US/Canada
```

---

## Project Structure

```
fall-prediction-device/
├── README.md
├── LICENSE
├── .gitignore
├── FallPrediction_v2.ino     ← Main firmware
├── docs/
│   ├── slides/               ← Research and design slides
│   └── wiring.md             ← Detailed wiring guide
└── tools/
    └── serial_visualizer.py  ← (Coming) Live data plotter
```

---

## Roadmap

- [ ] Python serial visualizer with real-time plot
- [ ] SD card logging for offline analysis
- [ ] Bluetooth Low Energy output
- [ ] Quaternion-based 3D tilt (replace pitch/roll Euler)
- [ ] On-device data logging buffer for pre/post-trigger windows
- [ ] Machine-learning classifier upgrade (when moving to STM32H7+)

---

## References

Key research informing the algorithm design:

- Pre-impact fall detection with three-stage IMU gating using ASVM, GSVM, and vertical angle thresholds
- Theta/delta slow-wave EEG markers in pre-syncope and vasovagal events
- Complementary filter sensor fusion for IMU tilt estimation
- ADS1299 hardware lead-off detection for electrode contact quality

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

## Author

Built as a research project exploring multi-modal pre-impact fall prediction on embedded hardware.
