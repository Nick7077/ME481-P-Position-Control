# ME 481 - Week 1: Motor and Encoder Test

ESP32 PlatformIO project for testing motor driver (VNH5019) direction, PWM speed control, optical/quadrature encoder feedback, and analog current-sense measurement.

## Hardware Setup
- **Microcontroller**: ESP32 Development Board (`esp32dev`)
- **Motor Driver**: VNH5019
  - INA: GPIO 15
  - INB: GPIO 2
  - PWM: GPIO 17 (20 kHz, 8-bit)
  - CS (Current Sense): GPIO 32 (ADC1_CH4, ~140 mV/A)
- **Quadrature Encoder**:
  - Phase A: GPIO 14
  - Phase B: GPIO 27

## Getting Started

### Prerequisites
- [PlatformIO](https://platformio.org/) installed in VS Code or CLI.

### Build and Upload
```bash
# Build project
pio run

# Upload to ESP32
pio run --target upload

# Open Serial Monitor
pio device monitor --baud 921600
```

### Usage
1. Open the Serial Monitor at 921600 baud.
2. Send `'s'` over the serial connection to initiate the 200 ms motor test run.
3. The serial monitor streams timestamp, encoder count, calculated speed, and current-sense data.
