# ME 481 - Week 5: P Position Control

ESP32 PlatformIO project for proportional (P) position control of the gearmotor output shaft using a VNH5019 motor driver and quadrature encoder feedback.

## Hardware Setup
- **Microcontroller**: ESP32 Development Board (`esp32dev`)
- **Motor Driver**: VNH5019
  - INA: GPIO 15
  - INB: GPIO 2
  - PWM: GPIO 17 (5 kHz, 8-bit, limited to 180)
  - CS (Current Sense): GPIO 32 (ADC1_CH4, ~140 mV/A; not used by the P controller)
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
pio device monitor --baud 115200
```

### Usage
1. Open the Serial Monitor at 115200 baud. `READY` is printed once at boot.
2. Press `s` to begin a trial. The current shaft position becomes 0 degrees. The encoder is logged with the motor off for `PRE_STEP_US` (0.5 s), then at `time_s = 0` the reference steps to `REFERENCE_DEG` (45 degrees) and the controller turns on. Baseline rows have negative time.
3. The serial monitor streams `time_s,reference_deg,angle_deg,error_deg,u_raw,pwm` at 200 Hz. Every line is also saved to `logs/device-monitor-*.log` (the `log2file` monitor filter), since a full run is longer than the VS Code terminal scrollback.
4. The motor stops automatically `TRIAL_DURATION_S` (15 s) after the step, or press `x` to stop it early (`x` before the step aborts the trial). Logging continues with the motor off for `POST_STOP_US` (1 s, `pwm` = 0) so the shaft coming to rest is captured, then `END` is printed.
5. Change `Kp` in `src/main.cpp` for each experiment.

The stream contains only the CSV header, data rows, and these cue lines:

| Line | Meaning |
|------|---------|
| `READY` | Board booted (printed once) |
| `MOTOR_ON` | The next row is the first with the motor on (`time_s = 0`) |
| `MOTOR_OFF` | The next row is the first with the motor off |
| `END` | Logging stopped |

### Logging to CSV with MATLAB
1. Close the PlatformIO serial monitor (only one program can have the port open).
2. Run `log_trial.m`. It finds the ESP32 port, starts a trial, and records until `END`, using the `MOTOR_ON` / `MOTOR_OFF` cues to find when the motor turned on and off.
3. It saves `data/trial_<date>_<time>.csv`, trimmed from `PRE_S` (0.25 s) before the motor turns on to `POST_S` (0.5 s) after the shaft has come to rest with the motor off. Set `NAME_TAG` (e.g. `"Kp10"`) to label the file. The trimmed data is also left in the workspace as the table `trial`.
