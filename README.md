# ME 481 - Week 9: PID Position Control

ESP32 PlatformIO project for PID position control of the gearmotor output shaft using a VNH5019 motor driver and quadrature encoder feedback. The 200 Hz control loop runs in its own FreeRTOS task on core 1, and CSV logging and serial commands run on core 0, so printing never delays a control sample.

## Hardware Setup
- **Microcontroller**: ESP32 Development Board (`esp32dev`), Arduino-ESP32 core 2.x
- **Motor Driver**: VNH5019
  - INA: GPIO 15
  - INB: GPIO 2
  - PWM: GPIO 21 (5 kHz, 8-bit, limited to 200)
  - CS (Current Sense): GPIO 32 (ADC1_CH4, ~140 mV/A; not used by the controller)
- **Gearmotor**: 150:1 gearbox (`GEAR_RATIO`)
- **Quadrature Encoder**:
  - Phase A: GPIO 14
  - Phase B: GPIO 27
  - 64 counts per motor revolution, full quadrature (`ENCODER_COUNTS_MOTOR_REV`), so 0.0375 degrees per count at the output shaft
  - Counts up for positive PWM (`ENCODER_DIRECTION = 1`)

## Controller
The control law, in volts, is

```
u = Kp*e + Ki*∫e dt - Kd*v_f
```

where `e = reference_deg - angle_deg` and `v_f` is the filtered shaft velocity. The output is converted to PWM counts with 255 counts = 3.3 V (`u_raw = u * 255 / 3.3`), then clamped to ±200 to give `pwm`.

| Gain | Value | Units |
|------|-------|-------|
| `Kp` | 0.8 | V/deg |
| `Ki` | 3.6 | V/(deg·s) |
| `Kd` | changed for each experiment | V·s/deg |

- **Derivative on measurement**: the D term uses the measured velocity rather than the error, so the reference step does not cause a derivative kick.
- **Velocity filter**: the backward-difference velocity is low-pass filtered as `v_f = ALPHA*v + (1 - ALPHA)*v_f`. `ALPHA = 0.20` is a time constant of about 22 ms (about 7 Hz cutoff) at 200 Hz. A smaller `ALPHA` filters more but adds lag.
- **Anti-windup**: while the output is saturated, the error is only integrated if it pulls the output back out of saturation. The integral is also clamped to ±300 deg·s (`INTEGRAL_LIMIT`).

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
2. Press `s` to begin a trial. The current shaft position becomes 0 degrees. The encoder is logged with the motor off for `PRE_STEP_US` (0.5 s). Then at `time_s = 0` the reference steps to `REFERENCE_DEG` (45 degrees) and the controller turns on. Baseline rows have negative time.
3. The serial monitor streams `time_s,reference_deg,angle_deg,error_deg,velocity_dps,u_raw,pwm` at 200 Hz, where `velocity_dps` is the filtered velocity used by the D term. Every line is also saved to `logs/device-monitor-*.log` (the `log2file` monitor filter), because a full run is longer than the VS Code terminal scrollback.
4. The motor stops automatically `TRIAL_DURATION_S` (3 s) after the step, or press `x` to stop it early (`x` before the step aborts the trial). Logging continues with the motor off for `POST_STOP_US` (3 s, `pwm` = 0) so the shaft coming to rest is captured, then `END` is printed.
5. Change `Kd` in `src/main.cpp` for each experiment and keep the Week 8 `Kp` and `Ki`.

The stream contains only the CSV header, data rows, and these cue lines:

| Line | Meaning |
|------|---------|
| `READY` | Board booted (printed once) |
| `MOTOR_ON` | The next row is the first with the motor on (`time_s = 0`) |
| `MOTOR_OFF` | The next row is the first with the motor off |
| `# missed a sample deadline, trial aborted` | The gap between two control samples was more than two sample periods (10 ms), so the motor was stopped and the trial ended early |
| `END; dropped_samples=N` | Logging stopped. `N` rows were lost because the serial link could not keep up (should be 0) |

### Firmware structure
- `controlTask` (core 1, priority 3) runs on a fixed 5 ms grid. Each sample reads the encoder, updates the filtered velocity, computes the PID output, drives the motor, and passes the row to the logger through a queue. It never prints.
- `loggerTask` (core 0, priority 1) handles the `s` / `x` commands and prints the queued rows and cue lines.
- The queue holds 512 rows (2.56 s). If it fills, the newest rows are dropped and counted in `dropped_samples`.

### Logging to CSV with MATLAB
1. Close the PlatformIO serial monitor (only one program can have the port open).
2. Run `log_trial.m`. It finds the ESP32 port, starts a trial, and records until `END`, using the `MOTOR_ON` / `MOTOR_OFF` cues to find when the motor turned on and off.
3. It saves `data/trial_<date>_<time>.csv`, trimmed from `PRE_S` (0.25 s) before the motor turns on to `POST_S` (6 s) after the shaft has come to rest with the motor off. `POST_S` is longer than the firmware's 3 s of post-stop logging, so in practice the file runs to `END`. Set `NAME_TAG` (e.g. `"kd=0.2"`) to label the file. The trimmed data is also left in the workspace as the table `trial`.

### Data
| Files | Experiment |
|-------|------------|
| `data/ki=*.csv` | Week 8 Ki sweep: `Kp` = 0.8, `Kd` = 0, 90 degree step (no `velocity_dps` column) |
| `data/kd=*.csv` | Week 9 Kd sweep: `Kp` = 0.8, `Ki` = 3.6, 45 degree step |
| `data/trial_*.csv` | Earlier P and PI trials |
