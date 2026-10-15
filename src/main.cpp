#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder;

// ============================================================
// Pin assignment (ME 481 course setup)
// ============================================================
constexpr int MOTOR_INA = 15;
constexpr int MOTOR_INB = 2;
constexpr int MOTOR_PWM = 17;
constexpr int MOTOR_CS  = 32;   // VNH5019 current-sense output -> ADC1_CH4 (unused by P control)
constexpr int ENCODER_A = 14;
constexpr int ENCODER_B = 27;

// ============================================================
// PWM configuration
// ============================================================
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 180;

// ============================================================
// Encoder / gearbox configuration
// ============================================================
// IMPORTANT:
// Change this if you install a different gearbox.
const float GEAR_RATIO = 20.4;

// Example for lower-ratio motor:
// const float GEAR_RATIO = 4.4;

// Full-quadrature encoder counts per motor revolution.
// Keep this consistent with your Week 4 convention.
const float ENCODER_COUNTS_MOTOR_REV = 48.0;

const float COUNTS_PER_OUTPUT_REV =
  ENCODER_COUNTS_MOTOR_REV * GEAR_RATIO;

// On this rig the encoder counts DOWN for positive PWM
// (Week 4: +232 PWM -> -16 counts). Flip it so positive PWM
// gives positive angle; otherwise the P loop is positive
// feedback and drives the shaft away from the reference.
const int ENCODER_DIRECTION = -1;

// ============================================================
// Controller
// ============================================================
// CHANGE THIS FOR EACH EXPERIMENT
float Kp = 10;

// Week 5: P control only
const float Ki = 0.0;
const float Kd = 0.0;

// TARGET ANGLE (deg) -- measured from wherever the output shaft
// is sitting when you press 's' (encoder is zeroed at start)
const float REFERENCE_DEG = 45.0;

// ============================================================
// Sampling
// ============================================================
// 5000 us = 0.005 s = 200 Hz
const unsigned long SAMPLE_PERIOD_US = 5000;

// Baseline logged with the motor OFF before the step.
// The step (motor on) happens at time_s = 0, so baseline
// rows have negative time. Keep a multiple of SAMPLE_PERIOD_US
// so the step lands exactly on a sample.
// 500000 us = 0.5 s = 100 samples
const unsigned long PRE_STEP_US = 500000;

// Increase if more time is required to settle
// (measured from the step)
const float TRIAL_DURATION_S = 15.0;

// Keep logging with the motor OFF after the trial ends
// (auto stop or 'x') so the shaft coming to rest is captured.
// END is printed once this has elapsed.
// 1000000 us = 1 s = 200 samples
const unsigned long POST_STOP_US = 1000000;

// ============================================================
// Experiment variables
// ============================================================
bool running = false;
unsigned long trialStartUs = 0;
unsigned long lastSampleUs = 0;

// true once the motor has been stopped and the post-stop
// rows are being logged
bool motorStopped = false;
unsigned long postStopSamplesLeft = 0;

// ============================================================
// Motor command
// ============================================================
void setMotor(int pwm)
{
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  if (pwm > 0)
  {
    digitalWrite(MOTOR_INA, HIGH);
    digitalWrite(MOTOR_INB, LOW);
    ledcWrite(PWM_CHANNEL, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(MOTOR_INA, LOW);
    digitalWrite(MOTOR_INB, HIGH);
    ledcWrite(PWM_CHANNEL, -pwm);
  }
  else
  {
    digitalWrite(MOTOR_INA, LOW);
    digitalWrite(MOTOR_INB, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}

// ============================================================
// End experiment (logging stops)
// ============================================================
void endTrial()
{
  setMotor(0);
  running = false;
  motorStopped = false;
  Serial.println("END");
}

// ============================================================
// Stop motor (logging continues for POST_STOP_US)
// ============================================================
void stopTrial()
{
  setMotor(0);
  motorStopped = true;
  postStopSamplesLeft = POST_STOP_US / SAMPLE_PERIOD_US;

  if (postStopSamplesLeft == 0)
  {
    endTrial();
  }
}

// ============================================================
// Setup
// ============================================================
void setup()
{
  Serial.begin(115200);

  // Let the USB-serial link settle after reset, then throw away
  // any junk bytes that arrived during boot so they can't be
  // mistaken for a command
  delay(500);
  while (Serial.available())
  {
    Serial.read();
  }
  Serial.println();

  // Hardware full-quadrature counter, same as the Week 4 code
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENCODER_A, ENCODER_B);
  encoder.clearCount();

  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);

  ledcSetup(
    PWM_CHANNEL,
    PWM_FREQ,
    PWM_RESOLUTION
  );

  ledcAttachPin(
    MOTOR_PWM,
    PWM_CHANNEL
  );

  setMotor(0);

  Serial.println("READY - press s to start, x to stop");
}

// ============================================================
// Main loop
// ============================================================
void loop()
{
  // --------------------------------------------------------
  // Serial commands (single key, Enter not needed)
  //   s = start, x = stop
  // --------------------------------------------------------
  while (Serial.available())
  {
    char c = Serial.read();

    if ((c == 's' || c == 'S')
        && !running)
    {
      // Current equilibrium becomes 0 degrees
      encoder.clearCount();

      trialStartUs = micros();

      // Back-date by one period so the first sample is taken
      // immediately, while the motor is still off
      lastSampleUs = trialStartUs - SAMPLE_PERIOD_US;

      running = true;

      Serial.println(
        "time_s,reference_deg,angle_deg,"
        "error_deg,u_raw,pwm"
      );
    }

    else if ((c == 'x' || c == 'X')
             && running && !motorStopped)
    {
      stopTrial();
    }
  }

  // --------------------------------------------------------
  // Motor off while waiting
  // --------------------------------------------------------
  if (!running)
  {
    setMotor(0);

    // Re-print the prompt so the monitor shows readable text
    // even if it connected after the boot banner went out
    static unsigned long lastPromptMs = 0;
    if (millis() - lastPromptMs >= 2000)
    {
      lastPromptMs = millis();
      Serial.println("READY - press s to start, x to stop");
    }
    return;
  }

  // --------------------------------------------------------
  // Fixed-rate sampling
  // --------------------------------------------------------
  unsigned long nowUs = micros();

  if ((unsigned long)(nowUs - lastSampleUs)
      >= SAMPLE_PERIOD_US)
  {
    // Fixed grid (no drift): sample k is at k * SAMPLE_PERIOD_US
    lastSampleUs += SAMPLE_PERIOD_US;

    unsigned long elapsedUs = nowUs - trialStartUs;

    // ----------------------------------------------------
    // Read encoder (before the motor command below, so the
    // t = 0 row is the angle at the instant the motor turns on)
    // ----------------------------------------------------
    int64_t count =
      ENCODER_DIRECTION * encoder.getCount();

    // ----------------------------------------------------
    // Convert encoder count to output angle
    // ----------------------------------------------------
    float angleDeg =
      360.0 *
      ((float)count /
      COUNTS_PER_OUTPUT_REV);

    // ----------------------------------------------------
    // Step input: reference is 0 and the motor is held off
    // until time_s = 0, then it steps to REFERENCE_DEG
    // ----------------------------------------------------
    bool stepped =
      elapsedUs >= PRE_STEP_US;

    float referenceDeg =
      stepped ? REFERENCE_DEG : 0.0;

    // ----------------------------------------------------
    // Position error
    // ----------------------------------------------------
    float errorDeg =
      referenceDeg - angleDeg;

    // ----------------------------------------------------
    // P controller (output forced to 0 before the step
    // and after the motor is stopped)
    // ----------------------------------------------------
    float uRaw =
      (stepped && !motorStopped) ? Kp * errorDeg : 0.0;

    int pwmCommand =
      constrain(
        (int)round(uRaw),
        -PWM_MAX,
        PWM_MAX
      );

    setMotor(pwmCommand);

    // ----------------------------------------------------
    // Time relative to the step (negative = motor still off)
    // ----------------------------------------------------
    float timeSec =
      (long)(elapsedUs - PRE_STEP_US)
      / 1000000.0;

    // ----------------------------------------------------
    // Output CSV data
    // ----------------------------------------------------
    Serial.print(timeSec, 6);
    Serial.print(",");
    Serial.print(referenceDeg, 3);
    Serial.print(",");
    Serial.print(angleDeg, 3);
    Serial.print(",");
    Serial.print(errorDeg, 3);
    Serial.print(",");
    Serial.print(uRaw, 3);
    Serial.print(",");
    Serial.println(pwmCommand);

    // ----------------------------------------------------
    // Automatic stop, then END after the post-stop rows
    // ----------------------------------------------------
    if (motorStopped)
    {
      if (--postStopSamplesLeft == 0)
      {
        endTrial();
      }
    }
    else if (timeSec >= TRIAL_DURATION_S)
    {
      stopTrial();
    }
  }
}
