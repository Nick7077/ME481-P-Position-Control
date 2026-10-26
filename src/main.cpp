#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder;

// ============================================================
// Pin assignment (ME 481 course setup)
// ============================================================
constexpr int MOTOR_INA = 15;
constexpr int MOTOR_INB = 2;
constexpr int MOTOR_PWM = 21;
constexpr int MOTOR_CS  = 32;   // VNH5019 current-sense output -> ADC1_CH4 (unused by P control)
constexpr int ENCODER_A = 14;
constexpr int ENCODER_B = 27;

// ============================================================
// PWM configuration
// ============================================================
constexpr int PWM_CHANNEL    = 0;
constexpr int PWM_FREQ       = 5000;
constexpr int PWM_RESOLUTION = 8;
constexpr int PWM_MAX        = 200;

// ============================================================
// Encoder / gearbox configuration
// ============================================================
// IMPORTANT:
// Change this if you install a different gearbox.
constexpr float GEAR_RATIO = 102.08f;

// Example for lower-ratio motor:
// constexpr float GEAR_RATIO = 4.4f;

// Full-quadrature encoder counts per motor revolution.
// Keep this consistent with your Week 4 convention.
constexpr float ENCODER_COUNTS_MOTOR_REV = 64.0f;

// On this rig (102.08:1 motor) the encoder counts UP for
// positive PWM, so no flip is needed. Set this to -1 for a
// motor that counts DOWN for positive PWM (the Week 4 motor:
// +232 PWM -> -16 counts); otherwise the P loop is positive
// feedback and drives the shaft away from the reference.
constexpr int ENCODER_DIRECTION = 1;

// Output-shaft degrees per encoder count (direction included)
constexpr float DEG_PER_COUNT =
  ENCODER_DIRECTION * 360.0f / (ENCODER_COUNTS_MOTOR_REV * GEAR_RATIO);

// ============================================================
// Controller
// ============================================================
// CHANGE THIS FOR EACH EXPERIMENT
// Units: V/deg. The output is converted to PWM counts with
// 255 counts = 3.3 V (see the P controller in loop()).
constexpr float Kp = 0.8f; //kp=135 oscillation

// Units: V/(deg*s), applied to the integral of the error
constexpr float Ki = 6.0f;
constexpr float Kd = 0.0f;   // not used yet

// Controller output (V) -> PWM counts
constexpr float PWM_PER_VOLT = 255.0f / 3.3f;

// TARGET ANGLE (deg) -- measured from wherever the output shaft
// is sitting when the trial starts (encoder is zeroed at start)
constexpr float REFERENCE_DEG = 90.0f;

// ============================================================
// Sampling
// ============================================================
// 5000 us = 0.005 s = 200 Hz
constexpr unsigned long SAMPLE_PERIOD_US = 5000;
constexpr float SAMPLE_PERIOD_S = SAMPLE_PERIOD_US / 1000000.0f;

// Baseline logged with the motor OFF before the step.
// The step (motor on) happens at time_s = 0, so baseline
// rows have negative time.
// 500000 us = 0.5 s = 100 samples
constexpr unsigned long PRE_STEP_US = 500000;

// Increase if more time is required to settle
// (measured from the step)
constexpr float TRIAL_DURATION_S = 3.0f;
constexpr long TRIAL_DURATION_US = (long)(TRIAL_DURATION_S * 1000000.0);

// Keep logging with the motor OFF after the motor stops
// (auto stop or 'x') so the shaft coming to rest is captured.
// END is printed once this has elapsed.
// 3000000 us = 3 s = 600 samples
constexpr unsigned long POST_STOP_US = 3000000;

static_assert(PRE_STEP_US % SAMPLE_PERIOD_US == 0,
              "PRE_STEP_US must be a multiple of SAMPLE_PERIOD_US so the step lands on a sample");
static_assert(POST_STOP_US >= SAMPLE_PERIOD_US,
              "POST_STOP_US must cover at least one sample");

// ============================================================
// Serial stream (read by log_trial.m)
// ============================================================
//   READY                    once at boot
//   time_s,...,pwm           CSV header, trial started
//   <rows>                   motor off, time_s < 0
//   MOTOR_ON                 next row is the first with the motor on (time_s = 0)
//   <rows>
//   MOTOR_OFF                next row is the first with the motor off
//   <rows>                   motor off for POST_STOP_US
//   END                      logging stopped
//
// Serial commands (single key, Enter not needed):
//   s = start, x = stop

// ============================================================
// Experiment state
// ============================================================
enum class Phase { Idle, PreStep, Control, PostStop };

Phase phase = Phase::Idle;
unsigned long trialStartUs = 0;
unsigned long lastSampleUs = 0;
unsigned long postStopSamplesLeft = 0;
float errorIntegral = 0.0f;   // deg*s, reset at the start of each trial

// ============================================================
// Motor command (returns the PWM actually applied)
// ============================================================
int setMotor(int pwm)
{
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  // pwm = 0 -> INA = INB = LOW (brake)
  digitalWrite(MOTOR_INA, pwm > 0);
  digitalWrite(MOTOR_INB, pwm < 0);
  ledcWrite(PWM_CHANNEL, abs(pwm));

  return pwm;
}

// ============================================================
// Trial control
// ============================================================
void startTrial()
{
  // Current position becomes 0 degrees
  encoder.clearCount();
  errorIntegral = 0.0f;

  trialStartUs = micros();

  // Back-date by one period so the first sample is taken
  // immediately, while the motor is still off
  lastSampleUs = trialStartUs - SAMPLE_PERIOD_US;

  phase = Phase::PreStep;
  Serial.println("time_s,reference_deg,angle_deg,error_deg,u_raw,pwm");
}

// Motor off, logging continues for POST_STOP_US
void stopMotor()
{
  setMotor(0);
  phase = Phase::PostStop;
  postStopSamplesLeft = POST_STOP_US / SAMPLE_PERIOD_US;
  Serial.println("MOTOR_OFF");
}

// Motor off, logging stops
void endTrial()
{
  setMotor(0);
  phase = Phase::Idle;
  Serial.println("END");
}

// ============================================================
// Setup
// ============================================================
void setup()
{
  Serial.begin(115200);

  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM, PWM_CHANNEL);
  setMotor(0);

  // Hardware full-quadrature counter, same as the Week 4 code
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENCODER_A, ENCODER_B);

  // Let the USB-serial link settle after reset, then throw away
  // any junk bytes that arrived during boot so they can't be
  // mistaken for a command
  delay(500);
  while (Serial.available())
  {
    Serial.read();
  }

  Serial.println("READY");
}

// ============================================================
// Main loop
// ============================================================
void loop()
{
  // --------------------------------------------------------
  // Serial commands
  // --------------------------------------------------------
  while (Serial.available())
  {
    switch (tolower(Serial.read()))
    {
      case 's':
        if (phase == Phase::Idle)
        {
          startTrial();
        }
        break;

      case 'x':
        if (phase == Phase::PreStep)
        {
          endTrial();   // motor never turned on, nothing to log
        }
        else if (phase == Phase::Control)
        {
          stopMotor();
        }
        break;
    }
  }

  if (phase == Phase::Idle)
  {
    return;
  }

  // --------------------------------------------------------
  // Fixed-rate sampling
  // --------------------------------------------------------
  unsigned long nowUs = micros();

  if (nowUs - lastSampleUs < SAMPLE_PERIOD_US)
  {
    return;
  }

  // Fixed grid (no drift): sample k is at k * SAMPLE_PERIOD_US
  lastSampleUs += SAMPLE_PERIOD_US;

  // Time relative to the step (negative = before the motor turns on)
  long tUs = (long)(nowUs - trialStartUs - PRE_STEP_US);

  // --------------------------------------------------------
  // Step on / auto stop, cued just before the first row
  // they apply to
  // --------------------------------------------------------
  if (phase == Phase::PreStep && tUs >= 0)
  {
    phase = Phase::Control;
    Serial.println("MOTOR_ON");
  }
  else if (phase == Phase::Control && tUs >= TRIAL_DURATION_US)
  {
    stopMotor();
  }

  // --------------------------------------------------------
  // Read encoder (before the motor command below, so the
  // t = 0 row is the angle at the instant the motor turns on)
  // --------------------------------------------------------
  float angleDeg = DEG_PER_COUNT * (float)encoder.getCount();

  // Step input: reference is 0 until the motor turns on
  float referenceDeg = (phase == Phase::PreStep) ? 0.0f : REFERENCE_DEG;
  float errorDeg = referenceDeg - angleDeg;

  // PI controller in volts, scaled to PWM counts (255 = 3.3 V);
  // output forced to 0 while the motor is off
  float uRaw = 0.0f;
  if (phase == Phase::Control)
  {
    // Anti-windup: don't integrate while the output is saturated
    // and the error would drive it further into saturation
    float uPrev = (Kp * errorDeg + Ki * errorIntegral) * PWM_PER_VOLT;
    bool saturated = fabsf(uPrev) >= PWM_MAX;
    if (!saturated || (uPrev > 0.0f) != (errorDeg > 0.0f))
    {
      errorIntegral += errorDeg * SAMPLE_PERIOD_S;
    }

    uRaw = (Kp * errorDeg + Ki * errorIntegral) * PWM_PER_VOLT;
  }
  int pwm = setMotor(lroundf(uRaw));

  // --------------------------------------------------------
  // CSV row in a single write; time_s is formatted from
  // integer microseconds so it is exact
  // --------------------------------------------------------
  unsigned long absUs = (tUs < 0) ? -tUs : tUs;
  Serial.printf("%s%lu.%06lu,%.3f,%.3f,%.3f,%.3f,%d\r\n",
                (tUs < 0) ? "-" : "", absUs / 1000000, absUs % 1000000,
                referenceDeg, angleDeg, errorDeg, uRaw, pwm);

  // --------------------------------------------------------
  // END after the post-stop rows
  // --------------------------------------------------------
  if (phase == Phase::PostStop && --postStopSamplesLeft == 0)
  {
    endTrial();
  }
}
