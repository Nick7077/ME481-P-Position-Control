#include <Arduino.h>
#include <ESP32Encoder.h>

// ME 481 Week 9: dual-core PID, Arduino-ESP32 core 2.x
// Core 1: 200 Hz control task. Core 0: CSV logging and serial commands.
// Change only Kd between trials; retain the Week 8 Kp and Ki.

ESP32Encoder encoder;

// ============================================================
// Pin assignment (ME 481 course setup)
// ============================================================
constexpr int MOTOR_INA = 15;
constexpr int MOTOR_INB = 2;
constexpr int MOTOR_PWM = 21;
constexpr int MOTOR_CS  = 32;   // VNH5019 current-sense output -> ADC1_CH4 (unused by the controller)
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
constexpr float GEAR_RATIO = 150.0f;

// Example for lower-ratio motor:
// constexpr float GEAR_RATIO = 4.4f;

// Full-quadrature encoder counts per motor revolution.
// Keep this consistent with your Week 4 convention.
constexpr float ENCODER_COUNTS_MOTOR_REV = 64.0f;

// On this rig (102.08:1 motor) the encoder counts UP for
// positive PWM, so no flip is needed. Set this to -1 for a
// motor that counts DOWN for positive PWM (the Week 4 motor:
// +232 PWM -> -16 counts); otherwise the loop is positive
// feedback and drives the shaft away from the reference.
constexpr int ENCODER_DIRECTION = 1;

// Output-shaft degrees per encoder count (direction included)
constexpr float DEG_PER_COUNT =
  ENCODER_DIRECTION * 360.0f / (ENCODER_COUNTS_MOTOR_REV * GEAR_RATIO);

// ============================================================
// Controller
// ============================================================
//   u = Kp*e + Ki*integral(e) - Kd*filtered d(angle)/dt
// in volts, converted to PWM counts with 255 counts = 3.3 V
// (see controlTask()).

// Units: V/deg
constexpr float Kp = 0.8f; //kp=135 oscillation

// Units: V/(deg*s), applied to the integral of the error
constexpr float Ki = 3.6f;

// CHANGE THIS FOR EACH EXPERIMENT
// Units: V/(deg/s) = V*s/deg, applied to the filtered measured
// velocity (derivative on measurement, so no derivative kick)
constexpr float Kd = 0.2f;

// Controller output (V) -> PWM counts
constexpr float PWM_PER_VOLT = 255.0f / 3.3f;

// Velocity filter: first-order low-pass (exponential moving
// average) on the backward-difference velocity,
//   vf = ALPHA*v + (1 - ALPHA)*vf
// At 200 Hz, ALPHA = 0.20 is a time constant of about 22 ms
// (about 7 Hz cutoff). Smaller ALPHA = more filtering, more lag.
constexpr float ALPHA = 0.20f;

// Clamp on the error integral (deg*s)
constexpr float INTEGRAL_LIMIT = 300.0f;

// TARGET ANGLE (deg) -- measured from wherever the output shaft
// is sitting when the trial starts (encoder is zeroed at start)
constexpr float REFERENCE_DEG = 45.0f;

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
//   END; dropped_samples=N   logging stopped; N rows were lost because
//                            the serial link could not keep up
//
// If the control task misses a sample deadline it stops the
// motor and the trial ends early with
//   # missed a sample deadline, trial aborted
// just before END.
//
// Serial commands (single key, Enter not needed):
//   s = start, x = stop

// ============================================================
// State shared between the two cores
// ============================================================
enum class Phase : uint8_t { PreStep, Control, PostStop };

// One CSV row, passed from the control task to the logger
struct Sample
{
  long tUs;            // time from the step (negative = before it)
  float referenceDeg;
  float angleDeg;
  float errorDeg;
  float velocityDps;   // filtered velocity used by the D term
  float uRaw;          // controller output before the PWM clamp
  int pwm;
  Phase phase;
};

QueueHandle_t logQueue;
volatile bool running = false;
volatile bool stopRequested = false;
volatile bool missedDeadline = false;
volatile uint32_t droppedSamples = 0;

// ============================================================
// Motor command
// ============================================================
void setMotor(int pwm)
{
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  // pwm = 0 -> INA = INB = LOW (brake)
  digitalWrite(MOTOR_INA, pwm > 0);
  digitalWrite(MOTOR_INB, pwm < 0);
  ledcWrite(PWM_CHANNEL, abs(pwm));
}

// ============================================================
// Control task (core 1, 200 Hz). It never prints: every row
// goes to the logger through logQueue.
// ============================================================
void controlTask(void*)
{
  for (;;)
  {
    if (!running)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Current position becomes 0 degrees
    encoder.clearCount();

    Phase phase = Phase::PreStep;
    unsigned long postStopSamplesLeft = 0;
    float integral = 0.0f, filteredVelocity = 0.0f;
    float previousAngle = DEG_PER_COUNT * (float)encoder.getCount();

    // The first sample is taken immediately; previous is
    // back-dated one period so its dt is a full period
    uint32_t start = micros(), next = start, previous = start - SAMPLE_PERIOD_US;

    for (;;)
    {
      uint32_t now = micros();
      if ((int32_t)(now - next) < 0)
      {
        taskYIELD();
        continue;
      }

      float dt = (uint32_t)(now - previous) * 1e-6f;
      previous = now;

      // Fixed grid (no drift): sample k is at k * SAMPLE_PERIOD_US
      next += SAMPLE_PERIOD_US;

      // A missed deadline invalidates a fixed-rate experiment
      if (dt <= 0.0f || dt > 2.0f * SAMPLE_PERIOD_S)
      {
        missedDeadline = true;
        break;
      }

      // Time relative to the step (negative = before the motor turns on)
      long tUs = (long)(now - start) - (long)PRE_STEP_US;

      // ------------------------------------------------------
      // Step on / stop ('x' or TRIAL_DURATION_S after the step)
      // ------------------------------------------------------
      if (phase == Phase::PreStep && stopRequested)
      {
        break;   // motor never turned on, nothing to log
      }
      if (phase == Phase::PreStep && tUs >= 0)
      {
        phase = Phase::Control;
      }
      else if (phase == Phase::Control && (stopRequested || tUs >= TRIAL_DURATION_US))
      {
        phase = Phase::PostStop;
        postStopSamplesLeft = POST_STOP_US / SAMPLE_PERIOD_US;
      }

      // ------------------------------------------------------
      // Read encoder (before the motor command below, so the
      // t = 0 row is the angle at the instant the motor turns
      // on) and update the filtered velocity
      // ------------------------------------------------------
      float angle = DEG_PER_COUNT * (float)encoder.getCount();
      float velocity = (angle - previousAngle) / dt;
      filteredVelocity = ALPHA * velocity + (1.0f - ALPHA) * filteredVelocity;
      previousAngle = angle;

      // Step input: reference is 0 until the motor turns on
      float reference = (phase == Phase::PreStep) ? 0.0f : REFERENCE_DEG;
      float error = reference - angle;

      // ------------------------------------------------------
      // PID in volts, scaled to PWM counts (255 = 3.3 V);
      // output forced to 0 while the motor is off
      // ------------------------------------------------------
      float uRaw = 0.0f;
      if (phase == Phase::Control)
      {
        float p = Kp * error, d = -Kd * filteredVelocity;
        float raw = (p + Ki * integral + d) * PWM_PER_VOLT;

        // Anti-windup (conditional integration): while the output
        // is saturated, only integrate an error that pulls it back
        bool high = raw > PWM_MAX, low = raw < -PWM_MAX;
        bool integrate = (!high && !low) || (high && error < 0) || (low && error > 0);
        if (Ki != 0.0f && integrate)
        {
          integral = constrain(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }

        uRaw = (p + Ki * integral + d) * PWM_PER_VOLT;
      }
      int pwm = constrain((int)lroundf(uRaw), -PWM_MAX, PWM_MAX);
      setMotor(pwm);

      Sample sample = {tUs, reference, angle, error, filteredVelocity, uRaw, pwm, phase};
      if (xQueueSend(logQueue, &sample, 0) != pdTRUE)
      {
        droppedSamples++;
      }

      if (phase == Phase::PostStop && --postStopSamplesLeft == 0)
      {
        break;
      }
    }

    setMotor(0);
    stopRequested = false;
    running = false;
  }
}

// ============================================================
// Logger task (core 0): serial commands and CSV output
// ============================================================
void loggerTask(void*)
{
  bool wasRunning = false;
  Phase lastPhase = Phase::PreStep;

  for (;;)
  {
    // --------------------------------------------------------
    // Serial commands
    // --------------------------------------------------------
    if (Serial.available())
    {
      char c = tolower(Serial.read());

      // !wasRunning: don't start until the last trial's END is out
      if (c == 's' && !running && !wasRunning)
      {
        xQueueReset(logQueue);
        droppedSamples = 0;
        missedDeadline = false;
        stopRequested = false;
        lastPhase = Phase::PreStep;
        Serial.println("time_s,reference_deg,angle_deg,error_deg,velocity_dps,u_raw,pwm");
        running = true;
      }
      else if (c == 'x')
      {
        stopRequested = true;
      }
    }

    // Read running before draining the queue: the control task
    // queues its last row before clearing running, so END can't
    // be printed ahead of any rows
    bool isRunning = running;

    // --------------------------------------------------------
    // CSV rows
    // --------------------------------------------------------
    Sample sample;
    while (xQueueReceive(logQueue, &sample, 0) == pdTRUE)
    {
      // Motor on/off cues, just before the first row they apply to
      if (sample.phase != lastPhase)
      {
        Serial.println(sample.phase == Phase::Control ? "MOTOR_ON" : "MOTOR_OFF");
        lastPhase = sample.phase;
      }

      // Single write per row; time_s is formatted from integer
      // microseconds so it is exact
      unsigned long absUs = (sample.tUs < 0) ? -sample.tUs : sample.tUs;
      Serial.printf("%s%lu.%06lu,%.3f,%.3f,%.3f,%.2f,%.3f,%d\r\n",
                    (sample.tUs < 0) ? "-" : "", absUs / 1000000, absUs % 1000000,
                    sample.referenceDeg, sample.angleDeg, sample.errorDeg,
                    sample.velocityDps, sample.uRaw, sample.pwm);
    }

    // --------------------------------------------------------
    // END once the control task has finished and every row
    // has been printed
    // --------------------------------------------------------
    if (wasRunning && !isRunning)
    {
      if (missedDeadline)
      {
        Serial.println("# missed a sample deadline, trial aborted");
      }
      Serial.printf("END; dropped_samples=%lu\r\n", (unsigned long)droppedSamples);
    }
    wasRunning = isRunning;

    vTaskDelay(pdMS_TO_TICKS(1));
  }
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

  // 512 rows = 2.56 s of buffering if the serial link falls behind
  logQueue = xQueueCreate(512, sizeof(Sample));
  if (!logQueue)
  {
    Serial.println("Queue allocation failed");
    return;
  }

  Serial.println("READY");

  xTaskCreatePinnedToCore(controlTask, "PID_200Hz", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(loggerTask, "CSV_Logger", 4096, nullptr, 1, nullptr, 0);
}

// ============================================================
// Main loop: all work is done in controlTask and loggerTask
// ============================================================
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
