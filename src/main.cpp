#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder;

// ---------------- Pin assignment (ME 481 course setup) ----------------
constexpr int MOTOR_INA = 15;
constexpr int MOTOR_INB = 2;
constexpr int MOTOR_PWM = 17;
constexpr int MOTOR_CS  = 32;   // VNH5019 current-sense output -> ADC1_CH4
constexpr int ENCODER_A = 14;
constexpr int ENCODER_B = 27;

// ---------------- PWM configuration ----------------
constexpr int PWM_FREQ = 20000;
constexpr int PWM_BITS = 8;
constexpr int PWM_CHANNEL = 0;
constexpr int TEST_PWM = 150;

// ---------------- Current-sense configuration ----------------
// VNH5019 datasheet: CS output is ~140 mV per amp of motor current while driving.
// Calibrate this value experimentally if accurate current is needed.
constexpr float CS_MV_PER_AMP = 140.0f;
// Number of ADC samples averaged per reading. PWM at 20 kHz ripples the CS
// line, so a small average smooths the 100 Hz sample considerably.
constexpr int CS_SAMPLES = 8;
constexpr int CS_RAW_SAMPLES = 256;
// Voltage measured on CS with the motor off. Captured in setup() and
// subtracted so that an idle motor reports ~0 A.
float csOffset_mV = 0.0f;

// ---------------- Run control ----------------
constexpr unsigned long RUN_DURATION_MS = 100; // how long the motor runs once started
constexpr char START_CHAR = 's'; // send this over serial to start the motor

bool motorRunning = false;
unsigned long motorStartTime = 0;

// Diagnostic raw counts: no calibration, offset subtraction, or clamping.
// Averaging can expose small changes, but cannot recover an undetected signal.
float readCSRawAverage() {
  uint32_t sum = 0;
  for (int i = 0; i < CS_RAW_SAMPLES; i++) {
    sum += analogRead(MOTOR_CS);
  }
  return static_cast<float>(sum) / CS_RAW_SAMPLES;
}

// Read the CS pin and return the averaged voltage in millivolts.
// analogReadMilliVolts() applies the ESP32's factory ADC calibration,
// which is noticeably more accurate than raw counts * 3300 / 4095.
float readCS_mV() {
  uint32_t sum = 0;
  for (int i = 0; i < CS_SAMPLES; i++) {
    sum += analogReadMilliVolts(MOTOR_CS);
  }
  return static_cast<float>(sum) / CS_SAMPLES;
}

// Convert the CS voltage into motor current in amperes.
float readMotorCurrent_A() {
  float mv = readCS_mV() - csOffset_mV;
  if (mv < 0.0f) mv = 0.0f;
  return mv / CS_MV_PER_AMP;
}

void setup() {
  Serial.begin(115200);

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENCODER_A, ENCODER_B);
  encoder.clearCount();

  // Motor driver direction + PWM
  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcAttachPin(MOTOR_PWM, PWM_CHANNEL);
  digitalWrite(MOTOR_INA, HIGH);
  digitalWrite(MOTOR_INB, LOW);
  ledcWrite(PWM_CHANNEL, 0); // motor off until serial start command

  // Current sense ADC
  // 12-bit resolution (0-4095). 0 dB improves sensitivity, with a specified
  // range of roughly 100-950 mV on the original ESP32. The measured 15-40 mV
  // signal is below that range, so current_A and the boot offset may be unreliable.
  pinMode(MOTOR_CS, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MOTOR_CS, ADC_0db);

  // Capture the CS zero-current offset while the motor is off.
  delay(100);
  csOffset_mV = readCS_mV();

  Serial.print("CS_OFFSET_mV,");
  Serial.println(csOffset_mV, 1);
  Serial.println("Send 's' to start motor");
  Serial.println("time_ms,pwm_command,encoder_count,current_A");
  // Serial.println("time_ms,pwm_command,encoder_count,current_A,cs_raw_avg");
}

void loop() {
  // Start the motor on serial input
  if (!motorRunning && Serial.available() > 0) {
    char c = Serial.read();
    if (c == START_CHAR) {
      encoder.clearCount();
      motorStartTime = millis();
      motorRunning = true;
      ledcWrite(PWM_CHANNEL, TEST_PWM);
      Serial.print("MOTOR_ON,");
      Serial.println(motorStartTime);
    }
  }

  // Stop the motor after the set duration
  if (motorRunning && (millis() - motorStartTime >= RUN_DURATION_MS)) {
    ledcWrite(PWM_CHANNEL, 0);
    motorRunning = false;
    Serial.print("MOTOR_OFF,");
    Serial.println(millis());
  }

  // CSV log. Raw counts and current use separate ADC sample batches.
  // Sampling and serial output add to the 10 ms delay below.
  // float csRawAverage = readCSRawAverage();
  Serial.print(millis());
  Serial.print(",");
  Serial.print(motorRunning ? TEST_PWM : 0);
  Serial.print(",");
  Serial.print(encoder.getCount());
  Serial.print(",");
  Serial.println(readMotorCurrent_A(), 3);
  // Serial.print(",");
  // Serial.println(csRawAverage, 3);

  delay(1); // Pause between readings; actual logging rate is below 1 kHz.
}
