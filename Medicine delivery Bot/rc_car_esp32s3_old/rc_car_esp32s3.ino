/*
  RC car control - ESP32-S3-WROOM-1 (N16R8) + Arduino IDE
  Reads FlySky FS-iA10B PWM channels (throttle + steering) and drives
  two BTS7960 motor drivers (left side = 2 motors in parallel,
  right side = 2 motors in parallel).

  Same wiring as the MicroPython version:
    Left BTS7960:  RPWM=GPIO4  LPWM=GPIO5  EN(R_EN+L_EN tied together)=GPIO6
    Right BTS7960: RPWM=GPIO7  LPWM=GPIO8  EN(R_EN+L_EN tied together)=GPIO9
    RC receiver:   CH3 (throttle) -> 10k/20k divider -> GPIO16
                   CH1 (steering) -> 10k/20k divider -> GPIO17

  Receiver's "Servo" port (top header, i-BUS/S.BUS output) is NOT used -
  only the individual CH1 and CH3 channel pins are wired in.

  Board settings in Arduino IDE (Tools menu):
    Board:        "ESP32S3 Dev Module"
    USB CDC On Boot: Disabled (upload/monitor via UART port)
    Flash Size:   16MB
    PSRAM:        "OPI PSRAM"   <-- required for N16R8, or GPIO35-37 break
    Upload port:  the UART Type-C port
*/

// ---------- includes ----------

#include <math.h>  // for fabs()

// ---------- pin definitions ----------

const int L_RPWM = 4;
const int L_LPWM = 5;
const int L_EN   = 6;

const int R_RPWM = 7;
const int R_LPWM = 8;
const int R_EN   = 9;

const int THROTTLE_PIN = 16;  // CH3
const int STEERING_PIN = 17;  // CH1

// ---------- configuration ----------

const int RC_MIN = 1000, RC_MID = 1500, RC_MAX = 2000;  // typical FlySky pulse range (us)
const int DEADBAND_US = 30;                              // ignore small stick jitter
const unsigned long FAILSAFE_TIMEOUT_US = 30000;         // 30ms - no pulse = signal lost
const int PWM_FREQ = 20000;                               // 20kHz - inaudible
const int PWM_RES  = 8;                                   // ledcWrite range 0-255
                                                            // (20kHz needs an 80MHz/20kHz=4000x
                                                            // clock margin; 8-bit=256x fits safely,
                                                            // 16-bit would need a 1.3GHz clock and
                                                            // silently fails on real hardware)
const int PWM_MAX_DUTY = 255;                              // matches PWM_RES above
const unsigned long LOOP_DELAY_MS = 20;                   // ~50Hz control loop

bool signalOk = false;

// ---------- setup ----------

void setup() {
  Serial.begin(115200);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);

  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, LOW);   // start disabled until we get a valid RC signal
  digitalWrite(R_EN, LOW);

  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);

  Serial.println("RC car ready. Waiting for transmitter signal...");
}

// ---------- helpers ----------

// Reads one RC PWM pulse width in microseconds.
// Returns -1 if the receiver signal is missing/invalid (failsafe).
long readChannel(int pin) {
  long us = pulseIn(pin, HIGH, FAILSAFE_TIMEOUT_US);
  if (us <= 0) return -1;              // pulseIn returns 0 on timeout
  if (us < 800 || us > 2200) return -1; // sanity check, reject glitches
  return us;
}

float pulseToPercent(long us) {
  us = constrain(us, RC_MIN, RC_MAX);
  if (abs(us - RC_MID) < DEADBAND_US) return 0;
  if (us > RC_MID) {
    return (us - RC_MID) / (float)(RC_MAX - RC_MID) * 100.0;
  } else {
    return (us - RC_MID) / (float)(RC_MID - RC_MIN) * 100.0;
  }
}

void setMotor(int rpwmPin, int lpwmPin, float speed) {
  // speed: -100 (full reverse) .. +100 (full forward)
  speed = constrain(speed, -100, 100);
  int duty = (int)(fabs(speed) / 100.0 * PWM_MAX_DUTY);
  if (speed >= 0) {
    ledcWrite(rpwmPin, duty);
    ledcWrite(lpwmPin, 0);
  } else {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, duty);
  }
}

void stopMotors() {
  ledcWrite(L_RPWM, 0);
  ledcWrite(L_LPWM, 0);
  ledcWrite(R_RPWM, 0);
  ledcWrite(R_LPWM, 0);
}

// ---------- main loop ----------

void loop() {
  long throttle_us = readChannel(THROTTLE_PIN);
  long steering_us = readChannel(STEERING_PIN);

  if (throttle_us < 0 || steering_us < 0) {
    // Signal lost - failsafe: cut motor power immediately
    if (signalOk) {
      Serial.println("RC signal lost - stopping motors");
    }
    stopMotors();
    digitalWrite(L_EN, LOW);
    digitalWrite(R_EN, LOW);
    signalOk = false;
  } else {
    if (!signalOk) {
      Serial.println("RC signal acquired");
      digitalWrite(L_EN, HIGH);
      digitalWrite(R_EN, HIGH);
    }
    signalOk = true;

    float throttle = pulseToPercent(throttle_us);
    float steering = pulseToPercent(steering_us);

    // arcade-style mixing: steering pulls one side down, pushes the other up
    float leftSpeed  = throttle + steering;
    float rightSpeed = throttle - steering;

    setMotor(L_RPWM, L_LPWM, leftSpeed);
    setMotor(R_RPWM, R_LPWM, rightSpeed);
  }

  delay(LOOP_DELAY_MS);
}

// ---------- calibration note ----------
// If your sticks don't reach full speed, or center isn't quite zero:
//   1. Open Serial Monitor (115200 baud), temporarily add
//      Serial.println(throttle_us) / Serial.println(steering_us) in the
//      loop, move each stick to its extremes and center, and note the
//      values shown.
//   2. Update RC_MIN / RC_MID / RC_MAX above to match what you saw.
