/*
  RC car control - ESP32-S3-WROOM-1 (N16R8) + Arduino IDE
  Control scheme (FlySky FS-i6, Mode 2):
    - Left stick (vertical, CH3)  = SPEED CAP, 0-100%. Full down = no
      speed at all, full up = full speed available.
    - Right stick (vertical, CH2) = forward/backward direction.
    - Right stick (horizontal, CH1) = left/right steering.
    - Right stick centered (both axes) = car stops, regardless of
      where the left stick (speed cap) is set.
    - Right stick's output is scaled by the left stick's speed cap,
      so the left stick acts as a master speed limiter over everything
      the right stick does (including turning-in-place).

  Same wiring as before, plus one new channel:
    Left BTS7960:  RPWM=GPIO4  LPWM=GPIO5  EN(R_EN+L_EN tied together)=GPIO6
    Right BTS7960: RPWM=GPIO7  LPWM=GPIO8  EN(R_EN+L_EN tied together)=GPIO9
    RC receiver:   CH3 (speed cap)      -> 10k/20k divider -> GPIO16
                   CH1 (left/right)     -> 10k/20k divider -> GPIO17
                   CH2 (forward/back)   -> 10k/20k divider -> GPIO18   <-- new

  Receiver's "Servo" port (top header, i-BUS/S.BUS output) is NOT used -
  only the individual CH1, CH2, CH3 channel pins are wired in.

  Board settings in Arduino IDE (Tools menu):
    Board:        "ESP32S3 Dev Module"
    USB CDC On Boot: Disabled (upload/monitor via UART port)
    Flash Size:   16MB
    PSRAM:        "OPI PSRAM"   <-- required for N16R8, or GPIO35-37 break
    Upload port:  the UART Type-C port

  NOTE: if forward/backward or left/right feel reversed once you test
  this, that's a transmitter-side fix, not a code fix - use the FS-i6's
  channel REVERSE setting for CH1/CH2 in the transmitter menu rather
  than editing this code.
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

const int SPEED_CAP_PIN = 16;  // CH3, left stick vertical
const int STEERING_PIN  = 17;  // CH1, right stick horizontal
const int FWDBACK_PIN   = 18;  // CH2, right stick vertical

// ---------- configuration ----------

const int RC_MIN = 1000, RC_MID = 1500, RC_MAX = 2000;  // typical FlySky pulse range (us)
const int DEADBAND_US = 30;                              // ignore small stick jitter
const unsigned long FAILSAFE_TIMEOUT_US = 30000;         // 30ms - no pulse = signal lost
const int PWM_FREQ = 20000;                               // 20kHz - inaudible
const int PWM_RES  = 8;                                   // ledcWrite range 0-255
const int PWM_MAX_DUTY = 255;                              // matches PWM_RES above
const unsigned long LOOP_DELAY_MS = 20;                   // ~50Hz control loop

bool signalOk = false;

// ---------- setup ----------

void setup() {
  Serial.begin(115200);

  pinMode(SPEED_CAP_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);
  pinMode(FWDBACK_PIN, INPUT);

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

// For the left stick (throttle-style, non-centering): 0-100%, linear.
float speedCapPercent(long us) {
  us = constrain(us, RC_MIN, RC_MAX);
  return (us - RC_MIN) / (float)(RC_MAX - RC_MIN) * 100.0;
}

// For the right stick (centering, spring-back): -100..100, with deadband.
float centeredPercent(long us) {
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

// ---------- direction functions ----------
// Each one takes a speed 0-100 and drives the car in that direction.
// Only one of these is called per loop, based on whichever axis the
// right stick is deflecting more - this mirrors the case-based style
// of forward()/backward()/left()/right() from the original sketch.

void moveForward(float speed) {
  setMotor(L_RPWM, L_LPWM, -speed);
  setMotor(R_RPWM, R_LPWM, speed);
}

void moveBackward(float speed) {
  setMotor(L_RPWM, L_LPWM, speed);
  setMotor(R_RPWM, R_LPWM, -speed);
}

void turnLeft(float speed) {
  // pivot: left side reverses, right side goes forward
  setMotor(L_RPWM, L_LPWM, speed);
  setMotor(R_RPWM, R_LPWM, speed);
}

void turnRight(float speed) {
  // pivot: right side reverses, left side goes forward
  setMotor(L_RPWM, L_LPWM, -speed);
  setMotor(R_RPWM, R_LPWM, -speed);
}

void stopCar() {
  stopMotors();
}

// ---------- main loop ----------

void loop() {
  long speedCap_us = readChannel(SPEED_CAP_PIN);
  long steering_us = readChannel(STEERING_PIN);
  long fwdback_us  = readChannel(FWDBACK_PIN);

  if (speedCap_us < 0 || steering_us < 0 || fwdback_us < 0) {
    // Signal lost - failsafe: cut motor power immediately
    if (signalOk) {
      Serial.println("RC signal lost - stopping motors");
    }
    stopCar();
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

    float speedCap = speedCapPercent(speedCap_us);   // 0..100
    float moveY = centeredPercent(fwdback_us);        // -100..100, forward/back
    float moveX = centeredPercent(steering_us);       // -100..100, left/right

    float scale = speedCap / 100.0;   // 0..1, master speed limiter
    float fb   = scale * moveY;       // effective forward/backward
    float turn = scale * moveX;       // effective left/right

    // Whichever axis is deflected more wins - keeps behavior as
    // clean forward/backward/left/right cases like the original sketch.
    if (fabs(fb) < 1 && fabs(turn) < 1) {
      stopCar();
    } else if (fabs(fb) >= fabs(turn)) {
      if (fb > 0) {
        moveForward(fb);
      } else {
        moveBackward(-fb);
      }
    } else {
      if (turn > 0) {
        turnRight(turn);
      } else {
        turnLeft(-turn);
      }
    }
  }

  delay(LOOP_DELAY_MS);
}

// ---------- calibration note ----------
// If your sticks don't reach full speed, or center isn't quite zero:
//   1. Open Serial Monitor (115200 baud), temporarily add
//      Serial.println(speedCap_us) / Serial.println(steering_us) /
//      Serial.println(fwdback_us) in the loop, move each stick to its
//      extremes and center, and note the values shown.
//   2. Update RC_MIN / RC_MID / RC_MAX above to match what you saw.
//
// If forward on the right stick makes the car go backward (or left/
// right feel swapped), reverse that channel in the FS-i6's transmitter
// menu (Reverse setting for CH1/CH2) - no code change needed.
