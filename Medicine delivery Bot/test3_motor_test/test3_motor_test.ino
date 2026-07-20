/*
  TEST 3 - Motor / BTS7960 check (no receiver needed)
  Wiring needed: just the two BTS7960 drivers and the 4 motors.
  This ignores the RC receiver completely and just runs each side
  forward and reverse automatically, so you can confirm the motor
  wiring is correct on its own.

  IMPORTANT: put the chassis on a stand so the wheels spin freely
  before running this - do NOT test with wheels on the ground.
*/

#include <math.h>

const int L_RPWM = 4;
const int L_LPWM = 5;
const int L_EN   = 6;

const int R_RPWM = 7;
const int R_LPWM = 8;
const int R_EN   = 9;

const int PWM_FREQ = 20000;
const int PWM_RES  = 8;     // 0-255
const int TEST_SPEED = 130; // ~50% speed, out of 255 - keep it gentle for testing

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);

  Serial.println("Motor test starting - wheels should be off the ground!");
}

void stopAll() {
  ledcWrite(L_RPWM, 0);
  ledcWrite(L_LPWM, 0);
  ledcWrite(R_RPWM, 0);
  ledcWrite(R_LPWM, 0);
}

void loop() {
  Serial.println("LEFT side - forward");
  ledcWrite(L_RPWM, TEST_SPEED);
  ledcWrite(L_LPWM, 0);
  delay(1500);
  stopAll();
  delay(800);

  Serial.println("LEFT side - reverse");
  ledcWrite(L_RPWM, 0);
  ledcWrite(L_LPWM, TEST_SPEED);
  delay(1500);
  stopAll();
  delay(800);

  Serial.println("RIGHT side - forward");
  ledcWrite(R_RPWM, TEST_SPEED);
  ledcWrite(R_LPWM, 0);
  delay(1500);
  stopAll();
  delay(800);

  Serial.println("RIGHT side - reverse");
  ledcWrite(R_RPWM, 0);
  ledcWrite(R_LPWM, TEST_SPEED);
  delay(1500);
  stopAll();
  delay(800);

  Serial.println("--- cycle complete, repeating ---");
  delay(2000);
}
