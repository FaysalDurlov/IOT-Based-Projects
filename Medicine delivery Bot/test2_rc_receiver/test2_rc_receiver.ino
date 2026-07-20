/*
  TEST 2 - RC receiver signal check
  Wiring needed: just the receiver's CH3 and CH1 signal wires through
  their voltage dividers into GPIO16 and GPIO17. No motors/BTS7960
  needed for this test.

  Power on your transmitter FIRST, then power the ESP32.
  Watch the Serial Monitor - move the throttle and steering sticks and
  confirm the numbers change smoothly between roughly 1000 and 2000.
*/

const int THROTTLE_PIN = 16;  // CH3
const int STEERING_PIN = 17;  // CH1
const unsigned long TIMEOUT_US = 30000;

void setup() {
  Serial.begin(115200);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);
  delay(1000);
  Serial.println("RC receiver test starting...");
  Serial.println("Move the sticks and watch the numbers below.");
}

void loop() {
  long throttle_us = pulseIn(THROTTLE_PIN, HIGH, TIMEOUT_US);
  long steering_us = pulseIn(STEERING_PIN, HIGH, TIMEOUT_US);

  Serial.print("Throttle (GPIO16): ");
  if (throttle_us <= 0) {
    Serial.print("NO SIGNAL");
  } else {
    Serial.print(throttle_us);
    Serial.print(" us");
  }

  Serial.print("   |   Steering (GPIO17): ");
  if (steering_us <= 0) {
    Serial.print("NO SIGNAL");
  } else {
    Serial.print(steering_us);
    Serial.print(" us");
  }

  Serial.println();
  delay(200);
}
