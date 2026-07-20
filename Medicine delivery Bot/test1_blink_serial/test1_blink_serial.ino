/*
  TEST 1 - Board & upload sanity check
  No wiring needed for this one except USB. If this doesn't upload or
  print anything, the problem is in the board/driver/IDE setup, not in
  your RC car wiring - fix this first before testing anything else.
*/

void setup() {
  Serial.begin(115200);
  delay(1000);  // give the Serial Monitor time to connect
  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32-S3 is alive and running your code!");
  Serial.println("========================================");
}

int counter = 0;

void loop() {
  Serial.print("Heartbeat #");
  Serial.println(counter);
  counter++;
  delay(1000);
}
