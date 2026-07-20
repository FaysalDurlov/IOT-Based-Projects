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

#include <math.h>       // for fabs()
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <MFRC522.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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

// Sensor / display pins
const int I2C_SDA_PIN = 1;     // OLED + BMP180 share this bus
const int I2C_SCL_PIN = 2;
const int DHT_PIN     = 11;
const int RFID_SCK_PIN  = 39;
const int RFID_MISO_PIN = 41;
const int RFID_MOSI_PIN = 40;
const int RFID_SS_PIN   = 42;
const int RFID_RST_PIN  = 38;

// ---------- sensor / display objects ----------

DHT dht(DHT_PIN, DHT11);
Adafruit_BMP085 bmp;
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledOk = false;

// cached sensor readings - updated on a slow timer, not every loop
float lastTempC = NAN;
float lastHumidity = NAN;
float lastPressurePa = NAN;
String lastCardUID = "none yet";

unsigned long lastSensorReadMs = 0;
const unsigned long SENSOR_READ_INTERVAL_MS = 2000;   // DHT11 tops out ~1Hz anyway

unsigned long lastRfidCheckMs = 0;
const unsigned long RFID_CHECK_INTERVAL_MS = 200;

unsigned long lastDisplayUpdateMs = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 500;

// ---------- BLE voice control ----------
// UUIDs must match the web app exactly.
#define BLE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *bleCharacteristic;
bool bleDeviceConnected = false;

String voiceUiState = "idle";        // "idle" / "listening" / "thinking" / "speaking"
unsigned long lastVoiceStateMs = 0;
const unsigned long VOICE_STATE_DISPLAY_TIMEOUT_MS = 8000;  // fall back to sensor dashboard if phone goes quiet

String pendingVoiceCommand = "";
unsigned long voiceCommandUntilMs = 0;
const unsigned long VOICE_COMMAND_PULSE_MS = 1500;  // how long a spoken command drives for
const float VOICE_COMMAND_SPEED = 50.0;             // fixed moderate speed for voice-driven moves

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleDeviceConnected = true;
    Serial.println("BLE: phone connected");
  }
  void onDisconnect(BLEServer* server) override {
    bleDeviceConnected = false;
    Serial.println("BLE: phone disconnected");
    server->getAdvertising()->start();  // keep advertising so it can reconnect
  }
};

class BleCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = String(characteristic->getValue().c_str());
    Serial.print("BLE received: ");
    Serial.println(value);

    if (value.startsWith("STATE:")) {
      voiceUiState = value.substring(6);
      lastVoiceStateMs = millis();
    } else if (value.startsWith("CMD:")) {
      pendingVoiceCommand = value.substring(4);
      voiceCommandUntilMs = millis() + VOICE_COMMAND_PULSE_MS;
    }
  }
};

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

  // ---- sensors + display ----
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  dht.begin();

  if (!bmp.begin()) {
    Serial.println("BMP180 not found - check I2C wiring");
  }

  SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);
  mfrc522.PCD_Init();

  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) {
    Serial.println("OLED not found - check I2C wiring/address");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Starting up...");
    display.display();
  }

  Serial.println("RC car ready. Waiting for transmitter signal...");

  // ---- BLE voice control ----
  BLEDevice::init("BanglaVoiceCar");
  BLEServer *bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());

  BLEService *bleService = bleServer->createService(BLE_SERVICE_UUID);
  bleCharacteristic = bleService->createCharacteristic(
    BLE_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  bleCharacteristic->setCallbacks(new BleCommandCallbacks());
  bleCharacteristic->addDescriptor(new BLE2902());
  bleService->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->start();
  Serial.println("BLE: advertising as 'BanglaVoiceCar'");
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

// ---------- sensors + display ----------
// All of these run on their own slow timers via millis() - none of them
// use delay(), so they never block the 20ms RC control loop above.

void updateSensorsIfDue() {
  unsigned long now = millis();
  if (now - lastSensorReadMs < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadMs = now;

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    lastHumidity = h;
    lastTempC = t;
  }

  lastPressurePa = bmp.readPressure();  // Pa
}

void checkRfidIfDue() {
  unsigned long now = millis();
  if (now - lastRfidCheckMs < RFID_CHECK_INTERVAL_MS) return;
  lastRfidCheckMs = now;

  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  String uid = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(mfrc522.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  lastCardUID = uid;
  Serial.print("Card scanned: ");
  Serial.println(uid);

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void updateDisplayIfDue() {
  if (!oledOk) return;
  unsigned long now = millis();
  if (now - lastDisplayUpdateMs < DISPLAY_UPDATE_INTERVAL_MS) return;
  lastDisplayUpdateMs = now;

  display.clearDisplay();
  display.setCursor(0, 0);

  bool voiceStateRecent = (now - lastVoiceStateMs < VOICE_STATE_DISPLAY_TIMEOUT_MS) && voiceUiState != "idle";

  if (voiceStateRecent) {
    display.setTextSize(2);
    display.setCursor(10, 20);
    if (voiceUiState == "listening") {
      display.println("Listening");
    } else if (voiceUiState == "thinking") {
      display.println("Thinking");
    } else if (voiceUiState == "speaking") {
      display.println("Speaking");
    } else {
      display.println(voiceUiState);
    }
    display.setTextSize(1);
    display.display();
    return;
  }

  if (isnan(lastTempC)) {
    display.println("Temp: --.- C");
    display.println("Hum:  --.- %");
  } else {
    display.print("Temp: ");
    display.print(lastTempC, 1);
    display.println(" C");
    display.print("Hum:  ");
    display.print(lastHumidity, 1);
    display.println(" %");
  }

  display.print("Pres: ");
  display.print(lastPressurePa / 100.0, 1);  // Pa -> hPa
  display.println(" hPa");

  display.println();
  display.print("Card: ");
  display.println(lastCardUID);

  display.display();
}

// ---------- main loop ----------

void loop() {
  updateSensorsIfDue();
  checkRfidIfDue();
  updateDisplayIfDue();

  // Voice-driven command takes priority for a short pulse, then hands
  // control back to the RC transmitter automatically.
  if (millis() < voiceCommandUntilMs) {
    digitalWrite(L_EN, HIGH);
    digitalWrite(R_EN, HIGH);
    if (pendingVoiceCommand == "forward") {
      moveForward(VOICE_COMMAND_SPEED);
    } else if (pendingVoiceCommand == "backward") {
      moveBackward(VOICE_COMMAND_SPEED);
    } else if (pendingVoiceCommand == "left") {
      turnLeft(VOICE_COMMAND_SPEED);
    } else if (pendingVoiceCommand == "right") {
      turnRight(VOICE_COMMAND_SPEED);
    } else if (pendingVoiceCommand == "stop") {
      stopCar();
    }
    delay(LOOP_DELAY_MS);
    return;  // skip RC reading this cycle - voice command is driving
  }

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
