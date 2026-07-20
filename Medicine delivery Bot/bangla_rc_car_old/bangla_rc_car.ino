/*
  Bangla Voice-Controlled RC Car — Follow Me + Lid Servo
  Board: ESP32-S3-WROOM-1 N16R8 (Arduino IDE)

  ARCHITECTURE:
  The ESP32 cannot understand speech itself. Your phone (using the
  companion "bangla_voice_control.html" page) listens in Bangla,
  turns speech into short command words, and sends them to this
  sketch over Bluetooth LE. This sketch drives the hardware and
  sends short Bangla replies back, which your phone speaks aloud.

  REQUIRED LIBRARIES (Arduino IDE > Tools > Manage Libraries):
   - ESP32Servo
   - Adafruit GFX Library
   - Adafruit SSD1306
   - Adafruit Unified Sensor
   - Adafruit BMP085 Library      (works for BMP180 / GY-68)
   - DHT sensor library (by Adafruit)
   - MFRC522                     (by GithubCommunity)
   (BLEDevice/BLEServer come built-in with the ESP32 board package)

  BOARD SETTINGS (Tools menu):
   - Board: "ESP32S3 Dev Module"
   - USB CDC On Boot: "Enabled"
   - Flash Size: "16MB"
   - PSRAM: "OPI PSRAM"
   - Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)" or similar

  CALIBRATE before first real use:
   - LID_OPEN_ANGLE / LID_CLOSED_ANGLE for your box
   - FOLLOW_FAR_CM / FOLLOW_NEAR_CM for your follow-me comfort distance
*/

#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <MFRC522.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- Pin map ----------------
#define I2C_SDA 1
#define I2C_SCL 2

#define L_RPWM 4
#define L_LPWM 5
#define L_EN 6

#define R_RPWM 7
#define R_LPWM 8
#define R_EN 9

#define DHT_PIN 11
#define DHT_TYPE DHT11

#define SERVO_PIN 21

#define TRIG_PIN 12
#define ECHO_PIN 13

#define RFID_SCK 39
#define RFID_MISO 41
#define RFID_MOSI 40
#define RFID_SS 42
#define RFID_RST 38

// ---------------- Objects ----------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_BMP085 bmp;
DHT dht(DHT_PIN, DHT_TYPE);
Servo lidServo;
MFRC522 rfid(RFID_SS, RFID_RST);

// ---------------- BLE (Nordic UART-style service) ----------------
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> car
#define CHAR_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // car -> phone

BLEServer *bleServer;
BLECharacteristic *txChar;
bool bleConnected = false;
volatile bool newCommand = false;
String incomingCmd = "";

// ---------------- State ----------------
enum Mode { IDLE,
            DRIVING,
            FOLLOWING };
Mode currentMode = IDLE;
unsigned long driveUntil = 0;

const int DRIVE_MS = 1000;    // each voice drive command runs this long
const int DRIVE_SPEED = 200;  // 0-255
const int TURN_SPEED = 180;

const float FOLLOW_FAR_CM = 40;   // if farther than this, drive forward
const float FOLLOW_NEAR_CM = 20;  // if closer than this, stop

int LID_OPEN_ANGLE = 90;   // calibrate for your box
int LID_CLOSED_ANGLE = 0;  // calibrate for your box
bool lidOpen = false;

unsigned long lastSensorRead = 0;
float temperature = 0, distanceCm = -1;
double pressure = 0;
String lastCmdShown = "-";

// ---------------- BLE callbacks ----------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    bleConnected = true;
  }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = String(c->getValue().c_str());
    v.trim();
    if (v.length() > 0) {
      incomingCmd = v;
      newCommand = true;
    }
  }
};

void speak(const String &bnText) {
  if (bleConnected) {
    txChar->setValue(bnText.c_str());
    txChar->notify();
  }
  Serial.println(bnText);
}

// ---------------- Motor control ----------------
void setLeftMotor(int speed) {
  if (speed >= 0) {
    analogWrite(L_RPWM, speed);
    analogWrite(L_LPWM, 0);
  } else {
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, -speed);
  }
}
void setRightMotor(int speed) {
  if (speed >= 0) {
    analogWrite(R_RPWM, speed);
    analogWrite(R_LPWM, 0);
  } else {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, -speed);
  }
}
void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  currentMode = IDLE;
}
void driveForward(int spd = DRIVE_SPEED) {
  setLeftMotor(spd);
  setRightMotor(spd);
}
void driveBackward(int spd = DRIVE_SPEED) {
  setLeftMotor(-spd);
  setRightMotor(-spd);
}
void turnLeft(int spd = TURN_SPEED) {
  setLeftMotor(-spd);
  setRightMotor(spd);
}
void turnRight(int spd = TURN_SPEED) {
  setLeftMotor(spd);
  setRightMotor(-spd);
}

// ---------------- Ultrasonic ----------------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);  // 25ms timeout ~ 4m
  if (duration == 0) return -1;
  return duration * 0.0343f / 2.0f;
}

// ---------------- Lid servo ----------------
void openLid() {
  lidServo.write(LID_OPEN_ANGLE);
  lidOpen = true;
  speak("বক্স খোলা হয়েছে");
}
void closeLid() {
  lidServo.write(LID_CLOSED_ANGLE);
  lidOpen = false;
  speak("বক্স বন্ধ করা হয়েছে");
}

// ---------------- OLED ----------------
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("RC Car Status");
  display.print("Mode: ");
  display.println(currentMode == IDLE ? "Idle" : currentMode == DRIVING ? "Drive"
                                                                        : "Follow");
  display.print("Lid: ");
  display.println(lidOpen ? "Open" : "Closed");
  display.print("Dist: ");
  display.print(distanceCm, 0);
  display.println("cm");
  display.print("Temp: ");
  display.print(temperature, 1);
  display.println("C");
  display.print("Cmd: ");
  display.println(lastCmdShown);
  display.print("BLE: ");
  display.println(bleConnected ? "Connected" : "Waiting");
  display.display();
}

// ---------------- Command handling ----------------
// The phone app sends one of these canonical Bangla words/phrases.
void handleCommand(String cmd) {
  lastCmdShown = cmd;

  if (cmd == "সামনে") {
    currentMode = DRIVING;
    driveForward();
    driveUntil = millis() + DRIVE_MS;
    speak("সামনে যাচ্ছি");
  } else if (cmd == "পেছনে") {
    currentMode = DRIVING;
    driveBackward();
    driveUntil = millis() + DRIVE_MS;
    speak("পেছনে যাচ্ছি");
  } else if (cmd == "বামে") {
    currentMode = DRIVING;
    turnLeft();
    driveUntil = millis() + DRIVE_MS;
    speak("বামে ঘুরছি");
  } else if (cmd == "ডানে") {
    currentMode = DRIVING;
    turnRight();
    driveUntil = millis() + DRIVE_MS;
    speak("ডানে ঘুরছি");
  } else if (cmd == "থামো") {
    stopMotors();
    speak("থেমে গেছি");
  } else if (cmd == "ফলো") {
    currentMode = FOLLOWING;
    speak("আমি আপনাকে ফলো করছি");
  } else if (cmd == "ফলো বন্ধ") {
    stopMotors();
    speak("ফলো করা বন্ধ করেছি");
  } else if (cmd == "খোলো") {
    openLid();
  } else if (cmd == "বন্ধ") {
    closeLid();
  } else if (cmd == "স্ট্যাটাস") {
    String msg = "তাপমাত্রা " + String(temperature, 1) + " ডিগ্রি, দূরত্ব " + String(distanceCm, 0) + " সেন্টিমিটার";
    speak(msg);
  } else {
    speak("দুঃখিত, বুঝতে পারিনি");
  }

  updateDisplay();
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  bmp.begin();
  dht.begin();

  lidServo.setPeriodHertz(50);
  lidServo.attach(SERVO_PIN, 500, 2400);
  closeLid();

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();

  BLEDevice::init("BanglaRC-Car");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());
  BLEService *service = bleServer->createService(SERVICE_UUID);

  txChar = service->createCharacteristic(CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic *rxChar = service->createCharacteristic(CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  rxChar->setCallbacks(new RxCallbacks());

  service->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  updateDisplay();
  Serial.println("BLE advertising as BanglaRC-Car");
}

// ---------------- Loop ----------------
void loop() {
  if (newCommand) {
    newCommand = false;
    handleCommand(incomingCmd);
  }

  // Auto-stop after a timed drive pulse
  if (currentMode == DRIVING && millis() > driveUntil) {
    stopMotors();
  }

  // Follow-me behavior
  if (currentMode == FOLLOWING) {
    if (distanceCm < 0) {
      setLeftMotor(0);
      setRightMotor(0);
    } else if (distanceCm > FOLLOW_FAR_CM) {
      driveForward(DRIVE_SPEED);
    } else if (distanceCm < FOLLOW_NEAR_CM) {
      setLeftMotor(0);
      setRightMotor(0);
    } else {
      driveForward(120);
    }
  }

  // Periodic sensor read
  if (millis() - lastSensorRead > 300) {
    lastSensorRead = millis();
    distanceCm = readDistanceCm();
    temperature = dht.readTemperature();
    pressure = bmp.readPressure();
    updateDisplay();
  }

  // RFID (read-only for now; UID printed to Serial — ask if you want
  // this to gate lid open/close by authorized card)
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("Card UID: ");
    for (byte i = 0; i < rfid.uid.size; i++) Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.println();
    rfid.PICC_HaltA();
  }
}
