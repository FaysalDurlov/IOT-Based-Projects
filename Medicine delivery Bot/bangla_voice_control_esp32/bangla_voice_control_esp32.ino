/*
  Bangla Voice-Controlled RC Car — Follow Me + Lid Servo
  Board: ESP32-S3-WROOM-1 N16R8 (Arduino IDE)
  COMMS: WiFi (HTTP) — phone and ESP32 must be on the SAME Wi-Fi network.

  ARCHITECTURE:
  The ESP32 cannot understand speech itself. Your phone (using the
  companion "bangla_voice_control_wifi.html" page) listens in Bangla,
  turns speech into short command words, and sends them to this
  sketch as a plain HTTP GET request over your local Wi-Fi. This
  sketch drives the hardware and sends back a short Bangla reply as
  the HTTP response body, which your phone speaks aloud.

  REQUIRED LIBRARIES (Arduino IDE > Tools > Manage Libraries):
   - ESP32Servo
   - Adafruit GFX Library
   - Adafruit SSD1306
   - Adafruit Unified Sensor
   - Adafruit BMP085 Library      (works for BMP180 / GY-68)
   - DHT sensor library (by Adafruit)
   - MFRC522                     (by GithubCommunity)
   (WiFi.h, WebServer.h, ESPmDNS.h come built-in with the ESP32 board
    package — no BLE libraries needed anymore.)

  BOARD SETTINGS (Tools menu):
   - Board: "ESP32S3 Dev Module"
   - USB CDC On Boot: "Enabled"
   - Flash Size: "16MB"
   - PSRAM: "OPI PSRAM"
   - Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)" or similar

  BEFORE UPLOADING:
   - Set WIFI_SSID / WIFI_PASSWORD below to your home Wi-Fi.
   - After upload, open Serial Monitor at 115200 baud — the car
     will print its IP address once connected. Type that IP into
     the HTML page on your phone.

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
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ---------------- WiFi settings ----------------
const char* WIFI_SSID     = "Mozahar2";
const char* WIFI_PASSWORD = "mozahar3118";

// mDNS hostname — lets you optionally use http://banglarc.local/ instead
// of typing the IP. Works on many phones, but not guaranteed on all
// browsers, so the IP address (printed on Serial + shown on OLED) is
// still the reliable fallback.
const char* MDNS_NAME = "banglarc";

// ---------------- Pin map (unchanged) ----------------
#define I2C_SDA   1
#define I2C_SCL   2

#define L_RPWM    4
#define L_LPWM    5
#define L_EN      6

#define R_RPWM    7
#define R_LPWM    8
#define R_EN      9

#define DHT_PIN   11
#define DHT_TYPE  DHT11

#define SERVO_PIN 21

#define TRIG_PIN  12
#define ECHO_PIN  13

#define RFID_SCK  39
#define RFID_MISO 41
#define RFID_MOSI 40
#define RFID_SS   42
#define RFID_RST  38

// ---------------- Objects ----------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_BMP085 bmp;
DHT dht(DHT_PIN, DHT_TYPE);
Servo lidServo;
MFRC522 rfid(RFID_SS, RFID_RST);

// ---------------- WiFi / HTTP server ----------------
WebServer server(80);
bool wifiConnected = false;
String replyText = "-"; // last Bangla reply, set by speak()

// ---------------- State ----------------
enum Mode { IDLE, DRIVING, FOLLOWING };
Mode currentMode = IDLE;
unsigned long driveUntil = 0;

const int DRIVE_MS    = 1000;  // each voice drive command runs this long
const int DRIVE_SPEED = 200;   // 0-255
const int TURN_SPEED  = 180;

const float FOLLOW_FAR_CM  = 40; // if farther than this, drive forward
const float FOLLOW_NEAR_CM = 20; // if closer than this, stop

int LID_OPEN_ANGLE   = 90;  // calibrate for your box
int LID_CLOSED_ANGLE = 0;   // calibrate for your box
bool lidOpen = false;

unsigned long lastSensorRead = 0;
float temperature = 0, distanceCm = -1;
double pressure = 0;
String lastCmdShown = "-";

// ---------------- Reply helper (replaces old BLE speak()) ----------------
void speak(const String &bnText) {
  replyText = bnText;
  Serial.println(bnText);
}

// ---------------- Motor control (unchanged) ----------------
void setLeftMotor(int speed) {
  if (speed >= 0) { analogWrite(L_RPWM, speed); analogWrite(L_LPWM, 0); }
  else            { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, -speed); }
}
void setRightMotor(int speed) {
  if (speed >= 0) { analogWrite(R_RPWM, speed); analogWrite(R_LPWM, 0); }
  else            { analogWrite(R_RPWM, 0); analogWrite(R_LPWM, -speed); }
}
void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  currentMode = IDLE;
}
void driveForward(int spd = DRIVE_SPEED) { setLeftMotor(spd); setRightMotor(spd); }
void driveBackward(int spd = DRIVE_SPEED) { setLeftMotor(-spd); setRightMotor(-spd); }
void turnLeft(int spd = TURN_SPEED)  { setLeftMotor(-spd); setRightMotor(spd); }
void turnRight(int spd = TURN_SPEED) { setLeftMotor(spd);  setRightMotor(-spd); }

// ---------------- Ultrasonic (unchanged) ----------------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout ~ 4m
  if (duration == 0) return -1;
  return duration * 0.0343f / 2.0f;
}

// ---------------- Lid servo (unchanged) ----------------
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
  display.println(currentMode == IDLE ? "Idle" : currentMode == DRIVING ? "Drive" : "Follow");
  display.print("Lid: ");  display.println(lidOpen ? "Open" : "Closed");
  display.print("Dist: "); display.print(distanceCm, 0); display.println("cm");
  display.print("Temp: "); display.print(temperature, 1); display.println("C");
  display.print("Cmd: ");  display.println(lastCmdShown);
  if (wifiConnected) {
    display.print("IP: "); display.println(WiFi.localIP());
  } else {
    display.println("WiFi: connecting...");
  }
  display.display();
}

// ---------------- Command handling ----------------
// Called from the HTTP handler with one of the canonical Bangla
// words/phrases. Returns the Bangla reply text (also stored via speak()).
String handleCommand(String cmd) {
  lastCmdShown = cmd;
  Serial.print("Command = ");
  Serial.println(cmd);

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
    String msg = "তাপমাত্রা " + String(temperature, 1) +
                 " ডিগ্রি, দূরত্ব " + String(distanceCm, 0) + " সেন্টিমিটার";
    speak(msg);
  } else {
    speak("দুঃখিত, বুঝতে পারিনি");
  }

  updateDisplay();
  return replyText;
}

// ---------------- HTTP handlers ----------------
// GET /cmd?c=<command>  -> runs the command, replies with Bangla text
void handleCmdRequest() {
  server.sendHeader("Access-Control-Allow-Origin", "*"); // allow phone page to fetch
  if (!server.hasArg("c")) {
    server.send(400, "text/plain; charset=utf-8", "কমান্ড পাইনি");
    return;
  }
  String cmd = server.arg("c");
  String reply = handleCommand(cmd);
  server.send(200, "text/plain; charset=utf-8", reply);
}

// GET /  -> simple health check page, useful for testing in a browser
void handleRoot() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String msg = "BanglaRC-Car is online. IP: " + WiFi.localIP().toString();
  server.send(200, "text/plain; charset=utf-8", msg);
}

void handleNotFound() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(404, "text/plain", "Not found");
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT); pinMode(L_EN, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT); pinMode(R_EN, OUTPUT);
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

  // ---- WiFi connect ----
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(400);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println();
    Serial.print("WiFi connected. IP address: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin(MDNS_NAME)) {
      Serial.print("mDNS started: http://");
      Serial.print(MDNS_NAME);
      Serial.println(".local/  (may not resolve on all phones — use the IP above if it fails)");
    }
  } else {
    Serial.println();
    Serial.println("WiFi FAILED to connect within 20s. Check SSID/password and retry (device will keep retrying in background).");
  }

  server.on("/", handleRoot);
  server.on("/cmd", handleCmdRequest);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started on port 80.");

  updateDisplay();
}

// ---------------- Loop ----------------
void loop() {
  server.handleClient();

  // Keep wifiConnected flag accurate + auto-retry if the connection drops
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      wifiConnected = false;
      Serial.println("WiFi disconnected, retrying...");
    }
    WiFi.reconnect();
  } else if (!wifiConnected) {
    wifiConnected = true;
    Serial.print("WiFi reconnected. IP address: ");
    Serial.println(WiFi.localIP());
  }

  // Auto-stop after a timed drive pulse
  if (currentMode == DRIVING && millis() > driveUntil) {
    stopMotors();
  }

  // Follow-me behavior
  if (currentMode == FOLLOWING) {
    if (distanceCm < 0) {
      setLeftMotor(0); setRightMotor(0);
    } else if (distanceCm > FOLLOW_FAR_CM) {
      driveForward(DRIVE_SPEED);
    } else if (distanceCm < FOLLOW_NEAR_CM) {
      setLeftMotor(0); setRightMotor(0);
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
