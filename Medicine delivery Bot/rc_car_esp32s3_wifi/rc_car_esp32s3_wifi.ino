/*
  RC car - phone-controlled (WiFi/WebSocket) + sensors + RFID + OLED
  ESP32-S3-WROOM-1 (N16R8) + Arduino IDE

  ============================================================
  WHAT CHANGED: RC receiver is GONE
  ============================================================
  Per your choice, the FS-iA10B / pulseIn reading is removed
  entirely. Driving now comes from your phone over WiFi, using a
  WebSocket message: "DRIVE:<forward-back>,<turn>", each -100..100.
  A companion HTML file (voice_control.html - separate file) runs
  in your phone's browser and sends these.

  Steering mixing is now proper differential/arcade drive (both
  forward and turn can be applied at once), not the old
  "whichever axis dominates wins" logic - smoother for touch
  control:
      leftPower  = fb + turn
      rightPower = fb - turn
  then mapped through your existing mirrored-motor-polarity
  convention (forward = L:-speed, R:+speed).

  ============================================================
  WHY THIS SPECIFIC DESIGN (please read before you wire/flash)
  ============================================================
  1. Gemini calls and the mic both live in the PHONE'S BROWSER,
     not on the ESP32. The ESP32 has no business calling an LLM
     directly here - it just needs to (a) get driving numbers and
     (b) get told when to show "listening / thinking / replying"
     on the OLED, over WiFi.

  2. The companion HTML file is NOT served by the ESP32. It's a
     plain .html file you save onto your phone and open directly
     in Chrome (file:// address). This is deliberate: browsers
     only allow microphone access (Web Speech API) from a
     "secure context" - https:// or a local file - and getting a
     real HTTPS certificate working on an ESP32 web server is a
     much bigger, flakier project than it's worth here. Opening
     the file locally sidesteps that entirely while the page
     still talks to the ESP32 over a plain WebSocket for driving
     and telemetry.

  3. mDNS: the ESP32 advertises itself as "esp32car.local" so the
     phone page can always find it even if its IP address changes
     on your WiFi/hotspot. If mDNS doesn't resolve on your phone,
     the ESP32's IP is also printed on Serial and the OLED at
     boot - use that IP directly in the HTML file instead.

  4. Phone and ESP32 must be on the SAME WiFi network (e.g. your
     phone's own hotspot, with the ESP32 connecting to it).

  ============================================================
  LIBRARIES TO INSTALL (Library Manager)
  ============================================================
    - "ESPAsyncWebServer" (also pulls in the WebSocket support)
    - "AsyncTCP"                              (its dependency)
    - "DHT sensor library" by Adafruit         (+ "Adafruit Unified Sensor")
    - "Adafruit BMP085 Library"                 (works for the BMP180)
    - "MFRC522" by GithubCommunity
    - "Adafruit GFX Library"
    - "Adafruit SSD1306"
    - "ESP32Servo" by Kevin Harrington
  ESPmDNS and WiFi are bundled with the ESP32 board package -
  nothing extra to install for those.

  ============================================================
  WIRING - unchanged from before except the RC receiver is gone
  ============================================================
    Left BTS7960:  RPWM=4  LPWM=5  EN=6
    Right BTS7960: RPWM=7  LPWM=8  EN=9
    I2C (OLED + BMP180): SDA=1  SCL=2
    DHT11: DATA=15
    RC522 (SPI, 3.3V only): SS=10 SCK=12 MOSI=11 MISO=13 RST=14
    Lid servo: signal=21 (from a 5V regulator, not the battery)
    ALL grounds (both BTS7960, ESP32, OLED, BMP180, DHT11, RC522,
    servo) tied to one common ground.

  Edit WIFI_SSID / WIFI_PASSWORD below before flashing.
*/

// ---------- includes ----------

#include <math.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// ---------- EDIT THESE ----------
const char* WIFI_SSID     = "YOUR_WIFI_OR_HOTSPOT_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* MDNS_NAME     = "esp32car"; // phone connects to ws://esp32car.local/ws

// ================================================================
// ---------- PIN DEFINITIONS ----------
// ================================================================

const int L_RPWM = 4;
const int L_LPWM = 5;
const int L_EN   = 6;

const int R_RPWM = 7;
const int R_LPWM = 8;
const int R_EN   = 9;

const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int PWM_MAX_DUTY = 255;
const unsigned long LOOP_DELAY_MS = 20;
const unsigned long DRIVE_TIMEOUT_MS = 400; // stop if phone stops sending

#define I2C_SDA   1
#define I2C_SCL   2

#define DHT_PIN   15
#define DHT_TYPE  DHT11

#define RFID_SS   10
#define RFID_SCK  12
#define RFID_MOSI 11
#define RFID_MISO 13
#define RFID_RST  14

#define LID_SERVO_PIN 21
#define SERVO_CLOSED_ANGLE 0
#define SERVO_OPEN_ANGLE   90
const unsigned long LID_OPEN_MS = 3000;

// ================================================================
// ---------- OBJECTS ----------
// ================================================================

Adafruit_SSD1306 display(128, 64, &Wire, -1);
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP085 bmp;
MFRC522 rfid(RFID_SS, RFID_RST);
Servo lidServo;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------- driving state (set by phone over WebSocket) ----------

float g_fb = 0, g_turn = 0;              // -100..100 each
unsigned long lastDriveCmdAt = 0;

// ---------- voice-assistant UI state (driven by phone) ----------

enum VoiceState { VOICE_IDLE, VOICE_LISTENING, VOICE_THINKING, VOICE_SPEAKING };
VoiceState voiceState = VOICE_IDLE;
unsigned long voiceStateChangedAt = 0;
String lastReplyText = "";
const unsigned long SPEAKING_DISPLAY_MS = 6000;

// ---------- sensor cache ----------

float g_temperature = NAN, g_humidity = NAN, g_pressure = NAN;
unsigned long lastDHTRead = 0, lastBMPRead = 0, lastTelemetrySend = 0;
const unsigned long DHT_INTERVAL_MS = 2500;
const unsigned long BMP_INTERVAL_MS = 1000;
const unsigned long TELEMETRY_INTERVAL_MS = 2000;

// ---------- RFID state ----------

String lastUID = "";
bool lastAccessGranted = false;
unsigned long lidOpenedAt = 0;
bool lidIsOpen = false;

const char* AUTHORIZED_UIDS[] = {
  "04A3F1C2",   // <-- replace with your real card UID(s)
  "AABBCCDD"
};
const int NUM_AUTHORIZED_UIDS = sizeof(AUTHORIZED_UIDS) / sizeof(AUTHORIZED_UIDS[0]);

// ---------- display page cycling ----------

unsigned long lastPageSwitch = 0;
int displayPage = 0; // 0 = car status, 1 = sensors, 2 = last RFID scan
const unsigned long PAGE_INTERVAL_MS = 3000;

// ================================================================
// ---------- SETUP ----------
// ================================================================

void setup() {
  Serial.begin(115200);

  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN, LOW);
  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found - check wiring/address (0x3C or 0x3D)");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.println("Connecting WiFi...");
    display.display();
  }

  dht.begin();

  if (!bmp.begin()) {
    Serial.println("BMP180 not found - check I2C wiring");
  }

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();

  lidServo.setPeriodHertz(50);
  lidServo.attach(LID_SERVO_PIN, 500, 2400);
  lidServo.write(SERVO_CLOSED_ANGLE);

  connectWiFi();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  digitalWrite(L_EN, HIGH); // motors enabled now that WiFi + failsafe timer are live
  digitalWrite(R_EN, HIGH);
  lastDriveCmdAt = millis();

  Serial.println("Car ready.");
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    if (MDNS.begin(MDNS_NAME)) {
      Serial.print("mDNS ready: ws://");
      Serial.print(MDNS_NAME);
      Serial.println(".local/ws");
    } else {
      Serial.println("mDNS failed to start - use the IP address above instead");
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi connected:");
    display.println(WiFi.localIP());
    display.print("ws://");
    display.print(MDNS_NAME);
    display.println(".local/ws");
    display.display();
    delay(2000);
  } else {
    Serial.println();
    Serial.println("WiFi FAILED - check credentials. Retrying in background.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi FAILED");
    display.println("Check SSID/password");
    display.display();
  }
}

// ================================================================
// ---------- WEBSOCKET ----------
// ================================================================

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected\n", client->id());
    lastDriveCmdAt = millis(); // don't instantly failsafe-trip on connect
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());
    g_fb = 0; g_turn = 0;
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      handleWsCommand(String((char*)data));
    }
  }
}

void handleWsCommand(const String &msgIn) {
  String msg = msgIn;
  msg.trim();

  if (msg.startsWith("DRIVE:")) {
    String rest = msg.substring(6);
    int comma = rest.indexOf(',');
    if (comma > 0) {
      g_fb = constrain(rest.substring(0, comma).toFloat(), -100, 100);
      g_turn = constrain(rest.substring(comma + 1).toFloat(), -100, 100);
      lastDriveCmdAt = millis();
    }
  } else if (msg == "LISTEN_START") {
    voiceState = VOICE_LISTENING;
    voiceStateChangedAt = millis();
  } else if (msg == "LISTEN_STOP") {
    voiceState = VOICE_THINKING;
    voiceStateChangedAt = millis();
  } else if (msg.startsWith("SPEAK:")) {
    lastReplyText = msg.substring(6);
    voiceState = VOICE_SPEAKING;
    voiceStateChangedAt = millis();
  } else if (msg == "IDLE") {
    voiceState = VOICE_IDLE;
  }
}

void wsBroadcast(const String &line) {
  if (ws.count() > 0) ws.textAll(line);
}

// ================================================================
// ---------- DRIVING ----------
// ================================================================

void setMotor(int rpwmPin, int lpwmPin, float speed) {
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

// fb, turn: -100..100. Proper differential/arcade mixing so forward
// and turning combine smoothly - matches touch-joystick control much
// better than a single-axis "winner takes all" scheme.
void drive(float fb, float turn) {
  float leftRaw  = constrain(fb + turn, -100, 100);
  float rightRaw = constrain(fb - turn, -100, 100);
  // mirrored motor polarity convention from your original sketch:
  // forward = L:-speed, R:+speed
  setMotor(L_RPWM, L_LPWM, -leftRaw);
  setMotor(R_RPWM, R_LPWM, rightRaw);
}

// ================================================================
// ---------- SENSORS ----------
// ================================================================

void updateSensors() {
  unsigned long now = millis();

  if (now - lastDHTRead >= DHT_INTERVAL_MS) {
    lastDHTRead = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) { g_humidity = h; g_temperature = t; }
  }

  if (now - lastBMPRead >= BMP_INTERVAL_MS) {
    lastBMPRead = now;
    g_pressure = bmp.readPressure() / 100.0;
  }

  if (now - lastTelemetrySend >= TELEMETRY_INTERVAL_MS) {
    lastTelemetrySend = now;
    wsBroadcast("SENSOR:T=" + String(g_temperature, 1) +
                ",H=" + String(g_humidity, 1) +
                ",P=" + String(g_pressure, 1));
  }
}

// ================================================================
// ---------- RFID ----------
// ================================================================

bool isAuthorized(const String &uid) {
  for (int i = 0; i < NUM_AUTHORIZED_UIDS; i++) {
    if (uid.equalsIgnoreCase(AUTHORIZED_UIDS[i])) return true;
  }
  return false;
}

void updateRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    if (lidIsOpen && millis() - lidOpenedAt >= LID_OPEN_MS) {
      lidServo.write(SERVO_CLOSED_ANGLE);
      lidIsOpen = false;
    }
    return;
  }

  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  bool granted = isAuthorized(uid);
  lastUID = uid;
  lastAccessGranted = granted;

  Serial.print("RFID scan: "); Serial.print(uid); Serial.println(granted ? " GRANTED" : " DENIED");
  wsBroadcast("RFID:" + uid + "," + (granted ? "GRANTED" : "DENIED"));

  if (granted) {
    lidServo.write(SERVO_OPEN_ANGLE);
    lidIsOpen = true;
    lidOpenedAt = millis();
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ================================================================
// ---------- OLED DISPLAY ----------
// ================================================================

void drawListeningAnimation() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Listening...");
  int cx = 64, cy = 40;
  int phase = (millis() / 150) % 4;
  for (int i = 0; i < 4; i++) {
    int barH = 6 + ((i + phase) % 4) * 6;
    int x = cx - 30 + i * 20;
    display.fillRect(x, cy + 15 - barH, 10, barH, SSD1306_WHITE);
  }
}

void drawThinking() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Thinking...");
  int dots = (millis() / 400) % 4;
  String s = "";
  for (int i = 0; i < dots; i++) s += ".";
  display.setCursor(0, 20);
  display.println(s);
}

void drawSpeaking() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Reply ready");
  display.setCursor(0, 14);
  display.println(lastReplyText); // ASCII placeholder only - see notes
}

void drawCarStatusPage() {
  display.setCursor(0, 0);
  display.println(WiFi.status() == WL_CONNECTED ? "WiFi: OK" : "WiFi: DOWN");
  display.setCursor(0, 12);
  display.print("Fwd/back: "); display.println((int)g_fb);
  display.setCursor(0, 24);
  display.print("Turn: "); display.println((int)g_turn);
  display.setCursor(0, 36);
  display.print("Phone link: ");
  display.println(ws.count() > 0 ? "connected" : "waiting");
  display.setCursor(0, 50);
  display.print("IP: "); display.println(WiFi.localIP());
}

void drawSensorPage() {
  display.setCursor(0, 0);
  display.println("Environment");
  display.setCursor(0, 16);
  display.print("Temp: "); display.print(isnan(g_temperature) ? -1 : g_temperature, 1); display.println(" C");
  display.setCursor(0, 30);
  display.print("Humid: "); display.print(isnan(g_humidity) ? -1 : g_humidity, 1); display.println(" %");
  display.setCursor(0, 44);
  display.print("Press: "); display.print(g_pressure, 1); display.println(" hPa");
}

void drawRfidPage() {
  display.setCursor(0, 0);
  display.println("Last card scan");
  if (lastUID.length() == 0) {
    display.setCursor(0, 20);
    display.println("No card scanned yet");
  } else {
    display.setCursor(0, 16); display.println(lastUID);
    display.setCursor(0, 30); display.println(lastAccessGranted ? "GRANTED" : "DENIED");
    display.setCursor(0, 44); display.println(lidIsOpen ? "Lid: OPEN" : "Lid: closed");
  }
}

void updateDisplay() {
  display.clearDisplay();

  if (voiceState == VOICE_SPEAKING && millis() - voiceStateChangedAt > SPEAKING_DISPLAY_MS) {
    voiceState = VOICE_IDLE;
  }

  switch (voiceState) {
    case VOICE_LISTENING: drawListeningAnimation(); break;
    case VOICE_THINKING:  drawThinking(); break;
    case VOICE_SPEAKING:  drawSpeaking(); break;
    case VOICE_IDLE:
    default: {
      unsigned long now = millis();
      if (now - lastPageSwitch >= PAGE_INTERVAL_MS) {
        lastPageSwitch = now;
        displayPage = (displayPage + 1) % 3;
      }
      if (displayPage == 0) drawCarStatusPage();
      else if (displayPage == 1) drawSensorPage();
      else drawRfidPage();
      break;
    }
  }
  display.display();
}

// ================================================================
// ---------- MAIN LOOP ----------
// ================================================================

void loop() {
  ws.cleanupClients();

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(L_EN, LOW);
    digitalWrite(R_EN, LOW);
    stopMotors();
  } else if (millis() - lastDriveCmdAt > DRIVE_TIMEOUT_MS) {
    g_fb = 0; g_turn = 0;
    stopMotors();
  } else {
    drive(g_fb, g_turn);
  }

  updateSensors();
  updateRFID();
  updateDisplay();

  delay(LOOP_DELAY_MS);
}
