/*
  BLE-ONLY TEST SKETCH
  ---------------------
  Purpose: isolate whether your Bluetooth problem is in the ESP32 firmware
  or in the phone/browser. This sketch does NOTHING except advertise as
  "BanglaRC-Car" and echo back whatever text you send it. No motors, no
  sensors, no RFID — so if this doesn't work, the problem is BLE/board
  related, not a library conflict with the rest of the project.

  HOW TO USE:
  1. Upload this sketch alone (comment out / don't upload the main sketch).
  2. Open Serial Monitor at 115200 baud. You should see:
       "BLE test — advertising as BanglaRC-Car"
     and it should NOT print anything crash-like or keep restarting.
  3. On your phone, install "nRF Connect" (Android, free, Nordic Semi) —
     NOT the web page yet. Open it, tap Scan, look for "BanglaRC-Car".
       - If it does NOT show up here -> the problem is 100% in the ESP32
         firmware/board/antenna, not your website. See notes at bottom.
       - If it DOES show up -> connect to it in nRF Connect, open the
         service starting 6e400001..., write some text (as UTF-8) to the
         6e400002... characteristic, and you should see the same text
         echoed back as a notification on 6e400003.... If that works,
         your firmware is 100% fine and the problem is in Chrome/website
         permissions (see the checklist you were given in chat).
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *txChar;
bool connected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    connected = true;
    Serial.println("Phone connected!");
  }
  void onDisconnect(BLEServer* s) override {
    connected = false;
    Serial.println("Phone disconnected, restarting advertising.");
    BLEDevice::startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = String(c->getValue().c_str());
    Serial.print("Received: ");
    Serial.println(v);
    String reply = "echo: " + v;
    txChar->setValue(reply.c_str());
    txChar->notify();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting BLE test...");

  BLEDevice::init("BanglaRC-Car");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  txChar = service->createCharacteristic(CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic *rxChar = service->createCharacteristic(CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  rxChar->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE test — advertising as BanglaRC-Car");
}

void loop() {
  // nothing to do — everything happens in the BLE callbacks
  delay(1000);
}

/*
  IF nRF Connect NEVER sees "BanglaRC-Car":
   - Confirm you selected the right board (ESP32S3 Dev Module) and the
     right USB port; re-flash and immediately reopen Serial Monitor to
     confirm "BLE test — advertising..." actually prints.
   - Some ESP32-S3 dev boards need "USB CDC On Boot: Enabled" or Serial
     prints won't show at all even though the board is running fine —
     don't assume "no serial output" means "not running."
   - Try moving the board closer to the phone (under 2m) for the first test.
   - Double check you're not accidentally running two BLE inits (don't
     have both this sketch and something else calling BLEDevice::init).
*/
