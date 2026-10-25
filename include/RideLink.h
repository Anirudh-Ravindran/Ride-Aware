// RideLink.h - BLE side of the ride hand-off: a Nordic UART Service named "ScooterGuard".
// The carrier writes command lines to RX and gets replies as notifications on TX, in MTU-sized
// chunks; every reply is one JSON line ending in '\n'. USB Serial takes the same commands.
// nRF Connect or Bluefruit Connect can drive it by hand for testing.
//
//   LIST          {"resp":"list","recording":7,"rides":[{"id":3,"bytes":812},...]}
//                 recording = id of the ride still in progress (0 = none); it is not in "rides"
//   GET <id>      {"resp":"ride","id":3,"ride":{...POST /api/rides body...}}
//   DEL <id>      {"resp":"del","id":3,"ok":true}    send only after the server answered 201
//   TIME <unix>   {"resp":"time","ok":true}          UTC seconds; rides started after this carry startedAt
//   SIG L|R|H|OFF {"resp":"sig","ok":true}           indicators: left, right, hazard, off
//   RECAL         {"resp":"recal","ok":true}         forget the forward axis and restart to relearn it
//
// Carrier loop: TIME now -> LIST -> for each id: GET, add GPS fields, POST /api/rides, on 201 DEL.
// The commands themselves are handled in main.cpp.
#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

class RideLink : public BLEServerCallbacks, public BLECharacteristicCallbacks {
 public:
  void begin(const char *name) {
    inbox_ = xQueueCreate(256, 1);
    BLEDevice::init(name);
    BLEDevice::setMTU(247);
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(this);
    BLEService *svc = server->createService(kService);
    tx_ = svc->createCharacteristic(kTx, BLECharacteristic::PROPERTY_NOTIFY);
    tx_->addDescriptor(new BLE2902());
    BLECharacteristic *rx = svc->createCharacteristic(kRx, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(this);
    svc->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(kService);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
  }

  bool connected() const { return connected_; }

  // Next complete command line from the carrier, if any. Call from loop().
  bool readLine(String &line) {
    char c;
    while (xQueueReceive(inbox_, &c, 0) == pdTRUE) {
      if (c != '\n') { if (part_.length() < 64) part_ += c; continue; }
      if (!part_.length()) continue;
      line = part_; part_ = "";
      return true;
    }
    return false;
  }

  // Queue a reply; pump() sends it a chunk at a time so the IMU loop never stalls.
  void send(const String &s) { if (connected_) out_ += s; }

  void pump() {
    if (!connected_) { out_ = ""; off_ = 0; return; }
    if (off_ >= out_.length()) { if (off_) { out_ = ""; off_ = 0; } return; }
    if (micros() - lastUs_ < 8000) return;              // pace notifications so none are dropped
    lastUs_ = micros();
    size_t n = out_.length() - off_, max = mtu_ - 3;
    if (n > max) n = max;
    tx_->setValue((uint8_t *)out_.c_str() + off_, n);
    tx_->notify();
    off_ += n;
  }

 private:
  static constexpr const char *kService = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
  static constexpr const char *kRx = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";   // carrier -> device
  static constexpr const char *kTx = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";   // device -> carrier
  QueueHandle_t inbox_ = nullptr;
  BLECharacteristic *tx_ = nullptr;
  volatile bool connected_ = false;
  volatile uint16_t mtu_ = 23;
  String part_, out_;
  size_t off_ = 0;
  uint32_t lastUs_ = 0;

  // These run on the BLE task: only touch the queue and the volatile flags.
  void onConnect(BLEServer *) override { mtu_ = 23; connected_ = true; }
  void onDisconnect(BLEServer *s) override { connected_ = false; s->startAdvertising(); }
  void onMtuChanged(BLEServer *, esp_ble_gatts_cb_param_t *p) override { mtu_ = p->mtu.mtu; }
  void onWrite(BLECharacteristic *c) override {
    auto v = c->getValue();
    for (size_t i = 0; i < v.length(); i++) { char ch = v[i] == '\r' ? '\n' : v[i]; xQueueSend(inbox_, &ch, 0); }
    char nl = '\n';                                     // one write = one command (for hand testing)
    xQueueSend(inbox_, &nl, 0);
  }
};
