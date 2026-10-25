// Imu6500.h - minimal I2C driver for MPU-6500 / MPU-9250 / MPU-9255 (accel + gyro only).
// No library needed. The magnetometer on the 9250 isn't used.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "DriveMotion.h"

class Imu6500 {
 public:
  uint8_t whoAmI = 0;

  bool begin() {
    const uint8_t addrs[2] = {0x68, 0x69};              // AD0 low / high
    for (int i = 0; i < 2; i++) {
      addr_ = addrs[i];
      uint8_t id;
      if (!readReg(0x75, id)) continue;
      if (id == 0x70 || id == 0x71 || id == 0x73 || id == 0x68) { whoAmI = id; return configure(); }
    }
    return false;
  }

  // accel in g, gyro in rad/s
  bool read(Vec3 &a, Vec3 &w) {
    Wire.beginTransmission(addr_);
    Wire.write((uint8_t)0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)addr_, (uint8_t)14, (uint8_t)true) != 14) return false;
    int16_t v[7];
    for (int i = 0; i < 7; i++) { uint8_t hi = Wire.read(), lo = Wire.read(); v[i] = (int16_t)((hi << 8) | lo); }
    a = Vec3{v[0] / 8192.0f, v[1] / 8192.0f, v[2] / 8192.0f};                     // +-4 g
    const float k = 0.0174533f / 65.5f;                                           // +-500 dps -> rad/s
    w = Vec3{v[4] * k, v[5] * k, v[6] * k};
    return true;
  }

 private:
  uint8_t addr_ = 0x68;
  bool writeReg(uint8_t r, uint8_t val) {
    Wire.beginTransmission(addr_); Wire.write(r); Wire.write(val);
    return Wire.endTransmission() == 0;
  }
  bool readReg(uint8_t r, uint8_t &out) {
    Wire.beginTransmission(addr_); Wire.write(r);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)addr_, (uint8_t)1, (uint8_t)true) != 1) return false;
    out = Wire.read(); return true;
  }
  bool configure() {
    bool ok = writeReg(0x6B, 0x80); delay(100);          // reset
    ok &= writeReg(0x6B, 0x01); delay(10);               // wake, PLL clock
    ok &= writeReg(0x1A, 0x03);                          // gyro low-pass ~41 Hz
    ok &= writeReg(0x1B, 0x08);                          // gyro +-500 dps
    ok &= writeReg(0x1C, 0x08);                          // accel +-4 g
    ok &= writeReg(0x1D, 0x03);                          // accel low-pass ~41 Hz
    ok &= writeReg(0x19, 0x09);                          // 100 Hz internal rate
    return ok;
  }
};
