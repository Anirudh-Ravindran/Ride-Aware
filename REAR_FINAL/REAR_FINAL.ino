#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// ================= WIRELESS =================

constexpr uint32_t MAGIC = 0x52494446;
constexpr uint8_t CHANNEL = 1;
constexpr uint32_t LINK_TIMEOUT_MS = 3000;

constexpr uint8_t BROADCAST[] = {
  255, 255, 255, 255, 255, 255
};

// Must match the front sketch.
struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint32_t sequence;

  uint8_t source;
  uint8_t turn;
  uint8_t brake;
  uint8_t distanceOK;
  uint8_t imuOK;
  uint8_t blink;

  float distanceCm;
  float forwardAccel;
};

static_assert(sizeof(Packet) == 22, "Packet layout changed");

// Explicit declarations for Arduino.
bool takePacket(Packet &p, uint32_t &time);
void sendPacket(Packet &p);

portMUX_TYPE mailboxLock = portMUX_INITIALIZER_UNLOCKED;

Packet mailbox = {};
bool pendingPacket = false;
uint32_t receivedAt = 0;

bool radioReady = false;
uint32_t sequence = 0;

// ================= PIN CONNECTIONS =================

constexpr int LEFT_LED = 4;
constexpr int RIGHT_LED = 5;
constexpr int BRAKE_LED = 8;

constexpr int TRIG_PIN = 6;
constexpr int ECHO_PIN = 7;  // Through your voltage divider.

constexpr int IMU_SDA = 15;
constexpr int IMU_SCL = 16;

// ================= BRAKE SETTINGS =================

// 0 = X, 1 = Y, 2 = Z.
// Selected axis must point along the scooter's travel.
constexpr int FORWARD_AXIS = 0;

// +1 if the selected axis arrow points forward.
// -1 if it points backward.
constexpr float FORWARD_SIGN = 1.0f;

static_assert(
  FORWARD_AXIS >= 0 && FORWARD_AXIS <= 2,
  "Invalid forward axis"
);

constexpr float BRAKE_ON_MPS2 = 0.80f;
constexpr float BRAKE_RELEASE_MPS2 = 0.30f;

constexpr uint32_t BRAKE_CONFIRM_MS = 100;
constexpr uint32_t BRAKE_HOLD_MS = 400;

constexpr float ACCEL_FILTER_SECONDS = 0.08f;

constexpr uint32_t MAX_BRAKE_MS = 5000;
constexpr uint32_t BRAKE_REARM_MS = 500;

// Brake LED: 200 ms ON, 200 ms OFF.
constexpr uint32_t BRAKE_FLASH_HALF_MS = 200;

constexpr float G = 9.80665f;

// ================= IMU STATE =================

uint8_t imuAddress = 0;
bool imuReady = false;

float gravityDir[3] = {0, 0, 1};
float gyroBias[3] = {};

float latestAccel[3] = {};
float latestGyro[3] = {};

float forwardAccel = 0;

uint32_t imuUs = 0;
uint8_t consecutiveImuFailures = 0;

// ================= BRAKE STATE =================

bool brakeOn = false;
bool brakeCandidate = false;

uint32_t brakeCandidateAt = 0;
uint32_t brakeSupportedAt = 0;
uint32_t brakeStartedAt = 0;

bool brakeLockedOut = false;
bool releaseTiming = false;

uint32_t brakeReleasedAt = 0;

// ================= FRONT LINK STATE =================

uint8_t requestedTurn = 0;
bool frontSeen = false;
uint32_t lastFrontAt = 0;

// ================= DISTANCE STATE =================

float distanceCm = 0;

bool hasDistance = false;
bool distanceFresh = false;

uint32_t lastValidDistanceAt = 0;

// ================= TASK TIMERS =================

uint32_t lastPing = 0;
uint32_t lastImu = 0;
uint32_t lastSend = 0;
uint32_t lastLog = 0;

// ================= ESP-NOW RECEIVE =================

void onReceive(
  const esp_now_recv_info_t*,
  const uint8_t* data,
  int len
) {
  if (len != sizeof(Packet)) return;

  Packet p;
  memcpy(&p, data, sizeof(p));

  if (
    p.magic != MAGIC ||
    p.source != 1 ||
    p.turn > 2 ||
    p.brake > 1 ||
    p.distanceOK > 1 ||
    p.imuOK > 1 ||
    p.blink > 1 ||
    !isfinite(p.distanceCm) ||
    p.distanceCm < 0 ||
    p.distanceCm > 400 ||
    !isfinite(p.forwardAccel)
  ) {
    return;
  }

  portENTER_CRITICAL(&mailboxLock);

  mailbox = p;
  receivedAt = millis();
  pendingPacket = true;

  portEXIT_CRITICAL(&mailboxLock);
}

bool takePacket(Packet &p, uint32_t &time) {
  portENTER_CRITICAL(&mailboxLock);

  bool ready = pendingPacket;

  if (ready) {
    p = mailbox;
    time = receivedAt;
    pendingPacket = false;
  }

  portEXIT_CRITICAL(&mailboxLock);

  return ready;
}

// ================= ESP-NOW SETUP =================

bool startRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (
    esp_wifi_set_channel(
      CHANNEL,
      WIFI_SECOND_CHAN_NONE
    ) != ESP_OK
  ) {
    return false;
  }

  if (esp_now_init() != ESP_OK) return false;

  if (
    esp_now_register_recv_cb(onReceive) != ESP_OK
  ) {
    return false;
  }

  esp_now_peer_info_t peer = {};

  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  return esp_now_add_peer(&peer) == ESP_OK;
}

void sendPacket(Packet &p) {
  if (!radioReady) return;

  p.magic = MAGIC;
  p.sequence = sequence++;

  if (
    esp_now_send(
      BROADCAST,
      reinterpret_cast<const uint8_t*>(&p),
      sizeof(p)
    ) != ESP_OK
  ) {
    Serial.println("Radio: could not queue packet");
  }
}

// ================= IMU REGISTER FUNCTIONS =================

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t count) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  return Wire.requestFrom(imuAddress, count) == count;
}

int16_t readSigned16() {
  uint16_t highByte = Wire.read();
  uint16_t lowByte = Wire.read();

  return static_cast<int16_t>(
    (highByte << 8) | lowByte
  );
}

bool readMotion(float *a, float *w) {
  if (!readRegisters(0x3B, 14)) return false;

  for (int i = 0; i < 3; i++) {
    a[i] = readSigned16() * G / 16384.0f;
  }

  readSigned16();  // Skip temperature.

  for (int i = 0; i < 3; i++) {
    w[i] = readSigned16() * DEG_TO_RAD / 131.0f;
  }

  return true;
}

float length3(const float *v) {
  return sqrtf(
    v[0] * v[0] +
    v[1] * v[1] +
    v[2] * v[2]
  );
}

void normalizeGravity() {
  float magnitude = length3(gravityDir);

  if (magnitude > 0.001f) {
    for (int i = 0; i < 3; i++) {
      gravityDir[i] /= magnitude;
    }
  }
}

// ================= IMU INITIALIZATION =================

bool startIMU() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);

  bool found = false;

  for (
    uint8_t address = 0x68;
    address <= 0x69;
    address++
  ) {
    imuAddress = address;

    if (!readRegisters(0x75, 1)) {
      Serial.printf(
        "IMU: no readable identity at 0x%02X\n",
        address
      );

      Serial.println(
        "Check 3V3, GND, SDA=15, SCL=16."
      );

      continue;
    }

    uint8_t id = Wire.read();

    Serial.printf(
      "IMU at 0x%02X, chip ID 0x%02X\n",
      address,
      id
    );

    if (id != 0x68 && id != 0x70) {
      Serial.println(
        "IMU ERROR: unsupported chip identity."
      );
      continue;
    }

    if (!writeRegister(0x6B, 0x80)) {
      Serial.println(
        "IMU ERROR: reset register write failed."
      );
      continue;
    }

    delay(100);

    bool ok = writeRegister(0x6B, 0x01);
    delay(100);

    ok &= writeRegister(0x6C, 0x00);
    ok &= writeRegister(0x1B, 0x00);
    ok &= writeRegister(0x1C, 0x00);
    ok &= writeRegister(0x1A, 0x03);

    if (id == 0x70) {
      ok &= writeRegister(0x1D, 0x03);
    }

    if (ok) {
      found = true;
      break;
    }

    Serial.println(
      "IMU ERROR: configuration write failed."
    );
  }

  if (!found) {
    Serial.println(
      "IMU initialization failed; automatic braking disabled."
    );
    return false;
  }

  Serial.println(
    "CALIBRATING: keep the rear completely STILL."
  );

  float sumA[3] = {};
  float sumW[3] = {};

  float a[3];
  float w[3];

  float minMagnitude = 1000;
  float maxMagnitude = 0;

  int count = 0;

  for (int sample = 0; sample < 150; sample++) {
    if (readMotion(a, w)) {
      for (int i = 0; i < 3; i++) {
        sumA[i] += a[i];
        sumW[i] += w[i];
      }

      float magnitude = length3(a);

      minMagnitude = fminf(minMagnitude, magnitude);
      maxMagnitude = fmaxf(maxMagnitude, magnitude);

      count++;
    }

    delay(10);
  }

  Serial.printf(
    "Calibration: %d/150 reads, variation %.3f g\n",
    count,
    (maxMagnitude - minMagnitude) / G
  );

  if (count < 120) {
    Serial.println(
      "IMU ERROR: too many failed calibration reads."
    );
    return false;
  }

  if (maxMagnitude - minMagnitude > 0.15f * G) {
    Serial.println(
      "IMU ERROR: moved during calibration. Keep still and RESET."
    );
    return false;
  }

  for (int i = 0; i < 3; i++) {
    gravityDir[i] = sumA[i] / count;
    gyroBias[i] = sumW[i] / count;
  }

  float magnitude = length3(gravityDir);

  Serial.printf(
    "Gravity: %.3f g | Gyro mean: %.2f deg/s\n",
    magnitude / G,
    length3(gyroBias) / DEG_TO_RAD
  );

  if (
    magnitude < 0.8f * G ||
    magnitude > 1.2f * G
  ) {
    Serial.println(
      "IMU ERROR: stationary acceleration out of range."
    );
    return false;
  }

  if (length3(gyroBias) > 0.15f) {
    Serial.println(
      "IMU ERROR: excessive rotation or gyro bias."
    );
    return false;
  }

  normalizeGravity();
  imuUs = micros();

  return true;
}

// ================= BRAKE EVENT LOGIC =================

void updateBrakeState(uint32_t now, bool usable) {
  if (
    brakeOn &&
    now - brakeStartedAt >= MAX_BRAKE_MS
  ) {
    brakeOn = false;
    brakeLockedOut = true;
    brakeCandidate = false;
    releaseTiming = false;

    Serial.println(
      "BRAKE: five-second limit reached."
    );
  }

  if (brakeLockedOut) {
    if (
      usable &&
      forwardAccel >= -BRAKE_RELEASE_MPS2
    ) {
      if (!releaseTiming) {
        releaseTiming = true;
        brakeReleasedAt = now;
      }

      if (
        now - brakeReleasedAt >= BRAKE_REARM_MS
      ) {
        brakeLockedOut = false;
        releaseTiming = false;

        Serial.println(
          "BRAKE: rearmed for a new slowdown."
        );
      }
    } else {
      releaseTiming = false;
    }

    return;
  }

  if (
    usable &&
    forwardAccel < -BRAKE_ON_MPS2
  ) {
    if (!brakeCandidate) {
      brakeCandidate = true;
      brakeCandidateAt = now;
    }

    if (
      !brakeOn &&
      now - brakeCandidateAt >= BRAKE_CONFIRM_MS
    ) {
      brakeOn = true;
      brakeStartedAt = now;
      brakeSupportedAt = now;
    }
  } else {
    brakeCandidate = false;
  }

  if (
    brakeOn &&
    usable &&
    forwardAccel < -BRAKE_RELEASE_MPS2
  ) {
    brakeSupportedAt = now;
  }

  if (
    brakeOn &&
    now - brakeSupportedAt >= BRAKE_HOLD_MS
  ) {
    brakeOn = false;
  }
}

bool brakeLedShouldBeOn(uint32_t now) {
  if (!brakeOn) return false;

  uint32_t elapsed = now - brakeStartedAt;

  if (elapsed >= MAX_BRAKE_MS) return false;

  return (elapsed / BRAKE_FLASH_HALF_MS) % 2 == 0;
}

// ================= IMU PROCESSING =================

void updateBrake(uint32_t now) {
  if (!imuReady) return;

  float a[3];
  float w[3];

  if (!readMotion(a, w)) {
    brakeOn = false;
    brakeCandidate = false;

    consecutiveImuFailures++;

    Serial.printf(
      "IMU read failure %u/5\n",
      consecutiveImuFailures
    );

    if (consecutiveImuFailures >= 5) {
      imuReady = false;

      Serial.println(
        "IMU DISABLED: check wiring, keep still and RESET."
      );
    }

    return;
  }

  consecutiveImuFailures = 0;

  for (int i = 0; i < 3; i++) {
    latestAccel[i] = a[i];
    latestGyro[i] = w[i];
  }

  uint32_t us = micros();
  float dt = (us - imuUs) * 0.000001f;
  imuUs = us;

  if (dt <= 0 || dt > 0.2f) {
    dt = 0.02f;
  }

  for (int i = 0; i < 3; i++) {
    w[i] -= gyroBias[i];
  }

  float x = gravityDir[0];
  float y = gravityDir[1];
  float z = gravityDir[2];

  gravityDir[0] += (y * w[2] - z * w[1]) * dt;
  gravityDir[1] += (z * w[0] - x * w[2]) * dt;
  gravityDir[2] += (x * w[1] - y * w[0]) * dt;

  normalizeGravity();

  float magnitude = length3(a);
  float rotation = length3(w);

  if (
    fabsf(magnitude - G) < 0.08f * G &&
    rotation < 0.5f
  ) {
    float blend = 1.0f - expf(-dt / 8.0f);

    for (int i = 0; i < 3; i++) {
      gravityDir[i] =
        (1 - blend) * gravityDir[i] +
        blend * a[i] / magnitude;
    }

    normalizeGravity();
  }

  float raw =
    FORWARD_SIGN *
    (
      a[FORWARD_AXIS] -
      G * gravityDir[FORWARD_AXIS]
    );

  forwardAccel +=
    (1.0f - expf(-dt / ACCEL_FILTER_SECONDS)) *
    (raw - forwardAccel);

  bool usable =
    magnitude > 0.5f * G &&
    magnitude < 1.5f * G &&
    rotation < 1.2f;

  updateBrakeState(now, usable);
}

// ================= ULTRASONIC =================

void measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    25000UL
  );

  float reading = duration * 0.0343f / 2.0f;

  bool valid =
    duration != 0 &&
    reading >= 2.0f &&
    reading <= 400.0f;

  if (valid) {
    distanceCm =
      hasDistance && distanceFresh
        ? 0.5f * (distanceCm + reading)
        : reading;

    hasDistance = true;
    lastValidDistanceAt = millis();
  }

  // Preserve the last distance if an echo is missed.
  distanceFresh = valid;
}

// ================= STARTUP =================

void setup() {
  pinMode(LEFT_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);
  pinMode(BRAKE_LED, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LEFT_LED, LOW);
  digitalWrite(RIGHT_LED, LOW);
  digitalWrite(BRAKE_LED, LOW);

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println(
    "REAR v8: IMMEDIATE BLINK UPDATES + FIVE SECOND BRAKE LIMIT"
  );

  Serial.printf(
    "Reset reason: %d\n",
    static_cast<int>(esp_reset_reason())
  );

  imuReady = startIMU();

  Serial.println(
    imuReady
      ? "IMU CALIBRATED"
      : "IMU FAILED: inspect startup errors."
  );

  radioReady = startRadio();

  Serial.println(
    radioReady
      ? "ESP-NOW ready on channel 1"
      : "RADIO FAILED: press RESET"
  );
}

// ================= MAIN LOOP =================

void loop() {
  uint32_t now = millis();
  uint32_t packetTime = 0;

  Packet incoming;

  if (takePacket(incoming, packetTime)) {
    requestedTurn = incoming.turn;
    lastFrontAt = packetTime;
    frontSeen = true;
  }

  // Keep this AFTER taking the packet.
  // Prevents false disconnections from timestamp underflow.
  now = millis();

  bool linked =
    frontSeen &&
    now - lastFrontAt < LINK_TIMEOUT_MS;

  uint8_t activeTurn =
    linked ? requestedTurn : 0;

  if (now - lastImu >= 20) {
    lastImu = now;
    updateBrake(now);
  }

  static bool previousBrake = false;

  if (brakeOn != previousBrake) {
    Serial.printf(
      "BRAKE %s | Forward: %.3f m/s2\n",
      brakeOn ? "ON" : "OFF",
      forwardAccel
    );

    previousBrake = brakeOn;
  }

  if (now - lastPing >= 100) {
    lastPing = now;
    measureDistance();
  }

  // Turn signals: 400 ms ON, 400 ms OFF.
  bool flash = (now / 400) % 2;

  digitalWrite(
    LEFT_LED,
    activeTurn == 1 && flash
  );

  digitalWrite(
    RIGHT_LED,
    activeTurn == 2 && flash
  );

  digitalWrite(
    BRAKE_LED,
    brakeLedShouldBeOn(millis())
  );

  static bool sentFlash = false;
  static bool sentBrake = false;
  static uint8_t sentTurn = 255;

  bool changed =
    flash != sentFlash ||
    activeTurn != sentTurn ||
    brakeOn != sentBrake;

  // Announce changes immediately, plus an 80 ms heartbeat.
  if (changed || now - lastSend >= 80) {
    lastSend = now;

    Packet p = {};

    p.source = 2;
    p.turn = activeTurn;
    p.brake = brakeOn;
    p.distanceOK = distanceFresh;
    p.imuOK = imuReady;
    p.blink = flash;
    p.distanceCm = hasDistance ? distanceCm : 0;
    p.forwardAccel = forwardAccel;

    sendPacket(p);

    sentFlash = flash;
    sentBrake = brakeOn;
    sentTurn = activeTurn;
  }

  if (now - lastLog >= 250) {
    lastLog = now;

    Serial.printf(
      "Uptime:%lu s | RX age:%lu ms | "
      "Front:%s | Turn:%u | Brake:%s | "
      "IMU:%s | Forward:%.3f m/s2\n",

      static_cast<unsigned long>(millis() / 1000),

      static_cast<unsigned long>(
        frontSeen ? millis() - lastFrontAt : 0
      ),

      linked ? "LINKED" : "OFFLINE",
      activeTurn,
      brakeOn ? "ON" : "OFF",
      imuReady ? "OK" : "FAIL",
      forwardAccel
    );

    if (hasDistance) {
      Serial.printf(
        "Distance: %.2f ft [%s; age %lu ms]\n",

        distanceCm / 30.48f,

        distanceFresh ? "LIVE" : "LAST KNOWN",

        static_cast<unsigned long>(
          millis() - lastValidDistanceAt
        )
      );
    } else {
      Serial.println(
        "Distance: waiting for first echo"
      );
    }

    if (imuReady) {
      Serial.printf(
        "ACCEL[g] X:%.3f Y:%.3f Z:%.3f | "
        "GYRO[deg/s] X:%.2f Y:%.2f Z:%.2f\n",

        latestAccel[0] / G,
        latestAccel[1] / G,
        latestAccel[2] / G,

        latestGyro[0] / DEG_TO_RAD,
        latestGyro[1] / DEG_TO_RAD,
        latestGyro[2] / DEG_TO_RAD
      );
    }
  }

  delay(1);
}