// ScooterGuard firmware - rear unit, ESP32-S3 N16R8
// Ultrasonic 1-10 threat meter + IMU driver score, with a brake light and left/right indicators.
// The brake light comes on when the IMU sees braking and flashes when a car closes in from behind.
// Indicators are switched by the SIG command (RideLink.h). Runs fully standalone. Rides are detected from motion, checkpointed to flash, and handed to a
// carrier (phone over BLE, or a script over USB Serial) with the commands in RideLink.h.
// Serial also prints live JSON lines (events + ride metrics).
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "ThreatMeter.h"
#include "Imu6500.h"
#include "DriveMotion.h"
#include "SafetyScore.h"
#include "RideLog.h"
#include "RideStore.h"
#include "RideLink.h"

// ---- Pins (ESP32-S3 N16R8: avoid 0,3,45,46 strapping, 19/20 USB, 26-37 flash/PSRAM) ----
constexpr int TRIG_PIN = 6, ECHO_PIN = 7;             // ECHO needs a voltage divider (5V sensor)
constexpr int LED_LEFT = 4, LED_RIGHT = 5, LED_BRAKE = 8;   // each through 330R to GND
constexpr int SDA_PIN = 15, SCL_PIN = 16;             // IMU (0x68 or 0x69)

#define REPORT_EVERY_MS 10000UL
#define CHECKPOINT_MS 30000UL                         // save the ride in progress this often
#define MIN_RIDE_S 60.0f                              // less moving time than this isn't kept as a ride
#define DEBUG_PLOT 0                                  // 1 = print brake/lat/level for Serial Plotter
#define BRAKE_ON_G 0.10f                              // brake light on above this deceleration...
#define BRAKE_OFF_G 0.06f                             // ...off below this
#define BRAKE_HOLD_MS 500UL                           // and stays on at least this long
#define BLINK_MS 400UL                                // indicator on/off time (~1.25 Hz)

Imu6500 imu;
DriveMotion motion;
SafetyTracker safety;
ThreatMeter meter;
Preferences prefs;
RideDetector rides;
Ride ride;                                            // the ride being recorded, while rides.active()
RideStore store;
RideLink ble;
char rideJson[Ride::kMaxJson];
double epochAtBoot = 0;                               // set by the TIME command; 0 = clock unknown
String serialLine;

enum Mode { CAL_REST, CAL_PUSH, RUN, IMU_FAIL };
Mode mode = CAL_REST, shownMode = IMU_FAIL;           // shownMode != mode makes the first status print
enum Indicator { IND_OFF, IND_LEFT, IND_RIGHT, IND_HAZARD };
Indicator indicator = IND_OFF;
int imuFails = 0, calN = 0;
Vec3 sumA = {0, 0, 0}, sumW = {0, 0, 0};
float amin = 9, amax = 0;
unsigned long lastSense = 0, lastImuUs = 0, lastBlink = 0, lastReport = 0, lastCheckpoint = 0, brakeT = 0, restartAt = 0;
bool blinkOn = false, braking = false;

// ---- non-blocking ultrasonic (interrupt on echo edges, so the 100 Hz IMU loop never stalls) ----
volatile uint32_t echoRise = 0, echoUs = 0;
volatile bool echoDone = false;
void IRAM_ATTR echoIsr() {
  if (digitalRead(ECHO_PIN)) echoRise = micros();
  else { echoUs = micros() - echoRise; echoDone = true; }
}
float takeEchoCm() {                                  // -1 = no echo since last trigger
  if (!echoDone) return -1;
  echoDone = false;
  float cm = echoUs * 0.0343f / 2.0f;
  return cm > 400 ? -1 : cm;
}
void fireTrigger() {
  echoDone = false;
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN, LOW);
}

void rideTick(unsigned long now);

// ---- calibration ----
void restartRest() { sumA = sumW = Vec3{0, 0, 0}; calN = 0; amin = 9; amax = 0; }

void stepCalRest(Vec3 a, Vec3 w) {
  float m = norm(a);
  if (m < amin) amin = m;
  if (m > amax) amax = m;
  if (amax - amin > 0.06f) { restartRest(); return; }     // it moved: start over
  sumA = sumA + a; sumW = sumW + w;
  if (++calN < 200) return;                               // 2 s of stillness
  motion.calibrateRest(sumA * (1.0f / calN), sumW * (1.0f / calN));
  float f[3];
  if (prefs.getBytes("fwd", f, sizeof(f)) == sizeof(f) && motion.setForward(Vec3{f[0], f[1], f[2]})) mode = RUN;
  else mode = CAL_PUSH;                                   // first time: learn forward from a push
}

void imuTick(float dt) {
  Vec3 a, w;
  if (!imu.read(a, w)) { if (++imuFails > 50) mode = IMU_FAIL; return; }
  imuFails = 0;
  if (mode == CAL_REST) stepCalRest(a, w);
  else if (mode == CAL_PUSH) {
    motion.update(a, w, dt);
    if (motion.state() == DriveMotion::READY) {
      Vec3 f = motion.forward(); float arr[3] = {f.x, f.y, f.z};
      prefs.putBytes("fwd", arr, sizeof(arr));
      mode = RUN;
    }
  } else if (mode == RUN) {
    motion.update(a, w, dt);
    safety.update(dt, motion.moving(), motion.brakeG(), motion.lateralG(), meter.level());
    EventType et; float sev, at;
    while (safety.pollEvent(et, sev, at)) {
      Serial.printf("{\"event\":\"%s\",\"t\":%.1f,\"severity\":%.2f}\n", eventName(et), at, sev);
      if (rides.active()) ride.addEvent(et, sev, (millis() - ride.startMs) / 1000.0f);
    }
    rideTick(millis());
  }
}

// ---- rides: detect, checkpoint to flash, hand off to the carrier ----
void saveRide() {
  ride.m = safety.m;
  if (epochAtBoot > 0) ride.startEpoch = epochAtBoot + ride.startMs / 1000.0;
  int n = ride.toJson(rideJson, sizeof(rideJson));
  if (n < 0 || !store.save(ride.id, rideJson, n)) Serial.println("{\"error\":\"ride save failed\"}");
  lastCheckpoint = millis();
}

void endRide() {
  bool keep = safety.m.moving_s >= MIN_RIDE_S;          // checkpoints only start past MIN_RIDE_S, so
  if (keep) saveRide();                                 // a short one never reached flash
  Serial.printf("{\"ride\":\"end\",\"id\":%u,\"saved\":%s}\n", (unsigned)ride.id, keep ? "true" : "false");
}

void rideTick(unsigned long now) {
  switch (rides.update(motion.moving(), now)) {
    case RideDetector::STARTED:
      safety.reset();                                   // metrics and the on-screen score are per ride
      ride.begin(prefs.getUInt("rideId", 1), rides.startMs());
      prefs.putUInt("rideId", ride.id + 1);
      lastCheckpoint = now;
      Serial.printf("{\"ride\":\"start\",\"id\":%u}\n", (unsigned)ride.id);
      break;
    case RideDetector::ENDED:
      endRide();
      break;
    default:
      if (rides.active() && now - lastCheckpoint >= CHECKPOINT_MS && safety.m.moving_s >= MIN_RIDE_S) saveRide();
  }
}

// One command line from USB Serial or BLE; the reply goes back the same way. Protocol: RideLink.h.
void handleCommand(String line, bool fromBle) {
  line.trim();
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp), arg = sp < 0 ? "" : line.substring(sp + 1);
  cmd.toUpperCase();
  uint32_t rec = rides.active() ? ride.id : 0, id = arg.toInt();
  String out;
  if (cmd == "LIST") {
    out = "{\"resp\":\"list\",\"recording\":" + String(rec) + ",\"rides\":[";
    bool first = true;
    store.forEach([&](uint32_t rid, size_t bytes) {
      if (rid == rec) return;
      out += (first ? "{\"id\":" : ",{\"id\":") + String(rid) + ",\"bytes\":" + String(bytes) + "}";
      first = false;
    });
    out += "]}";
  } else if (cmd == "GET") {
    String body;
    if (store.read(id, body)) out = "{\"resp\":\"ride\",\"id\":" + String(id) + ",\"ride\":" + body + "}";
    else out = "{\"resp\":\"ride\",\"id\":" + String(id) + ",\"error\":\"not found\"}";
  } else if (cmd == "DEL") {
    bool ok = id && id != rec && store.remove(id);
    out = "{\"resp\":\"del\",\"id\":" + String(id) + ",\"ok\":" + (ok ? "true" : "false") + "}";
  } else if (cmd == "TIME") {
    double t = arg.toDouble();
    bool ok = t > 1.6e9;
    if (ok) epochAtBoot = t - millis() / 1000.0;
    out = String("{\"resp\":\"time\",\"ok\":") + (ok ? "true" : "false") + "}";
  } else if (cmd == "SIG") {
    arg.toUpperCase();
    bool ok = true;
    if (arg == "L") indicator = IND_LEFT;
    else if (arg == "R") indicator = IND_RIGHT;
    else if (arg == "H") indicator = IND_HAZARD;
    else if (arg == "OFF") indicator = IND_OFF;
    else ok = false;
    out = String("{\"resp\":\"sig\",\"ok\":") + (ok ? "true" : "false") + "}";
  } else if (cmd == "RECAL") {                          // forget the forward axis and relearn it
    if (rides.active()) endRide();
    prefs.remove("fwd");
    restartAt = millis() + 500;                         // give the reply time to go out
    out = "{\"resp\":\"recal\",\"ok\":true}";
  } else {
    out = "{\"resp\":\"error\",\"error\":\"unknown command\"}";
  }
  out += '\n';
  if (fromBle) ble.send(out); else Serial.print(out);
}

void pollCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (serialLine.length() < 64) serialLine += c; continue; }
    if (serialLine.length()) handleCommand(serialLine, false);
    serialLine = "";
  }
  String line;
  while (ble.readLine(line)) handleCommand(line, true);
  ble.pump();
}

void report() {
  const DriveMetrics &m = safety.m;
  ScoreResult r = computeScore(m);
  Serial.printf("{\"durationS\":%.0f,\"score\":%d,\"metrics\":{\"brakeBase_s\":%.2f,\"brakeHard_s\":%.2f,"
                "\"turnBase_s\":%.2f,\"turnHard_s\":%.2f,\"proxBase_s\":%.2f,\"proxHigh_s\":%.2f}}\n",
                m.moving_s, r.score, m.brakeBase_s, m.brakeHard_s, m.turnBase_s, m.turnHard_s, m.proxBase_s, m.proxHigh_s);
}

// ---- lights ----
const char *modeName(Mode m) { return m == CAL_REST ? "cal_rest" : m == CAL_PUSH ? "cal_push" : m == RUN ? "run" : "imu_fail"; }

void updateLights() {
  unsigned long now = millis();
  if (now - lastBlink >= BLINK_MS) { blinkOn = !blinkOn; lastBlink = now; }
  if (mode != shownMode) { shownMode = mode; Serial.printf("{\"status\":\"%s\"}\n", modeName(mode)); }

  // Indicators. With no screen, calibration shows here too:
  // both solid = hold still, both blinking = push it forward (one-time), alternating = no IMU.
  bool left, right;
  if (mode == CAL_REST) left = right = true;
  else if (mode == CAL_PUSH) left = right = blinkOn;
  else if (mode == IMU_FAIL) { left = blinkOn; right = !blinkOn; }
  else {
    left = (indicator == IND_LEFT || indicator == IND_HAZARD) && blinkOn;
    right = (indicator == IND_RIGHT || indicator == IND_HAZARD) && blinkOn;
  }
  digitalWrite(LED_LEFT, left);
  digitalWrite(LED_RIGHT, right);

  // Brake light: on while braking, flashing when a car is closing in (threat 7+), faster at 9+.
  float g = mode == RUN ? motion.brakeG() : 0;
  if (g >= BRAKE_ON_G) { braking = true; brakeT = now; }
  else if (braking && g < BRAKE_OFF_G && now - brakeT >= BRAKE_HOLD_MS) braking = false;
  int lvl = meter.level();
  unsigned long period = lvl >= 9 ? 100 : lvl >= 7 ? 250 : 0;
  digitalWrite(LED_BRAKE, period ? (now / period) % 2 == 0 : braking);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(LED_LEFT, OUTPUT); pinMode(LED_RIGHT, OUTPUT); pinMode(LED_BRAKE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoIsr, CHANGE);
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000);
  prefs.begin("sg", false);
  if (!store.begin()) Serial.println("{\"error\":\"flash mount failed\"}");
  ble.begin("ScooterGuard");
  if (!imu.begin()) mode = IMU_FAIL;                      // meter still works without the IMU
  restartRest();
  lastImuUs = micros();
}

void loop() {
  unsigned long now = millis(), nowUs = micros();
  if (nowUs - lastImuUs >= 10000) {                       // IMU at 100 Hz
    float dt = (nowUs - lastImuUs) / 1e6f;
    if (dt > 0.03f) dt = 0.03f;
    lastImuUs = nowUs;
    if (mode != IMU_FAIL) imuTick(dt);
#if DEBUG_PLOT
    static int n = 0;
    if (++n % 10 == 0) Serial.printf("brake:%.2f,lat:%.2f,level:%d\n", motion.brakeG(), motion.lateralG(), meter.level());
#endif
  }
  if (now - lastSense >= 60) {                            // ultrasonic at ~16 Hz
    lastSense = now;
    meter.update(takeEchoCm(), now);
    fireTrigger();
  }
  if (mode == RUN && now - lastReport >= REPORT_EVERY_MS) { lastReport = now; report(); }
  updateLights(); pollCommands();
  if (restartAt && (long)(now - restartAt) >= 0) ESP.restart();
}
