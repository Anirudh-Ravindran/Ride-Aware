// ScooterGuard firmware - ESP32-S3 N16R8
// Ultrasonic 1-10 threat meter + IMU driver score, on an SSD1306 OLED, LEDs and buzzer.
// Runs fully standalone. Rides are detected from motion, checkpointed to flash, and handed to a
// carrier (phone over BLE, or a script over USB Serial) with the commands in RideLink.h.
// Serial also prints live JSON lines (events + ride metrics).
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include "ThreatMeter.h"
#include "Imu6500.h"
#include "DriveMotion.h"
#include "SafetyScore.h"
#include "RideLog.h"
#include "RideStore.h"
#include "RideLink.h"

// ---- Pins (ESP32-S3 N16R8: avoid 0,3,45,46 strapping, 19/20 USB, 26-37 flash/PSRAM) ----
constexpr int TRIG_PIN = 4, ECHO_PIN = 5;             // ECHO needs a 1k/2k divider (5V sensor)
constexpr int LED_GREEN = 15, LED_YELLOW = 16, LED_RED = 17, BUZZER_PIN = 18;
constexpr int BUTTON_PIN = 14;                        // to GND. Tap = switch page, hold 3 s = recalibrate
constexpr int SDA_PIN = 8, SCL_PIN = 9;               // OLED (0x3C) and IMU (0x68) share this bus

#define REPORT_EVERY_MS 10000UL
#define CHECKPOINT_MS 30000UL                         // save the ride in progress this often
#define MIN_RIDE_S 60.0f                              // less moving time than this isn't kept as a ride
#define DEBUG_PLOT 0                                  // 1 = print brake/lat/level for Serial Plotter

Adafruit_SSD1306 display(128, 64, &Wire, -1);
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
Mode mode = CAL_REST;
int page = 0, imuFails = 0, calN = 0;
Vec3 sumA = {0, 0, 0}, sumW = {0, 0, 0};
float amin = 9, amax = 0;
unsigned long lastSense = 0, lastImuUs = 0, lastDisp = 0, lastBlink = 0, lastReport = 0, lastCheckpoint = 0, btnT = 0;
bool blinkOn = false, btnDown = false, longDone = false;

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

// ---- outputs ----
void updateOutputs() {
  int lvl = meter.level();
  unsigned long now = millis(), period = lvl >= 9 ? 100 : lvl >= 7 ? 300 : 0;
  if (period && now - lastBlink >= period) { blinkOn = !blinkOn; lastBlink = now; }
  if (!period) blinkOn = true;
  digitalWrite(LED_GREEN, lvl <= 3);
  digitalWrite(LED_YELLOW, lvl >= 4 && lvl <= 6);
  digitalWrite(LED_RED, lvl >= 7 && blinkOn);
  digitalWrite(BUZZER_PIN, lvl >= 7 && blinkOn);
}

void drawThreat() {
  int lvl = meter.level();
  display.setTextSize(1); display.setCursor(0, 0); display.print(ThreatMeter::label(lvl));
  if (mode == RUN) { display.setCursor(86, 0); display.print("DS "); display.print(computeScore(safety.m).score); }
  display.setTextSize(4); display.setCursor(0, 14); display.print(lvl);
  display.setTextSize(1);
  display.setCursor(66, 16);
  if (meter.distanceCm() >= 400) display.print("-- cm"); else { display.print((int)meter.distanceCm()); display.print(" cm"); }
  display.setCursor(66, 28); display.print(meter.closingMps() > 0 ? meter.closingMps() : 0, 1); display.print(" m/s");
  display.setCursor(66, 40); display.print("TTC ");
  if (meter.ttcSec() > 0) { display.print(meter.ttcSec(), 1); display.print("s"); } else display.print("--");
  for (int i = 0; i < 10; i++) {
    int x = i * 12 + 4;
    if (i < lvl) display.fillRect(x, 54, 10, 9, SSD1306_WHITE); else display.drawRect(x, 54, 10, 9, SSD1306_WHITE);
  }
}

void drawDrive() {
  ScoreResult r = computeScore(safety.m);
  const char *lab = r.score >= 90 ? "EXCELLENT" : r.score >= 80 ? "GOOD" : r.score >= 65 ? "FAIR" : "NEEDS WORK";
  display.setTextSize(1); display.setCursor(0, 0); display.print("DRIVER SCORE");
  if (rides.active()) { display.setCursor(110, 0); display.print("REC"); }
  display.setTextSize(3); display.setCursor(0, 14); display.print(r.score);
  display.setTextSize(1); display.setCursor(66, 18); display.print(lab);
  display.setCursor(66, 30); display.printf("Ride %d:%02d", (int)safety.m.moving_s / 60, (int)safety.m.moving_s % 60);
  display.setCursor(0, 42); display.printf("BRK %.1f%%  TRN %.1f%%", r.fBrake, r.fTurn);
  display.setCursor(0, 54); display.printf("PRX %.1f%%  %.2fg/%.2fg", r.fProx, motion.brakeG(), motion.lateralG());
}

void drawMsg(const char *a, const char *b, const char *c) {
  display.setTextSize(2); display.setCursor(0, 6); display.print(a);
  display.setTextSize(1); display.setCursor(0, 32); display.print(b); display.setCursor(0, 44); display.print(c);
}

void updateDisplay() {
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  if (meter.level() >= 7) drawThreat();                   // danger always wins the screen
  else if (mode == CAL_REST) { drawMsg("HOLD STILL", "Calibrating sensor...", "Keep scooter upright"); display.fillRect(0, 58, calN * 128 / 200, 5, SSD1306_WHITE); }
  else if (mode == CAL_PUSH) drawMsg("PUSH FWD", "Roll it forward gently", "(one-time setup)");
  else if (mode == IMU_FAIL) { drawThreat(); display.setCursor(86, 0); display.print("NO IMU"); }
  else if (page == 0) drawThreat();
  else drawDrive();
  display.display();
}

void handleButton() {
  bool down = digitalRead(BUTTON_PIN) == LOW;
  unsigned long now = millis();
  if (down && !btnDown) { btnDown = true; btnT = now; longDone = false; }
  if (down && !longDone && now - btnT > 3000) { longDone = true; if (rides.active()) endRide(); prefs.remove("fwd"); ESP.restart(); }
  if (!down && btnDown) { btnDown = false; if (!longDone && now - btnT > 30) page ^= 1; }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT); pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_GREEN, OUTPUT); pinMode(LED_YELLOW, OUTPUT); pinMode(LED_RED, OUTPUT); pinMode(BUZZER_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoIsr, CHANGE);
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
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
  updateOutputs(); handleButton(); pollCommands();
  if (now - lastDisp >= 100) { lastDisp = now; updateDisplay(); }
}
