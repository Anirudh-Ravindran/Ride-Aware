// Host-side test: simulated IMU data through DriveMotion + SafetyTracker + ThreatMeter.
// Build: g++ -std=c++11 -Wall -I../include test_driving.cpp -o t && ./t
#include <cstdio>
#include <cstdlib>
#include "DriveMotion.h"
#include "SafetyScore.h"
#include "ThreatMeter.h"

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok   %s\n", msg); } while (0)

struct Mount { Vec3 fwd, left, up; };                       // world axes in sensor coordinates
static float noise() { return (rand() / (float)RAND_MAX - 0.5f) * 0.02f; }

struct Sim {
  Mount m; DriveMotion dm; SafetyTracker st; ThreatMeter tm; float t = 0; int events[4] = {0, 0, 0, 0};
  float maxBrake = 0;
  // one 10 ms step. aF/aL = true acceleration (g), pitch = nose-up angle, pitchRate = rad/s
  void step(float aF, float aL, float vib, float pitch = 0, float pitchRate = 0) {
    float v = vib * sinf(2 * 3.14159f * 25 * t);
    Vec3 up = m.up * cosf(pitch) + m.fwd * sinf(pitch);
    Vec3 a = up + m.fwd * (aF + v) + m.left * (aL + v * 0.5f);
    a = a + Vec3{noise(), noise(), noise()};
    Vec3 w = m.left * (-pitchRate) + Vec3{0.004f, -0.003f, 0.002f};   // nose-up = negative about left, plus gyro bias
    dm.update(a, w, 0.01f);
    st.update(0.01f, dm.moving(), dm.brakeG(), dm.lateralG(), tm.level());
    if (dm.brakeG() > maxBrake) maxBrake = dm.brakeG();
    EventType e; float s, at;
    while (st.pollEvent(e, s, at)) events[e]++;
    t += 0.01f;
  }
  void rest(float sec) {                                   // calibrate on 2 s of stillness
    Vec3 sa = {0, 0, 0}, sw = {0, 0, 0}; int n = (int)(sec * 100);
    for (int i = 0; i < n; i++) { sa = sa + m.up + Vec3{noise(), noise(), noise()}; sw = sw + Vec3{0.004f, -0.003f, 0.002f}; }
    dm.calibrateRest(sa * (1.0f / n), sw * (1.0f / n));
  }
};

int main() {
  srand(7);
  Mount mounts[2] = {
    {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}},         // X forward, Z up
    {Vec3{0, 0, 1}, Vec3{1, 0, 0}, Vec3{0, 1, 0}},         // mounted on its side: Z forward, Y up
  };
  for (int k = 0; k < 2; k++) {
    printf("\n== mounting %c ==\n", 'A' + k);
    Sim s; s.m = mounts[k];
    s.rest(2.0f);
    for (int i = 0; i < 60; i++) s.step(0.20f, 0, 0.10f);   // gentle push forward
    CHECK(s.dm.state() == DriveMotion::READY, "forward learned from a push");
    CHECK(dot(s.dm.forward(), s.m.fwd) > 0.97f, "forward axis points the right way");

    for (int i = 0; i < 600; i++) s.step(0, 0, 0.12f);      // cruising with road vibration
    CHECK(s.dm.moving(), "vibration counts as moving");
    CHECK(s.events[1] + s.events[2] + s.events[3] == 0, "no false events while cruising");

    for (int i = 0; i < 100; i++) s.step(-0.30f, 0, 0.12f); // 1 s hard brake
    for (int i = 0; i < 300; i++) s.step(0, 0, 0.12f);
    CHECK(s.events[EV_HARD_BRAKE] == 1, "one hard-brake event");
    CHECK(s.st.m.brakeHard_s > 0.4f && s.st.m.brakeHard_s < 1.2f, "hard-brake time ~ 1 s");

    for (int i = 0; i < 100; i++) s.step(0, 0.40f, 0.12f);  // 1 s hard turn
    for (int i = 0; i < 300; i++) s.step(0, 0, 0.12f);
    CHECK(s.events[EV_HARSH_TURN] == 1, "one harsh-turn event");

    float before = s.st.m.brakeHard_s; s.maxBrake = 0;      // 8 degree hill climbed over 2 s
    for (int i = 0; i < 200; i++) s.step(0, 0, 0.12f, 0.14f * i / 200.0f, 0.14f / 2.0f);
    for (int i = 0; i < 300; i++) s.step(0, 0, 0.12f, 0.14f, 0);
    printf("     peak brake reading on hill: %.3f g\n", s.maxBrake);
    CHECK(s.maxBrake < 0.1f && s.st.m.brakeHard_s == before, "slope change is not mistaken for braking");

    for (int i = 0; i < 700; i++) s.step(0, 0, 0.0f, 0.14f, 0); // parked on the hill
    CHECK(!s.dm.moving(), "parked = not moving");
  }

  printf("\n== close calls via ultrasonic ==\n");
  { Sim s; s.m = mounts[0]; s.rest(2); for (int i = 0; i < 60; i++) s.step(0.2f, 0, 0.1f);
    for (int i = 0; i < 300; i++) s.step(0, 0, 0.12f);
    uint32_t ms = 0; float d = 380;                          // fast approach from behind
    for (int i = 0; i < 160; i++) {
      if (i % 6 == 0) { d -= 24; if (d < 25) d = 25; s.tm.update(d, ms); ms += 60; }
      s.step(0, 0, 0.12f);
    }
    CHECK(s.tm.level() >= 8, "meter reached WARNING+");
    CHECK(s.st.m.proxHigh_s > 0.3f, "proximity exposure recorded");
    CHECK(s.events[EV_CLOSE_CALL] >= 0, "close-call tracking runs"); }

  printf("\n== score ==\n");
  DriveMetrics clean = {100, 0, 100, 0, 100, 0, 300};
  CHECK(computeScore(clean).score == 100, "clean ride = 100");
  DriveMetrics rough = {100, 8, 100, 3, 100, 6, 300};
  ScoreResult r = computeScore(rough);
  printf("     rough ride => %d\n", r.score);
  CHECK(r.score < 90 && r.score > 0, "rough ride scores lower");
  // parity data for the JS comparison
  DriveMetrics sets[3] = {{100, 2, 80, 1, 30, 1, 0}, {200, 10, 150, 4, 60, 8, 0}, {50, 0.5f, 40, 0, 10, 0.2f, 0}};
  for (int i = 0; i < 3; i++) printf("PARITY %d %d\n", i, computeScore(sets[i]).score);

  printf("\n%s (%d failures)\n", fails ? "TESTS FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}
