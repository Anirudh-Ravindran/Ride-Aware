// SafetyScore.h - on-device driver score. MUST stay identical to score.js on the server:
//   PCF   = 0.682854 x 1.127294^hardBrake% x 1.019630^turn% x 1.03^prox%
//   Score = 115.382324 - 22.526504 x PCF   (0..100)
// Each factor % = time past the HARD threshold / (time past the BASE threshold + prior) x 100.
// The prior (20 s) stops a few seconds of data from swinging the score wildly.
#pragma once
#include <math.h>
#include <stdint.h>

namespace SafetyCfg {
constexpr float kBase = 0.682854f, kHardBrake = 1.127294f, kTurn = 1.019630f, kProx = 1.03f;
constexpr float kA = 115.382324f, kB = 22.526504f, kPriorS = 20.0f;
constexpr float kBrakeBase = 0.10f, kBrakeHard = 0.25f;   // g
constexpr float kTurnBase = 0.15f, kTurnHard = 0.30f;     // g
constexpr int kProxBase = 4, kProxHigh = 7;               // ThreatMeter levels
}

struct DriveMetrics { float brakeBase_s, brakeHard_s, turnBase_s, turnHard_s, proxBase_s, proxHigh_s, moving_s; };
struct ScoreResult { int score; float pcf, fBrake, fTurn, fProx; };

inline float clamp100(float x) { return x < 0 ? 0 : (x > 100 ? 100 : x); }

inline ScoreResult computeScore(const DriveMetrics &m) {
  using namespace SafetyCfg;
  ScoreResult r;
  r.fBrake = 100.0f * m.brakeHard_s / (m.brakeBase_s + kPriorS);
  r.fTurn = 100.0f * m.turnHard_s / (m.turnBase_s + kPriorS);
  r.fProx = 100.0f * m.proxHigh_s / (m.proxBase_s + kPriorS);
  r.pcf = kBase * powf(kHardBrake, r.fBrake) * powf(kTurn, r.fTurn) * powf(kProx, r.fProx);
  r.score = (int)floorf(clamp100(kA - kB * r.pcf) + 0.5f);
  return r;
}

enum EventType { EV_NONE, EV_HARD_BRAKE, EV_HARSH_TURN, EV_CLOSE_CALL };
inline const char *eventName(EventType t) {
  return t == EV_HARD_BRAKE ? "hard_brake" : t == EV_HARSH_TURN ? "harsh_turn" : t == EV_CLOSE_CALL ? "close_call" : "none";
}

// Accumulates the metrics and detects discrete events (one per episode, with a severity 0.3..1).
class SafetyTracker {
 public:
  DriveMetrics m;
  SafetyTracker() { reset(); }
  void reset() {
    m = DriveMetrics{0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 3; i++) { act_[i] = false; peak_[i] = 0; }
    qh_ = qt_ = 0;
  }

  void update(float dt, bool moving, float brakeG, float latG, int level) {
    using namespace SafetyCfg;
    if (!moving) { brakeG = latG = 0; level = 0; }
    else m.moving_s += dt;
    if (brakeG > kBrakeBase) m.brakeBase_s += dt;
    if (brakeG > kBrakeHard) m.brakeHard_s += dt;
    if (latG > kTurnBase) m.turnBase_s += dt;
    if (latG > kTurnHard) m.turnHard_s += dt;
    if (level >= kProxBase) m.proxBase_s += dt;
    if (level >= kProxHigh) m.proxHigh_s += dt;
    track(0, brakeG, kBrakeHard, kBrakeHard * 0.8f, 0.25f, EV_HARD_BRAKE);
    track(1, latG, kTurnHard, kTurnHard * 0.8f, 0.25f, EV_HARSH_TURN);
    track(2, (float)level, (float)kProxHigh, (float)kProxHigh - 1, 3.0f, EV_CLOSE_CALL);
  }

  bool pollEvent(EventType &t, float &severity, float &atSec) {
    if (qh_ == qt_) return false;
    t = qt_Type_[qh_]; severity = qSev_[qh_]; atSec = qAt_[qh_]; qh_ = (qh_ + 1) % kQ;
    return true;
  }

 private:
  static constexpr int kQ = 4;
  bool act_[3]; float peak_[3];
  EventType qt_Type_[kQ]; float qSev_[kQ], qAt_[kQ]; int qh_, qt_;

  void track(int i, float v, float start, float stop, float span, EventType type) {
    if (!act_[i] && v > start) { act_[i] = true; peak_[i] = v; }
    else if (act_[i]) {
      if (v > peak_[i]) peak_[i] = v;
      if (v < stop) {
        act_[i] = false;
        float s = 0.3f + 0.7f * (peak_[i] - start) / span;
        s = s < 0.3f ? 0.3f : (s > 1 ? 1 : s);
        int next = (qt_ + 1) % kQ;
        if (next != qh_) { qt_Type_[qt_] = type; qSev_[qt_] = s; qAt_[qt_] = m.moving_s; qt_ = next; }
      }
    }
  }
};
