// ThreatMeter.h - converts ultrasonic readings into a 1-10 threat level.
//   1  = nothing nearby / very far
//   10 = crisis (object very close and/or closing fast)
// Pure C++ (no Arduino calls) so it can be unit-tested on a PC.
//
// How it works:
//   1. Dropout hold + 5-sample median filter clean the raw distance.
//   2. Closing speed comes from the change in filtered distance (smoothed).
//   3. Distance score (0..1): log scale, so resolution is finest when close.
//   4. Time-to-collision score (0..1): distance / closing speed.
//   5. Risk = 1 - (1 - distScore)(1 - ttcScore), so either factor can raise it.
//   6. Level = 1 + 9 * risk, with fast rise / slow fall so it doesn't flicker.
#pragma once
#include <stdint.h>
#include <math.h>

class ThreatMeter {
 public:
  struct Config {
    float farCm = 400.0f;        // at/beyond: distance score 0
    float nearCm = 30.0f;        // at/below: distance score 1
    float ttcFarS = 4.0f;        // TTC at/above: score 0
    float ttcNearS = 0.8f;       // TTC at/below: score 1
    float minClosingMps = 0.3f;  // ignore slower drift
    float attack = 0.6f;         // 0..1, how fast the meter rises per update
    float fallPerSec = 3.0f;     // levels per second the meter may fall
    uint8_t dropoutHold = 5;     // samples to keep last distance on no-echo
  };

  ThreatMeter() { reset(); }
  explicit ThreatMeter(const Config &c) : cfg_(c) { reset(); }

  void reset() {
    for (int i = 0; i < kWin; i++) buf_[i] = cfg_.farCm;
    idx_ = 0; filt_ = prev_ = lastGood_ = cfg_.farCm;
    closing_ = 0; ttc_ = -1; value_ = 1; shown_ = 1;
    lastMs_ = 0; started_ = false; dropRun_ = 0;
  }

  // rawCm < 0 means "no echo". Call every sensor sample.
  void update(float rawCm, uint32_t nowMs) {
    float d;
    if (rawCm < 0) {
      dropRun_++;
      d = (dropRun_ <= cfg_.dropoutHold) ? lastGood_ : cfg_.farCm;
    } else {
      dropRun_ = 0;
      d = rawCm > cfg_.farCm ? cfg_.farCm : rawCm;
      lastGood_ = d;
    }
    buf_[idx_] = d;
    idx_ = (idx_ + 1) % kWin;
    filt_ = median5();

    float dt = 0;
    if (!started_) {
      started_ = true; prev_ = filt_;
    } else {
      dt = (nowMs - lastMs_) / 1000.0f;
      if (dt < 0.005f) return;  // duplicate call
      float inst = ((prev_ - filt_) / 100.0f) / dt;
      closing_ = 0.7f * closing_ + 0.3f * inst;
      prev_ = filt_;
    }
    lastMs_ = nowMs;
    ttc_ = closing_ > cfg_.minClosingMps ? (filt_ / 100.0f) / closing_ : -1;

    float target = rawLevel(filt_, closing_, cfg_);
    if (target > value_) value_ += (target - value_) * cfg_.attack;
    else { float drop = cfg_.fallPerSec * dt; value_ -= (value_ - target < drop) ? (value_ - target) : drop; }

    // Hysteresis on the integer shown to the rider
    if (value_ > shown_ + 0.6f || value_ < shown_ - 0.6f) shown_ = (int)(value_ + 0.5f);
    if (shown_ < 1) shown_ = 1;
    if (shown_ > 10) shown_ = 10;
  }

  // Stateless model: level for a given distance and closing speed. Handy for tuning.
  static float rawLevel(float distCm, float closingMps) {
    return rawLevel(distCm, closingMps, Config());
  }
  static float rawLevel(float distCm, float closingMps, const Config &c) {
    float ds = distScore(distCm, c);
    float ts = 0;
    if (closingMps > c.minClosingMps) ts = ttcScore((distCm / 100.0f) / closingMps, c);
    float risk = 1.0f - (1.0f - ds) * (1.0f - ts);
    return 1.0f + 9.0f * risk;
  }

  int level() const { return shown_; }          // 1..10 for display and alerts
  float value() const { return value_; }        // smooth 1.0..10.0
  float distanceCm() const { return filt_; }
  float closingMps() const { return closing_; }
  float ttcSec() const { return ttc_; }         // -1 if not closing

  static const char *label(int lvl) {
    if (lvl <= 2) return "CLEAR";
    if (lvl <= 4) return "AWARE";
    if (lvl <= 6) return "CAUTION";
    if (lvl <= 8) return "WARNING";
    return "CRISIS";
  }

 private:
  static constexpr int kWin = 5;
  Config cfg_;
  float buf_[kWin];
  int idx_;
  float filt_, prev_, lastGood_, closing_, ttc_, value_;
  int shown_;
  uint32_t lastMs_;
  bool started_;
  uint8_t dropRun_;

  static float distScore(float d, const Config &c) {
    if (d >= c.farCm) return 0;
    if (d <= c.nearCm) return 1;
    return logf(c.farCm / d) / logf(c.farCm / c.nearCm);
  }
  static float ttcScore(float t, const Config &c) {
    if (t >= c.ttcFarS) return 0;
    if (t <= c.ttcNearS) return 1;
    return (c.ttcFarS - t) / (c.ttcFarS - c.ttcNearS);
  }
  float median5() const {
    float t[kWin];
    for (int i = 0; i < kWin; i++) t[i] = buf_[i];
    for (int i = 1; i < kWin; i++) {
      float k = t[i]; int j = i - 1;
      while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
      t[j + 1] = k;
    }
    return t[kWin / 2];
  }
};
