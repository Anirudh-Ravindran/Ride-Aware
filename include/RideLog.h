// RideLog.h - ride start/end detection and the ride record that is saved to flash and uploaded.
// Pure C++ (host-testable). Ride::toJson() writes the POST /api/rides body.
#pragma once
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "SafetyScore.h"

// A ride starts after kStartMs of continuous movement and ends after kEndMs with none
// (long enough to sit at a red light). moving() already bridges short pauses.
class RideDetector {
 public:
  enum Edge { NONE, STARTED, ENDED };
  static constexpr uint32_t kStartMs = 5000, kEndMs = 120000;

  RideDetector() { reset(); }
  void reset() { active_ = wasMoving_ = false; since_ = lastMove_ = start_ = 0; }

  Edge update(bool moving, uint32_t nowMs) {
    if (moving) {
      if (!wasMoving_) { wasMoving_ = true; since_ = nowMs; }
      lastMove_ = nowMs;
      if (!active_ && nowMs - since_ >= kStartMs) { active_ = true; start_ = since_; return STARTED; }
    } else {
      wasMoving_ = false;
      if (active_ && nowMs - lastMove_ >= kEndMs) { active_ = false; return ENDED; }
    }
    return NONE;
  }
  bool active() const { return active_; }
  uint32_t startMs() const { return start_; }     // when the movement that started the ride began

 private:
  bool active_, wasMoving_;
  uint32_t since_, lastMove_, start_;
};

struct RideEvent { EventType type; float severity, t; };

struct Ride {
  static constexpr int kMaxEvents = 200;          // later events are dropped (metrics still count them)
  static constexpr int kMaxJson = 12288;          // toJson() worst case at kMaxEvents fits in this
  uint32_t id, startMs;
  double startEpoch;                              // unix seconds; 0 = clock not set (server uses upload time)
  DriveMetrics m;
  RideEvent ev[kMaxEvents];
  int nEvents;

  void begin(uint32_t rideId, uint32_t atMs) {
    id = rideId; startMs = atMs; startEpoch = 0; m = DriveMetrics{0, 0, 0, 0, 0, 0, 0}; nEvents = 0;
  }
  // t = seconds since the ride started (wall time, so the phone can match it to its GPS track)
  void addEvent(EventType type, float severity, float t) {
    if (nEvents < kMaxEvents) ev[nEvents++] = RideEvent{type, severity, t};
  }

  // Returns the length written, or -1 if cap is too small. No distanceKm/lat/lon: the device has
  // no GPS, so the carrier adds those before uploading.
  int toJson(char *out, int cap) const {
    int n = 0;
    bool ok = put(out, cap, n, "{\"id\":%u", (unsigned)id);
    if (startEpoch > 0) {
      time_t s = (time_t)startEpoch;
      struct tm u;
      gmtime_r(&s, &u);
      ok &= put(out, cap, n, ",\"startedAt\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\"",
                u.tm_year + 1900, u.tm_mon + 1, u.tm_mday, u.tm_hour, u.tm_min, u.tm_sec);
    }
    ok &= put(out, cap, n, ",\"durationS\":%.0f,\"score\":%d,\"metrics\":{\"brakeBase_s\":%.2f,\"brakeHard_s\":%.2f,"
              "\"turnBase_s\":%.2f,\"turnHard_s\":%.2f,\"proxBase_s\":%.2f,\"proxHigh_s\":%.2f},\"events\":[",
              m.moving_s, computeScore(m).score, m.brakeBase_s, m.brakeHard_s, m.turnBase_s, m.turnHard_s,
              m.proxBase_s, m.proxHigh_s);
    for (int i = 0; i < nEvents; i++)
      ok &= put(out, cap, n, "%s{\"type\":\"%s\",\"severity\":%.2f,\"t\":%.1f}", i ? "," : "",
                eventName(ev[i].type), ev[i].severity, ev[i].t);
    ok &= put(out, cap, n, "]}");
    return ok ? n : -1;
  }

 private:
  static bool put(char *out, int cap, int &n, const char *fmt, ...) {
    if (n >= cap) return false;
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(out + n, cap - n, fmt, ap);
    va_end(ap);
    if (k < 0 || k >= cap - n) { n = cap; return false; }
    n += k;
    return true;
  }
};
