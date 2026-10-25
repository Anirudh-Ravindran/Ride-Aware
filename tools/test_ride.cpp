// Host-side test: ride start/end detection and the saved ride JSON.
// Build: g++ -std=c++11 -Wall -I../include test_ride.cpp -o tr && ./tr
#include <cstdio>
#include <cstring>
#include "RideLog.h"

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok   %s\n", msg); } while (0)

// Feed the detector `sec` seconds of moving/not moving at 100 Hz; count the edges.
struct Run {
  RideDetector d; uint32_t t = 1000; int starts = 0, ends = 0;
  void go(bool moving, float sec) {
    for (int i = 0; i < (int)(sec * 100); i++, t += 10) {
      RideDetector::Edge e = d.update(moving, t);
      starts += e == RideDetector::STARTED; ends += e == RideDetector::ENDED;
    }
  }
};

int main() {
  Run a;
  a.go(false, 10); a.go(true, 3); a.go(false, 10);
  CHECK(a.starts == 0, "3 s of movement is not a ride");
  a.go(true, 6);
  CHECK(a.starts == 1 && a.d.active(), "6 s of movement starts a ride");
  CHECK(a.d.startMs() == 24000, "ride start = when that movement began");
  a.go(false, 90); a.go(true, 60);
  CHECK(a.ends == 0 && a.starts == 1, "90 s red light does not split the ride");
  a.go(false, 119);
  CHECK(a.ends == 0, "still riding at 119 s stopped");
  a.go(false, 2);
  CHECK(a.ends == 1 && !a.d.active(), "ride ends after 120 s stopped");
  a.go(true, 10);
  CHECK(a.starts == 2, "next movement starts a new ride");

  static Ride r;
  r.begin(42, 5000);
  r.m = DriveMetrics{110, 3, 90, 1, 45, 4, 900};
  r.addEvent(EV_HARD_BRAKE, 0.8f, 312.25f);
  r.addEvent(EV_CLOSE_CALL, 0.5f, 400);
  char buf[Ride::kMaxJson];
  int n = r.toJson(buf, sizeof(buf));
  printf("%s\n", buf);
  CHECK(n > 0 && (int)strlen(buf) == n, "json written");
  CHECK(!strstr(buf, "startedAt"), "no startedAt while the clock is unknown");
  CHECK(strstr(buf, "\"durationS\":900,") && strstr(buf, "\"brakeHard_s\":3.00"), "metrics present");
  CHECK(strstr(buf, "{\"type\":\"hard_brake\",\"severity\":0.80,\"t\":312.2},{\"type\":\"close_call\""), "events present");
  CHECK(buf[n - 2] == ']' && buf[n - 1] == '}', "json closed");

  r.startEpoch = 1792850520;                        // 2026-10-24T14:02:00Z
  n = r.toJson(buf, sizeof(buf));
  CHECK(strstr(buf, "\"startedAt\":\"2026-10-24T14:02:00Z\""), "startedAt from the clock");

  CHECK(r.toJson(buf, 100) == -1, "too-small buffer reports failure");

  r.m = DriveMetrics{99999, 99999, 99999, 99999, 99999, 99999, 99999};
  for (int i = 0; i < Ride::kMaxEvents + 50; i++) r.addEvent(EV_CLOSE_CALL, 1, 99999.9f);
  CHECK(r.nEvents == Ride::kMaxEvents, "events capped");
  n = r.toJson(buf, sizeof(buf));
  printf("worst case %d of %d bytes\n", n, Ride::kMaxJson);
  CHECK(n > 0, "worst-case ride fits in kMaxJson");

  printf("\n%s (%d failures)\n", fails ? "TESTS FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}
