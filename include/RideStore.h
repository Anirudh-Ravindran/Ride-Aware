// RideStore.h - rides kept in flash until a carrier uploads them and deletes them.
// LittleFS on the "spiffs" partition (3.4 MB in default_16MB.csv). One file per ride,
// /rides/<id>.json, holding the POST /api/rides body. Every save writes a temp file and renames
// it over the old one, so a power cut mid-write leaves the previous checkpoint intact.
#pragma once
#include <Arduino.h>
#include <LittleFS.h>

class RideStore {
 public:
  static constexpr int kMaxRides = 300;                 // past this, or when flash is nearly full,
  static constexpr size_t kMinFreeBytes = 64 * 1024;    // the oldest ride is dropped

  bool begin() {
    if (!LittleFS.begin(true)) return false;            // formats the partition on first boot
    if (!LittleFS.exists("/rides")) LittleFS.mkdir("/rides");
    LittleFS.remove(kTmp);                              // half-written save from a power cut
    return ok_ = true;
  }

  bool save(uint32_t id, const char *json, size_t len) {
    if (!ok_) return false;
    if (id != lastSaved_) makeRoom(id);                 // only on a ride's first save
    File f = LittleFS.open(kTmp, FILE_WRITE);
    if (!f) return false;
    bool good = f.write((const uint8_t *)json, len) == len;
    f.close();
    if (!good || !LittleFS.rename(kTmp, path(id))) { LittleFS.remove(kTmp); return false; }
    lastSaved_ = id;
    return true;
  }

  bool read(uint32_t id, String &out) {
    File f = LittleFS.open(path(id), FILE_READ);
    if (!f) return false;
    out = f.readString();
    return true;
  }

  bool remove(uint32_t id) { return ok_ && LittleFS.remove(path(id)); }

  // fn(id, bytes) for every saved ride, in directory order
  template <typename F> void forEach(F fn) {
    if (!ok_) return;
    File dir = LittleFS.open("/rides");
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      uint32_t id = strtoul(f.name(), nullptr, 10);
      if (id) fn(id, f.size());
    }
  }

 private:
  static constexpr const char *kTmp = "/ride.tmp";
  bool ok_ = false;
  uint32_t lastSaved_ = 0;

  static String path(uint32_t id) { return "/rides/" + String(id) + ".json"; }

  void makeRoom(uint32_t keep) {
    for (;;) {
      int n = 0;
      uint32_t oldest = 0;
      forEach([&](uint32_t id, size_t) { if (id != keep) { n++; if (!oldest || id < oldest) oldest = id; } });
      if (!oldest || (n < kMaxRides && LittleFS.totalBytes() - LittleFS.usedBytes() >= kMinFreeBytes)) return;
      remove(oldest);
    }
  }
};
