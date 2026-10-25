// DriveMotion.h - turns raw IMU samples into forward/backward and sideways acceleration,
// independent of how the sensor is mounted. Pure C++ (host-testable).
//
//  1. Rest calibration: average accel gives "up"; average gyro gives bias.
//  2. Forward calibration: the first gentle push/kick-off after rest defines "forward".
//  3. Running: gravity is tracked with a gyro-aided complementary filter, so hills and bumps
//     don't look like braking. Linear accel = measured - gravity, split into forward/sideways.
// Units: accel in g, gyro in rad/s. Body frame = the IMU's own axes.
#pragma once
#include <math.h>

struct Vec3 { float x, y, z; };
inline Vec3 operator+(Vec3 a, Vec3 b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return Vec3{a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float norm(Vec3 a) { return sqrtf(dot(a, a)); }
inline Vec3 unit(Vec3 a) { float n = norm(a); return n > 1e-6f ? a * (1.0f / n) : a; }

class DriveMotion {
 public:
  enum State { UNCAL, WAIT_PUSH, READY };

  DriveMotion() { reset(); }
  void reset() {
    st_ = UNCAL; up_ = g_ = bias_ = push_ = f_ = l_ = Vec3{0, 0, 0};
    pushN_ = quiet_ = 0; long_ = lat_ = mag_ = 0; hold_ = 0;
  }
  State state() const { return st_; }

  // Call with the average of ~2 s of samples while the scooter is perfectly still.
  void calibrateRest(Vec3 meanAccel, Vec3 meanGyro) {
    reset();
    up_ = unit(meanAccel); g_ = meanAccel; bias_ = meanGyro; st_ = WAIT_PUSH;
  }

  // Forward direction in sensor axes (e.g. restored from flash). Returns false if unusable.
  bool setForward(Vec3 fBody) {
    if (st_ == UNCAL) return false;
    Vec3 f = fBody - up_ * dot(fBody, up_);          // remove the vertical part
    if (norm(f) < 0.02f) return false;
    f_ = unit(f); l_ = cross(up_, f_); st_ = READY;
    return true;
  }
  Vec3 forward() const { return f_; }

  void update(Vec3 a, Vec3 w, float dt) {
    if (st_ == UNCAL) return;
    if (st_ == WAIT_PUSH) { learnForward(a); return; }

    w = w - bias_;
    g_ = g_ - cross(w, g_) * dt;                      // rotate gravity with the sensor
    if (fabsf(norm(a) - norm(g_)) < 0.2f) g_ = g_ + (a - g_) * kGravGain;  // slow pull to accel
    Vec3 lin = a - g_;
    long_ += (dot(lin, f_) - long_) * kSmooth;        // + = speeding up, - = braking
    lat_  += (dot(lin, l_) - lat_) * kSmooth;
    mag_  += (norm(lin) - mag_) * 0.02f;
    if (mag_ > kMoveG) hold_ = kHoldSamples; else if (hold_ > 0) hold_--;
  }

  float brakeG() const { return long_ < 0 ? -long_ : 0; }
  float accelG() const { return long_ > 0 ? long_ : 0; }
  float lateralG() const { return fabsf(lat_); }
  bool moving() const { return hold_ > 0; }   // vibration-based; replace with GPS speed later

 private:
  static constexpr float kGravGain = 0.003f;      // ~3 s time constant at 100 Hz
  static constexpr float kSmooth = 0.08f;         // ~0.12 s smoothing of long/lat accel
  static constexpr float kMoveG = 0.05f;
  static constexpr int kHoldSamples = 400;        // 4 s at 100 Hz
  State st_;
  Vec3 up_, g_, bias_, push_, f_, l_;
  int pushN_, quiet_, hold_;
  float long_, lat_, mag_;

  void learnForward(Vec3 a) {
    Vec3 lin = a - g_;
    if (norm(lin) > 0.08f) {
      quiet_ = 0;
      push_ = push_ + lin;
      if (++pushN_ >= 40) {                            // 0.4 s of pushing
        Vec3 mean = push_ * (1.0f / pushN_);
        Vec3 h = mean - up_ * dot(mean, up_);
        if (norm(h) >= 0.04f && setForward(h)) return;
        pushN_ = 0; push_ = Vec3{0, 0, 0};
      }
    } else if (pushN_ > 0 && ++quiet_ > 10) {          // noise burst, start over
      pushN_ = quiet_ = 0; push_ = Vec3{0, 0, 0};
    }
  }
};
