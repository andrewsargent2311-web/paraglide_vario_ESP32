#pragma once

class AglDisplayBuffer {
public:
  static constexpr unsigned long HOLD_TIME_MS = 5000UL;

  bool update(bool sourcesValid, float currentAglM, unsigned long nowMs, float& displayAglM) {
    if (sourcesValid) {
      lastAglM = currentAglM;
      lastValidAtMs = nowMs;
      hasLastValidAgl = true;
      displayAglM = currentAglM;
      return true;
    }

    if (hasLastValidAgl && nowMs - lastValidAtMs <= HOLD_TIME_MS) {
      displayAglM = lastAglM;
      return true;
    }

    return false;
  }

private:
  float lastAglM = 0.0f;
  unsigned long lastValidAtMs = 0;
  bool hasLastValidAgl = false;
};
