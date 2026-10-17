#ifndef ENGINE_NMEA_H
#define ENGINE_NMEA_H

#include <stddef.h>
#include <stdint.h>

struct EngineReading {
  bool rpmUpdated;
  float rpm;
  bool rpmUnavailable;
  bool egtUpdated;
  float egtC;
  bool egtFault;
  bool chtUpdated;
  float chtC;
  bool chtFault;
  bool batteryUpdated;
  uint8_t batteryPercent;
};

struct EngineNmeaCounters {
  uint32_t checksumRejected;
  uint32_t malformedRejected;
};

class EngineNmeaLineAssembler {
 public:
  static constexpr size_t kLineCapacity = 96U;
  using LineHandler = void (*)(const char* line, size_t len, void* context);

  void feed(const uint8_t* data, size_t len, LineHandler handler, void* context);
  uint32_t overflowCount() const;

 private:
  char lineBuffer_[kLineCapacity];
  size_t lineLength_ = 0U;
  bool collecting_ = false;
  uint32_t overflowCount_ = 0U;
};

[[nodiscard]] bool parseEngineSentence(const char* line, size_t len, EngineReading& out);
[[nodiscard]] bool parseEngineSentence(const char* line, size_t len, EngineReading& out,
                                       EngineNmeaCounters& counters);

#endif  // ENGINE_NMEA_H
