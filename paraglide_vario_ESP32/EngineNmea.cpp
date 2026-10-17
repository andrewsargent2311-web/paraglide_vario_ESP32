#include "EngineNmea.h"

#include <string.h>

namespace {

bool parseInteger(const char* text, size_t len, int32_t& result) {
  if (len == 0U) return false;

  size_t index = 0U;
  bool negative = false;
  if (text[index] == '-' || text[index] == '+') {
    negative = text[index] == '-';
    ++index;
  }
  if (index == len) return false;

  const uint32_t limit = negative ? 2147483648UL : 2147483647UL;
  uint32_t value = 0U;
  for (; index < len; ++index) {
    const char ch = text[index];
    if (ch < '0' || ch > '9') return false;
    const uint32_t digit = static_cast<uint32_t>(ch - '0');
    if (value > (limit - digit) / 10U) return false;
    value = value * 10U + digit;
  }

  if (negative) {
    result = value == 2147483648UL ? (-2147483647 - 1) : -static_cast<int32_t>(value);
  } else {
    result = static_cast<int32_t>(value);
  }
  return true;
}

int hexValue(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  return -1;
}

void rejectMalformed(EngineNmeaCounters* counters) {
  if (counters != nullptr) ++counters->malformedRejected;
}

bool validateEnvelope(const char* line, size_t len, size_t& starIndex,
                      EngineNmeaCounters* counters) {
  if (line == nullptr || len < 4U || line[0] != '$') {
    rejectMalformed(counters);
    return false;
  }

  starIndex = len;
  for (size_t i = 1U; i < len; ++i) {
    if (line[i] == '*') {
      starIndex = i;
      break;
    }
  }
  if (starIndex == len || len - starIndex != 3U) {
    rejectMalformed(counters);
    return false;
  }

  const int high = hexValue(line[starIndex + 1U]);
  const int low = hexValue(line[starIndex + 2U]);
  if (high < 0 || low < 0) {
    rejectMalformed(counters);
    return false;
  }

  uint8_t checksum = 0U;
  for (size_t i = 1U; i < starIndex; ++i) {
    checksum ^= static_cast<uint8_t>(line[i]);
  }
  const uint8_t expectedChecksum = static_cast<uint8_t>((high << 4) | low);
  if (checksum != expectedChecksum) {
    if (counters != nullptr) ++counters->checksumRejected;
    return false;
  }
  return true;
}

bool parseEmsFields(const char* line, size_t fieldStart, size_t starIndex,
                    int32_t* fields, EngineNmeaCounters* counters) {
  for (size_t field = 0U; field < 9U; ++field) {
    size_t fieldEnd = fieldStart;
    while (fieldEnd < starIndex && line[fieldEnd] != ',') ++fieldEnd;
    if (fieldEnd == fieldStart ||
        !parseInteger(line + fieldStart, fieldEnd - fieldStart, fields[field])) {
      rejectMalformed(counters);
      return false;
    }
    const bool finalField = field == 8U;
    if ((finalField && fieldEnd != starIndex) ||
        (!finalField && fieldEnd == starIndex)) {
      rejectMalformed(counters);
      return false;
    }
    fieldStart = fieldEnd + 1U;
  }
  return true;
}

bool parseBatterySentence(const char* line, size_t identifierEnd, size_t starIndex,
                          EngineReading& out, EngineNmeaCounters* counters) {
  if (identifierEnd == starIndex || line[identifierEnd] != ',') {
    rejectMalformed(counters);
    return false;
  }

  int32_t battery = 0;
  const size_t valueStart = identifierEnd + 1U;
  if (!parseInteger(line + valueStart, starIndex - valueStart, battery) ||
      battery < 0 || battery > 100) {
    rejectMalformed(counters);
    return false;
  }

  EngineReading parsed = {};
  parsed.batteryUpdated = true;
  parsed.batteryPercent = static_cast<uint8_t>(battery);
  out = parsed;
  return true;
}

bool parseEmsSentence(const char* line, size_t identifierEnd, size_t starIndex,
                      EngineReading& out, EngineNmeaCounters* counters) {
  if (identifierEnd == starIndex || line[identifierEnd] != ',') {
    rejectMalformed(counters);
    return false;
  }

  int32_t fields[9] = {};
  if (!parseEmsFields(line, identifierEnd + 1U, starIndex, fields, counters)) {
    return false;
  }
  if (fields[8] < 0) {
    rejectMalformed(counters);
    return false;
  }

  EngineReading parsed = {};
  parsed.chtUpdated = true;
  parsed.chtFault = fields[5] == 99;
  parsed.chtC = parsed.chtFault ? 0.0f : static_cast<float>(fields[5]) / 100.0f;
  parsed.egtUpdated = true;
  parsed.egtFault = fields[6] == 99;
  parsed.egtC = parsed.egtFault ? 0.0f : static_cast<float>(fields[6]) / 100.0f;
  parsed.rpmUpdated = true;
  parsed.rpmUnavailable = fields[8] == 9999;
  parsed.rpm = parsed.rpmUnavailable ? 0.0f : static_cast<float>(fields[8]) / 100.0f;
  out = parsed;
  return true;
}

bool parseSentence(const char* line, size_t len, EngineReading& out,
                   EngineNmeaCounters* counters) {
  size_t starIndex = 0U;
  if (!validateEnvelope(line, len, starIndex, counters)) return false;

  size_t identifierEnd = 1U;
  while (identifierEnd < starIndex && line[identifierEnd] != ',') ++identifierEnd;
  const size_t identifierLen = identifierEnd - 1U;
  if (identifierLen == 0U) {
    rejectMalformed(counters);
    return false;
  }
  if (identifierLen == 3U && memcmp(line + 1U, "EMS", 3U) == 0) {
    return parseEmsSentence(line, identifierEnd, starIndex, out, counters);
  }
  if (identifierLen == 6U && memcmp(line + 1U, "PEMBAT", 6U) == 0) {
    return parseBatterySentence(line, identifierEnd, starIndex, out, counters);
  }
  return false;
}

}  // namespace

void EngineNmeaLineAssembler::feed(const uint8_t* data, size_t len,
                                   LineHandler handler, void* context) {
  if (data == nullptr) return;

  for (size_t i = 0U; i < len; ++i) {
    const char byte = static_cast<char>(data[i]);
    if (byte == '\r') continue;
    if (byte == '$') {
      collecting_ = true;
      lineLength_ = 0U;
      lineBuffer_[lineLength_++] = byte;
      continue;
    }
    if (!collecting_) continue;
    if (byte == '\n') {
      if (handler != nullptr) handler(lineBuffer_, lineLength_, context);
      collecting_ = false;
      lineLength_ = 0U;
      continue;
    }
    if (lineLength_ >= kLineCapacity) {
      ++overflowCount_;
      collecting_ = false;
      lineLength_ = 0U;
      continue;
    }
    lineBuffer_[lineLength_++] = byte;
  }
}

uint32_t EngineNmeaLineAssembler::overflowCount() const {
  return overflowCount_;
}

bool parseEngineSentence(const char* line, size_t len, EngineReading& out) {
  return parseSentence(line, len, out, nullptr);
}

bool parseEngineSentence(const char* line, size_t len, EngineReading& out,
                         EngineNmeaCounters& counters) {
  return parseSentence(line, len, out, &counters);
}
