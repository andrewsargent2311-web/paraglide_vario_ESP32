#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <Arduino.h>

// =====================================================
// FLIGHT LOG (Flight Recordings > Flights)
// -----------------------------------------------------
// Keeps a one-line summary of each recording in /FLIGHTS.CSV on the SD card,
// which the Flights screen reads back.
//
// Most recordings never get a clean stop: Auto-Stop is off by default, so
// they usually end with the power going off. So the summary so far is saved
// to /FLIGHT.CUR once a minute during a recording. A clean stop moves it into
// FLIGHTS.CSV straight away; otherwise the next boot does, which costs at
// most the last minute of the flight.
//
// A recording that has no summary (from before this feature, or lost some
// other way) still lists its date and time, from the IGC filename, and its
// duration, from the first and last track points in the file.
//
// All SD work happens in flightLogService() on Core 0, so Core 1 is never
// held up by the card.
// =====================================================

#define FLIGHT_LOG_MAX_LISTED 12

struct FlightLogEntry {
  uint32_t startEpoch;   // UTC; 0 if the clock wasn't set when it started
  uint32_t durationSec;
  bool hasDuration;
  bool hasSummary;       // false = only what the IGC file itself could tell us
  bool hasMaxAlt;
  float maxAltM;
  bool hasBestClimb;
  float bestClimbMs;
};

enum FlightLogListState {
  FLIGHT_LIST_IDLE = 0,
  FLIGHT_LIST_LOADING,
  FLIGHT_LIST_READY
};

// Call every loop() pass on Core 1, after updateIgcRecorder(). Notices
// recordings starting and stopping, and keeps track of their summary.
void flightLogUpdate();

// Call from backgroundTask() on Core 0.
void flightLogService();

// Called by the menu (Core 1) when the Flights screen opens: asks Core 0 to
// load the list. While loading, the state is FLIGHT_LIST_LOADING. Only read
// the entries once it's FLIGHT_LIST_READY, newest first.
void flightLogRequestList();
FlightLogListState flightLogListState();
uint8_t flightLogListCount();
const FlightLogEntry& flightLogListEntry(uint8_t index);

#endif  // FLIGHT_LOG_H
