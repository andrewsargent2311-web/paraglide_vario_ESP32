#include "FanetHandlers.h"
#include "FanetMessaging.h"  // onFanetMessageReceived
#include "settings.h"

// NOTE: fanetRadio, myFanetAddress and fanet must stay together, in this
// order, in this one translation unit (static-init order).
// =====================================================
// FANET (HT-RA62 / SX1262) -- wired MISO=0 SCK=1 BUSY=2 MOSI=3 CS=17,
// DIO1 and RST both NC (see Sx126xLink.h for why). Own SPI bus, separate
// from the display's -- no pin overlap with anything else on this board.
// =====================================================
Sx126xLink fanetRadio;
bool fanetRadioOK = false;

// TODO: pick a real manufacturer ID from the FANET spec's registered
// list (or use a private/testing value while bench-testing) rather than
// this placeholder, and give this device a unique 16-bit ID.
FanetAddress myFanetAddress = { 0xFC, 0x0001 };
FanetStack fanet(fanetRadio, myFanetAddress);


// Live FANET traffic table (type/constants in DrawPages.h, alongside
// WindMeter -- same "shared between the .ino writer and the DrawPages.cpp
// reader" pattern). Definition lives here; DrawPages.h only declares it
// extern.
FanetContact fanetContacts[MAX_FANET_CONTACTS];
FanetWeatherStation fanetWeatherStations[MAX_FANET_WEATHER_STATIONS];

// Fires whenever a FANET tracking beacon is received from another
// aircraft. Updates fanetContacts[] so drawADSBPage() (DrawPages.cpp) can
// plot it alongside ADS-B traffic -- see the FanetContact comment in
// DrawPages.h for why no mutex is needed here despite drawADSBPage()
// reading the same array.
void onFanetTracking(const FanetAddress& src, const FanetTracking& pkt,
                      float rssi, float snr) {
  Serial.printf("[FANET] from %02X:%04X  lat=%.5f lon=%.5f alt=%ldm  spd=%.0fkm/h  rssi=%.0f snr=%.1f\n",
                src.manufacturer, src.id,
                pkt.latitude, pkt.longitude, (long)pkt.altitudeM,
                pkt.speedKmh, rssi, snr);

  // Find the existing slot for this address, or the first free slot if
  // it's new, or -- if the table is completely full of other live
  // contacts -- evict whichever one has gone quietest. A beacon actually
  // arriving right now is always more useful to show than a contact we
  // haven't heard from in a while.
  int slot = -1;
  int quietestSlot = 0;
  unsigned long quietestSeenMs = (unsigned long)-1;  // wraps to ULONG_MAX

  for (int i = 0; i < MAX_FANET_CONTACTS; i++) {
    if (fanetContacts[i].valid &&
        fanetContacts[i].addr.manufacturer == src.manufacturer &&
        fanetContacts[i].addr.id == src.id) {
      slot = i;
      break;
    }
    if (!fanetContacts[i].valid && slot == -1) {
      slot = i;  // remember the first free slot, but keep scanning for an exact match
    }
    if (fanetContacts[i].lastSeenMs < quietestSeenMs) {
      quietestSeenMs = fanetContacts[i].lastSeenMs;
      quietestSlot = i;
    }
  }

  if (slot == -1) {
    slot = quietestSlot;  // table full of other live contacts -- evict the quietest
  }

  FanetContact& c = fanetContacts[slot];
  c.valid = true;
  c.addr = src;
  c.lat = (float)pkt.latitude;
  c.lon = (float)pkt.longitude;
  c.altitudeM = pkt.altitudeM;
  c.speedKmh = pkt.speedKmh;
  c.climbMs = pkt.climbMs;
  c.headingDeg = pkt.headingDeg;
  c.aircraftType = pkt.aircraftType;
  c.rssi = rssi;
  c.snr = snr;
  c.lastSeenMs = millis();
}

// Fires whenever a FANET Service (type 4) packet with wind data is
// received from a ground weather station. Updates fanetWeatherStations[]
// so drawWeatherPage() (DrawPages.cpp) can show it -- as the preferred
// source (Config > Weather Settings > Source = FANET) or as the fallback
// when the Zephyr network has no data. See the FanetWeatherStation
// comment in DrawPages.h for why no mutex is needed here.
void onFanetWeather(const FanetAddress& src, const FanetWeather& pkt,
                     float rssi, float snr) {
  Serial.printf("[FANET WX] from %02X:%04X  lat=%.5f lon=%.5f  wind=%.0f@%.0fkm/h gust=%.0fkm/h  rssi=%.0f snr=%.1f\n",
                src.manufacturer, src.id,
                pkt.latitude, pkt.longitude,
                pkt.windHeadingDeg, pkt.windSpeedKmh, pkt.windGustKmh,
                rssi, snr);

  // Same find-slot-or-evict-quietest approach as onFanetTracking() above.
  int slot = -1;
  int quietestSlot = 0;
  unsigned long quietestSeenMs = (unsigned long)-1;

  for (int i = 0; i < MAX_FANET_WEATHER_STATIONS; i++) {
    if (fanetWeatherStations[i].valid &&
        fanetWeatherStations[i].addr.manufacturer == src.manufacturer &&
        fanetWeatherStations[i].addr.id == src.id) {
      slot = i;
      break;
    }
    if (!fanetWeatherStations[i].valid && slot == -1) {
      slot = i;
    }
    if (fanetWeatherStations[i].lastSeenMs < quietestSeenMs) {
      quietestSeenMs = fanetWeatherStations[i].lastSeenMs;
      quietestSlot = i;
    }
  }

  if (slot == -1) {
    slot = quietestSlot;
  }

  FanetWeatherStation& w = fanetWeatherStations[slot];
  w.valid = true;
  w.addr = src;
  w.lat = (float)pkt.latitude;
  w.lon = (float)pkt.longitude;
  w.windHeadingDeg = pkt.windHeadingDeg;
  w.windSpeedKmh = pkt.windSpeedKmh;
  w.windGustKmh = pkt.windGustKmh;
  w.hasTemperature = pkt.hasTemperature;
  w.temperatureC = pkt.temperatureC;
  w.lastSeenMs = millis();
}

// =====================================================
// FANET ENABLE/DISABLE (Config > FANET)
// Called once from setup() (only if fanetEnabled was already true at
// boot -- see the FANET init block above, which handles that first-time
// case directly) and again immediately from menu.cpp any time the pilot
// flips the toggle.
//
// Two distinct cases:
//   - Radio was never successfully brought up (fanetRadioOK == false,
//     e.g. it was off at boot) -- run the exact same init sequence
//     setup() would have run, so switching it on later actually starts
//     it for the first time.
//   - Radio is already up -- just sleep/wake the chip itself via
//     Sx126xLink::setEnabled(), which is far cheaper than a full begin()
//     and preserves its current config.
// =====================================================
void setFanetEnabled(bool enabled) {
  if (!fanetRadioOK) {
    if (!enabled) {
      // Nothing to turn off -- it was never brought up.
      return;
    }

    fanetRadioOK = fanetRadio.begin(/*freqMHz=*/868.2f, /*bwKHz=*/250.0f,
                                     /*sf=*/7, /*cr=*/5, /*syncWord=*/0xF1,
                                     /*powerDbm=*/14, /*preambleLen=*/8);
    if (fanetRadioOK) {
      fanet.begin();
      fanet.onTracking(onFanetTracking);
      fanet.onWeather(onFanetWeather);
      fanet.onMessage(onFanetMessageReceived);
      fanet.setBeaconIntervalMs(FANET_BEACON_INTERVAL_MS);
      Serial.println("FANET RADIO INITIALIZED (enabled from menu)");
    } else {
      Serial.printf("FANET RADIO INIT FAILED -- status=%d\n", fanetRadio.lastStatus());
    }
    return;
  }

  if (!fanetRadio.setEnabled(enabled)) {
    Serial.printf("[FANET] setEnabled(%d) FAILED -- status=%d\n",
                  enabled, fanetRadio.lastStatus());
    return;
  }

  Serial.printf("[FANET] Radio %s\n", enabled ? "enabled" : "disabled (sleep)");
}
