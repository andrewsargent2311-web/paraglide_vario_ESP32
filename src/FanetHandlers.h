#pragma once

#include <Arduino.h>

#include "Sx126xLink.h"
#include "Fanet.h"
#include "DrawPages.h"  // FanetContact, FanetWeatherStation, MAX_FANET_CONTACTS, MAX_FANET_WEATHER_STATIONS
#include "settings.h"   // fanetEnabled

// =====================================================
// FANET (HT-RA62 / SX1262) -- wired MISO=0 SCK=1 BUSY=2 MOSI=3 CS=17,
// DIO1 and RST both NC (see Sx126xLink.h for why). Own SPI bus, separate
// from the display's -- no pin overlap with anything else on this board.
// -----------------------------------------------------
// Named FanetHandlers.h, not Fanet.h, to avoid colliding with the
// existing Fanet.h this file includes.
//
// Radio bring-up in setup() (only runs if fanetEnabled was already true
// at boot) and the per-loop position-update block (feeding
// fanet.setPosition()/setAircraftType()) both stay inline in the main
// .ino, unchanged -- they just reference the externs/functions below
// instead of file-local globals. fanet.update() (RX polling + beacon
// schedule) is also called inline from loop(), unchanged.
//
// IMPORTANT: fanetRadio, myFanetAddress and fanet must all stay defined
// in the same .cpp, in the same relative order they appear below --
// FanetStack fanet(fanetRadio, myFanetAddress) depends on the other two
// at construction time, and C++ only guarantees static-init order
// *within* one translation unit.
// =====================================================

extern Sx126xLink fanetRadio;
extern bool fanetRadioOK;

// TODO: pick a real manufacturer ID from the FANET spec's registered
// list (or use a private/testing value while bench-testing) rather than
// this placeholder, and give this device a unique 16-bit ID.
extern FanetAddress myFanetAddress;
extern FanetStack fanet;

// Live FANET traffic table (type/constants in DrawPages.h, alongside
// WindMeter -- same "shared between the writer here and the
// DrawPages.cpp reader" pattern). Definition lives in FanetHandlers.cpp;
// DrawPages.h only declares these extern.
extern FanetContact fanetContacts[MAX_FANET_CONTACTS];
extern FanetWeatherStation fanetWeatherStations[MAX_FANET_WEATHER_STATIONS];

#define FANET_BEACON_INTERVAL_MS 5000UL

// Fires whenever a FANET tracking beacon is received from another
// aircraft. Updates fanetContacts[] so drawADSBPage() (DrawPages.cpp) can
// plot it alongside ADS-B traffic -- see the FanetContact comment in
// DrawPages.h for why no mutex is needed here despite drawADSBPage()
// reading the same array.
void onFanetTracking(const FanetAddress& src, const FanetTracking& pkt,
                      float rssi, float snr);

// Fires whenever a FANET Service (type 4) packet with wind data is
// received from a ground weather station. Updates fanetWeatherStations[]
// so drawWeatherPage() (DrawPages.cpp) can show it -- as the preferred
// source (Config > Weather Settings > Source = FANET) or as the fallback
// when the Zephyr network has no data. See the FanetWeatherStation
// comment in DrawPages.h for why no mutex is needed here.
void onFanetWeather(const FanetAddress& src, const FanetWeather& pkt,
                     float rssi, float snr);

// =====================================================
// FANET ENABLE/DISABLE (Config > FANET)
// Called once from setup() (only if fanetEnabled was already true at
// boot -- see the FANET init block that stays inline there, which
// handles that first-time case directly) and again immediately from
// menu.cpp any time the pilot flips the toggle.
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
void setFanetEnabled(bool enabled);
