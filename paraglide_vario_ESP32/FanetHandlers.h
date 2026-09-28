#pragma once
// FanetHandlers -- FANET radio/stack objects, live contact tables and the
// tracking/weather receive callbacks. Radio bring-up in setup() and the
// per-loop position feed stay inline in the .ino.
#include <Arduino.h>
#include "Sx126xLink.h"
#include "Fanet.h"
#include "DrawPages.h"  // FanetContact, FanetWeatherStation, MAX_FANET_*

#define FANET_BEACON_INTERVAL_MS 5000UL

extern Sx126xLink fanetRadio;
extern bool fanetRadioOK;
extern FanetAddress myFanetAddress;
extern FanetStack fanet;
extern FanetContact fanetContacts[MAX_FANET_CONTACTS];
extern FanetWeatherStation fanetWeatherStations[MAX_FANET_WEATHER_STATIONS];

void onFanetTracking(const FanetAddress& src, const FanetTracking& pkt,
                     float rssi, float snr);
void onFanetWeather(const FanetAddress& src, const FanetWeather& pkt,
                    float rssi, float snr);
void setFanetEnabled(bool enabled);
