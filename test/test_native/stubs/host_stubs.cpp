#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SD_MMC.h>

HostSerial Serial;
TwoWire Wire;
SDMMCFS    SD_MMC;