#ifndef FILE_SERVER_H
#define FILE_SERVER_H

#include <Arduino.h>

// =====================================================
// FileServer
// -----------------------------------------------------
// A small read-only HTTP server exposing the SD card's IGC flight logs
// over the pilot's existing WiFi connection (Connections > WiFi) --
// browse to the ESP32's IP address from any phone/laptop on the same
// network, see a list of recordings, tap one to download it. Built as
// the WiFi-based alternative to USB mass-storage mode, which isn't
// available under this board's "Hardware CDC and JTAG" USB Mode setting
// (that mode doesn't run the TinyUSB stack that a custom USB device
// class like USBMSC needs).
//
// Started/stopped from the menu (Flight Recordings > Export Files, see
// menu.cpp) -- never starts on its own. Requires an active WiFi
// connection (wifiConnected, wifi_manager.h) and a mounted SD card
// (sdCardOK, menu.h); startFileServer() fails cleanly if either isn't
// true rather than starting a server with nothing to serve.
//
// Auto-stops after FILE_SERVER_IDLE_TIMEOUT_MS of no requests, so an
// unauthenticated read-only server doesn't sit reachable on the network
// indefinitely just because turning it off was forgotten.
//
// Read-only by design -- no upload/delete routes. Downloads are served
// directly out of the SD root (where startIgcRecording(), main .ino,
// writes *.IGC files); the requested filename is validated against path
// traversal before ever reaching SD_MMC.open().
// =====================================================

// How long the server stays up with no requests before it stops itself.
// Generous enough to browse the file list and start a download without
// the server disappearing mid-visit, short enough that it doesn't
// linger reachable on the network for hours after the pilot's done.
#define FILE_SERVER_IDLE_TIMEOUT_MS (15UL * 60UL * 1000UL)

// Starts the server. Returns false (and does nothing) if WiFi isn't
// currently connected or the SD card isn't mounted -- check
// isFileServerRunning() afterward, or just use the return value
// directly, to know whether it actually started. Safe to call again
// while already running (resets the idle timer, otherwise a no-op).
bool startFileServer();

// Stops the server if running. Safe to call when not running.
void stopFileServer();

bool isFileServerRunning();

// "http://<ip>/" while running, or an empty string otherwise -- for
// menu.cpp to show the pilot where to browse (Flight Recordings >
// Export Files).
String getFileServerURL();

// Call frequently from a Core 0 task loop (backgroundTask(), main
// .ino) -- NOT from loop() on Core 1, since handling a request can
// block for a noticeable stretch while streaming a file, and Core 1
// must never be delayed (GPS/vario/audio/display/buttons). No-ops
// immediately if the server isn't currently running.
void fileServerLoop();

#endif  // FILE_SERVER_H
