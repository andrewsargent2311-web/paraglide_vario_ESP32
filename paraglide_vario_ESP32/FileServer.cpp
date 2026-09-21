#include "FileServer.h"
#include "menu.h"          // sdCardOK, sdMutex
#include "wifi_manager.h"  // wifiConnected

#include <WebServer.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <FS.h>

static WebServer server(80);

static bool serverRunning = false;
static unsigned long lastActivityMs = 0;

// ---------------------------------------------------------------------------
// HTML page listing every *.IGC file in the SD root, each a download link.
// ---------------------------------------------------------------------------

static void handleRoot() {
  lastActivityMs = millis();

  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    server.send(503, "text/plain", "SD card busy -- try again in a moment");
    return;
  }

  String html;
  html.reserve(2048);

  html += "<!doctype html><html><head><title>Flight Recordings</title>";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<style>body{font-family:sans-serif;margin:2em;max-width:480px}";
  html += "li{margin:0.6em 0}a{text-decoration:none;color:#0645AD;font-size:1.1em}</style>";
  html += "</head><body><h1>Flight Recordings</h1>";

  int count = 0;
  File root = SD_MMC.open("/");

  if (root && root.isDirectory()) {
    html += "<ul>";

    File f = root.openNextFile();
    while (f) {
      String name = f.name();

      // File::name() has returned either a bare filename or a
      // leading-slash path depending on core version -- normalize to
      // bare so the /download?file= links below are consistent either
      // way.
      if (name.startsWith("/")) {
        name.remove(0, 1);
      }

      bool isIgc = name.endsWith(".IGC") || name.endsWith(".igc");

      if (!f.isDirectory() && isIgc) {
        size_t sizeKb = (f.size() + 512) / 1024;  // rounded, not truncated
        html += "<li><a href=\"/download?file=" + name + "\">" + name + "</a>";
        html += " (" + String(sizeKb) + " KB)</li>";
        count++;
      }

      f = root.openNextFile();
    }

    html += "</ul>";
  }

  xSemaphoreGive(sdMutex);

  if (count == 0) {
    html += "<p>No flight recordings found.</p>";
  }

  html += "</body></html>";

  server.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// Streams a single file for download.
// ---------------------------------------------------------------------------

static void handleDownload() {
  lastActivityMs = millis();

  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }

  String filename = server.arg("file");

  // Always serves directly out of the SD root -- reject anything that
  // could escape it (a literal '/' anywhere, or '..') before it ever
  // reaches SD_MMC.open(). Every real link on the root page above is a
  // bare filename, so this only ever rejects a hand-crafted request.
  if (filename.indexOf("..") >= 0 || filename.indexOf('/') >= 0 || filename.length() == 0) {
    server.send(400, "text/plain", "Invalid filename");
    return;
  }

  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    server.send(503, "text/plain", "SD card busy -- try again in a moment");
    return;
  }

  String path = "/" + filename;
  File f = SD_MMC.open(path, FILE_READ);

  if (!f || f.isDirectory()) {
    if (f) f.close();
    xSemaphoreGive(sdMutex);
    server.send(404, "text/plain", "File not found");
    return;
  }

  server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
  server.streamFile(f, "application/octet-stream");
  f.close();

  xSemaphoreGive(sdMutex);
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool startFileServer() {
  if (serverRunning) {
    lastActivityMs = millis();  // treat a repeat call as activity too
    return true;
  }

  if (!wifiConnected) {
    Serial.println("[EXPORT] Cannot start file server -- WiFi not connected");
    return false;
  }

  if (!sdCardOK) {
    Serial.println("[EXPORT] Cannot start file server -- SD card not mounted");
    return false;
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/download", HTTP_GET, handleDownload);
  server.onNotFound(handleNotFound);
  server.begin();

  serverRunning = true;
  lastActivityMs = millis();

  Serial.printf("[EXPORT] File server started at http://%s/\n", WiFi.localIP().toString().c_str());

  return true;
}

void stopFileServer() {
  if (!serverRunning) return;

  server.stop();
  serverRunning = false;

  Serial.println("[EXPORT] File server stopped");
}

bool isFileServerRunning() {
  return serverRunning;
}

String getFileServerURL() {
  if (!serverRunning) return "";
  return "http://" + WiFi.localIP().toString() + "/";
}

void fileServerLoop() {
  if (!serverRunning) return;

  server.handleClient();

  if (millis() - lastActivityMs >= FILE_SERVER_IDLE_TIMEOUT_MS) {
    Serial.println("[EXPORT] File server idle timeout -- stopping");
    stopFileServer();
  }
}
