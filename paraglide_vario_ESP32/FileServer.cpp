#include "FileServer.h"
#include "menu.h"          // sdCardOK, sdMutex
#include "wifi_manager.h"  // wifiConnected
#include "IgcRecorder.h"   // igcRecording
#include "settings.h"      // selectedDemFile, DEM_FILENAME_MAX_LEN

#include <WebServer.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <FS.h>

// Upper bound on entries in the recordings listing.
#define FILE_SERVER_MAX_LISTED 200

static WebServer server(80);

static bool serverRunning = false;
static unsigned long lastActivityMs = 0;

// ---------------------------------------------------------------------------
// Helpers shared by the pages below.
// ---------------------------------------------------------------------------

static bool hasExtensionCI(const String& name, const char* ext) {
  String lower = name;
  lower.toLowerCase();
  return lower.endsWith(ext);
}

// Bare filename in the SD root only -- nothing that could escape it.
static bool isSafeRootName(const String& name) {
  return name.length() > 0 && name.indexOf("..") < 0 && name.indexOf('/') < 0 &&
         name.indexOf('\\') < 0;
}

static bool isIgcName(const String& name) { return hasExtensionCI(name, ".igc"); }
static bool isMapName(const String& name) { return hasExtensionCI(name, ".adem"); }
static bool isAirspaceName(const String& name) {
  String lower = name;
  lower.toLowerCase();
  return lower == "airspace.txt";
}
static bool isSplashName(const String& name) { return name.equalsIgnoreCase("ROY.BIN"); }

// A file that belongs in a voice folder: V000.PCM ... V059.PCM style clips
// (VoiceAlert.cpp), or the manifest.txt listing that sits beside them.
static bool isVoiceFileName(const String& file) {
  if (file.equalsIgnoreCase("manifest.txt")) return true;
  if (file.length() != 8 || (file[0] != 'V' && file[0] != 'v')) return false;
  for (int i = 1; i <= 3; i++) {
    if (!isdigit((unsigned char)file[i])) return false;
  }
  return file.substring(4).equalsIgnoreCase(".PCM");
}

// "VOICE/<file>" (Default voice) or "VOICE/<PROFILE>/<file>". The profile name
// follows the same rule as the voice loader: letters, numbers, '_' and '-', up
// to 12 characters. Only those characters get through, so a path that passes
// can't climb out of /VOICE. Returns the profile in *profile ("" for Default).
static bool parseVoicePath(const String& path, String* profile) {
  if (!path.startsWith("VOICE/")) return false;
  String rest = path.substring(6);
  int slash = rest.indexOf('/');
  String prof = "";
  if (slash >= 0) {
    prof = rest.substring(0, slash);
    rest = rest.substring(slash + 1);
    if (prof.length() == 0 || prof.length() > 12) return false;
    for (size_t i = 0; i < prof.length(); i++) {
      char c = prof[i];
      if (!isalnum((unsigned char)c) && c != '_' && c != '-') return false;
    }
  }
  if (!isVoiceFileName(rest)) return false;
  if (profile != nullptr) *profile = prof;
  return true;
}

// The map the vario is currently reading from (selectedDemFile, settings.h,
// stored with a leading '/'). It's held open by the terrain lookup, so it
// can't be replaced or deleted from here.
static bool isSelectedMap(const String& name) {
  const char* sel = selectedDemFile;
  if (sel[0] == '/') sel++;
  return name.equalsIgnoreCase(sel);
}

static String htmlEscapeFs(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

static String sizeLabel(size_t bytes) {
  if (bytes >= 1024UL * 1024UL) return String((bytes + 524288UL) / 1048576UL) + " MB";
  return String((bytes + 512) / 1024) + " KB";
}

static void sendMessagePage(const String& title, const String& message) {
  String html;
  html += "<!doctype html><html><head><meta charset=\"utf-8\"><title>" + title + "</title>";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<style>body{font-family:sans-serif;margin:2em;max-width:480px}</style></head><body>";
  html += "<h1>" + title + "</h1><p>" + message + "</p><p><a href=\"/\">Back to files</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// Main page: flight recordings (download / delete), maps and airspace
// (delete / upload).
// ---------------------------------------------------------------------------

static void handleRoot() {
  lastActivityMs = millis();

  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    server.send(503, "text/plain", "SD card busy -- try again in a moment");
    return;
  }

  String igcRows;
  String mapRows;
  igcRows.reserve(2048);
  int igcCount = 0;
  int mapCount = 0;
  bool igcTruncated = false;

  File root = SD_MMC.open("/");
  if (root && root.isDirectory()) {
    File f = root.openNextFile();
    while (f) {
      String name = f.name();
      // File::name() has returned either a bare filename or a
      // leading-slash path depending on core version -- normalize.
      if (name.startsWith("/")) name.remove(0, 1);

      if (!f.isDirectory()) {
        String esc = htmlEscapeFs(name);
        if (isIgcName(name)) {
          if (igcCount >= FILE_SERVER_MAX_LISTED) {
            igcTruncated = true;
          } else {
            igcRows += "<li><a href=\"/download?file=" + esc + "\">" + esc + "</a> (" +
                       sizeLabel(f.size()) + ")";
            if (!igcRecording) {
              igcRows += " <form class=\"d\" method=\"POST\" action=\"/delete\" "
                         "onsubmit=\"return confirm('Delete " + esc + "?')\">"
                         "<input type=\"hidden\" name=\"file\" value=\"" + esc + "\">"
                         "<button>Delete</button></form>";
            }
            igcRows += "</li>";
            igcCount++;
          }
        } else if (isMapName(name) || isAirspaceName(name)) {
          mapRows += "<li>" + esc + " (" + sizeLabel(f.size()) + ")";
          if (isSelectedMap(name)) {
            mapRows += " <i>in use</i>";
          } else if (!igcRecording) {
            mapRows += " <form class=\"d\" method=\"POST\" action=\"/delete\" "
                       "onsubmit=\"return confirm('Delete " + esc + "?')\">"
                       "<input type=\"hidden\" name=\"file\" value=\"" + esc + "\">"
                       "<button>Delete</button></form>";
          }
          mapRows += "</li>";
          mapCount++;
        }
      }
      f = root.openNextFile();
    }
  }
  xSemaphoreGive(sdMutex);

  String html;
  html.reserve(igcRows.length() + mapRows.length() + 2048);
  html += "<!doctype html><html><head><meta charset=\"utf-8\"><title>Vario Files</title>";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<style>body{font-family:sans-serif;margin:2em;max-width:520px}"
          "li{margin:0.7em 0}a{text-decoration:none;color:#0645AD;font-size:1.1em}"
          "form.d{display:inline;margin-left:.6em}"
          "form.d button{color:#b00020;background:none;border:1px solid #b00020;"
          "border-radius:4px;padding:.1em .5em}"
          ".note{color:#666;font-size:.9em}</style>";
  html += "</head><body>";

  if (igcRecording) {
    html += "<p class=\"note\"><b>Recording in progress</b> &ndash; downloads, deleting and "
            "uploads are disabled until it stops.</p>";
  }

  html += "<h1>Flight Recordings</h1>";
  if (igcCount == 0) {
    html += "<p>No flight recordings found.</p>";
  } else {
    html += "<ul>" + igcRows;
    if (igcTruncated) html += "<li>... more recordings on the card are not listed</li>";
    html += "</ul>";
  }

  html += "<h1>Maps &amp; Airspace</h1>";
  if (mapCount == 0) {
    html += "<p>No .ADEM maps or AIRSPACE.txt found.</p>";
  } else {
    html += "<ul>" + mapRows + "</ul>";
  }

  if (!igcRecording) {
    html += "<h2>Upload a map or airspace file</h2>"
            "<form method=\"POST\" action=\"/upload\" enctype=\"multipart/form-data\" "
            "onsubmit=\"this.querySelector('button').disabled=true;"
            "this.querySelector('button').innerText='Uploading - keep this page open...'\">"
            "<input type=\"file\" name=\"file\" accept=\".ADEM,.adem,.txt\"> "
            "<button>Upload</button></form>"
            "<p class=\"note\">.ADEM terrain maps, or <b>AIRSPACE.txt</b> (replaces the current one). "
            "Large maps take several minutes over WiFi &ndash; do this on the ground. "
            "Choose a new map from the vario's <b>Map</b> menu afterwards.</p>";
  }

  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// Deletes one recording, map or airspace file from the SD root.
// ---------------------------------------------------------------------------

static void handleDelete() {
  lastActivityMs = millis();

  String filename = server.arg("file");
  if (!isSafeRootName(filename) ||
      !(isIgcName(filename) || isMapName(filename) || isAirspaceName(filename))) {
    sendMessagePage("Not deleted", "Only flight recordings, .ADEM maps and AIRSPACE.txt can be deleted here.");
    return;
  }
  if (igcRecording) {
    sendMessagePage("Not deleted", "A flight is being recorded &ndash; try again after landing.");
    return;
  }
  if (isMapName(filename) && isSelectedMap(filename)) {
    sendMessagePage("Not deleted", "That map is in use. Choose a different map in the vario's Map menu first.");
    return;
  }
  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    sendMessagePage("Not deleted", "The SD card is busy &ndash; try again in a moment.");
    return;
  }
  bool ok = SD_MMC.remove("/" + filename);
  xSemaphoreGive(sdMutex);

  Serial.printf("[EXPORT] Delete %s: %s\n", filename.c_str(), ok ? "ok" : "FAILED");
  if (ok) {
    server.sendHeader("Location", "/", true);  // straight back to the list
    server.send(303, "text/plain", "");
  } else {
    sendMessagePage("Not deleted", htmlEscapeFs(filename) + " could not be deleted.");
  }
}

// ---------------------------------------------------------------------------
// Upload of maps, airspace, the splash image and voice files. Written to a
// temporary file first and only swapped in once it has fully arrived, so an
// interrupted upload never leaves a half-written file behind.
//
// The page's form sends a bare filename, which is how maps and airspace
// arrive. ROY.BIN and voice files are for a script loading a whole card: it
// names the destination with /upload?path=VOICE/US_LESSAC/V000.PCM, because
// those files don't all live in the SD root.
// ---------------------------------------------------------------------------

// The WebServer library drops an upload after 5 s with no data, which a short
// pause on a phone hotspot can trigger, losing a large map most of the way
// through. Applies to this upload's connection only.
#define UPLOAD_CLIENT_TIMEOUT_MS 30000UL

static const char* UPLOAD_TEMP_PATH = "/UPLOAD.TMP";
static File uploadFile;
static String uploadTarget;   // final "/NAME" path, empty if rejected
static String uploadError;    // set when the upload is refused or fails
static size_t uploadBytes = 0;

static bool takeSd(uint32_t ms) {
  return sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(ms)) == pdTRUE;
}
static void giveSd() {
  if (sdMutex != nullptr) xSemaphoreGive(sdMutex);
}

// Only a selected map that's really on the card can be in use. The default map
// is selected even on a blank card, and has to be uploadable there. If the card
// is too busy to check, this says it's in use, which just refuses the upload.
static bool selectedMapIsOnCard(const String& name) {
  if (!takeSd(1000)) return true;
  bool onCard = SD_MMC.exists("/" + name);
  giveSd();
  return onCard;
}

static void handleUploadChunk() {
  lastActivityMs = millis();
  HTTPUpload& up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    uploadError = "";
    uploadTarget = "";
    uploadBytes = 0;
    server.client().setTimeout(UPLOAD_CLIENT_TIMEOUT_MS);

    String name;
    if (server.hasArg("path")) {
      name = server.arg("path");                 // destination named by a script
    } else {
      name = up.filename;
      int slash = name.lastIndexOf('/');         // some browsers send a path
      if (slash >= 0) name = name.substring(slash + 1);
      slash = name.lastIndexOf('\\');
      if (slash >= 0) name = name.substring(slash + 1);
    }
    String voiceProfile;
    bool isVoice = parseVoicePath(name, &voiceProfile);

    if (igcRecording) {
      uploadError = "A flight is being recorded &ndash; try again after landing.";
    } else if (!isVoice && (!isSafeRootName(name) ||
                            !(isMapName(name) || isAirspaceName(name) || isSplashName(name)))) {
      uploadError = "Only .ADEM maps, AIRSPACE.txt, ROY.BIN and VOICE files can be uploaded.";
    } else if (!isVoice && name.length() >= DEM_FILENAME_MAX_LEN - 1) {
      uploadError = "That file name is too long &ndash; rename it to under 30 characters.";
    } else if (isMapName(name) && isSelectedMap(name) && selectedMapIsOnCard(name)) {
      uploadError = "A map with that name is in use. Choose a different map in the Map menu first, "
                    "or rename the file.";
    }
    if (uploadError.length() > 0) return;

    uploadTarget = "/" + (isAirspaceName(name) ? String("AIRSPACE.txt")
                          : isSplashName(name) ? String("ROY.BIN") : name);
    if (!takeSd(1000)) {
      uploadError = "The SD card is busy &ndash; try again in a moment.";
      return;
    }
    if (isVoice) {
      // A new card has no voice folders yet, and the final rename needs them.
      if (!SD_MMC.exists("/VOICE")) SD_MMC.mkdir("/VOICE");
      if (voiceProfile.length() > 0) {
        String dir = "/VOICE/" + voiceProfile;
        if (!SD_MMC.exists(dir)) SD_MMC.mkdir(dir);
      }
    }
    SD_MMC.remove(UPLOAD_TEMP_PATH);
    uploadFile = SD_MMC.open(UPLOAD_TEMP_PATH, FILE_WRITE);
    giveSd();
    if (!uploadFile) uploadError = "Could not create the file on the SD card.";
    Serial.printf("[EXPORT] Upload started: %s\n", uploadTarget.c_str());

  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadError.length() > 0 || !uploadFile) return;
    if (!takeSd(2000)) {
      uploadError = "The SD card stayed busy during the upload.";
      return;
    }
    size_t written = uploadFile.write(up.buf, up.currentSize);
    giveSd();
    if (written != up.currentSize) {
      uploadError = "Writing to the SD card failed (is it full?).";
      return;
    }
    uploadBytes += written;

  } else if (up.status == UPLOAD_FILE_END || up.status == UPLOAD_FILE_ABORTED) {
    bool aborted = (up.status == UPLOAD_FILE_ABORTED);
    if (aborted && uploadError.length() == 0) uploadError = "The upload was interrupted.";

    if (!takeSd(2000)) {
      if (uploadError.length() == 0) uploadError = "The SD card is busy &ndash; try again.";
      return;
    }
    if (uploadFile) uploadFile.close();
    if (uploadError.length() == 0 && uploadBytes > 0) {
      SD_MMC.remove(uploadTarget);
      if (!SD_MMC.rename(UPLOAD_TEMP_PATH, uploadTarget)) {
        uploadError = "Could not move the uploaded file into place.";
      }
    } else if (uploadError.length() == 0) {
      uploadError = "The file was empty.";
    }
    if (uploadError.length() > 0) SD_MMC.remove(UPLOAD_TEMP_PATH);
    giveSd();
    Serial.printf("[EXPORT] Upload %s: %u bytes%s\n", uploadTarget.c_str(),
                  (unsigned)uploadBytes, uploadError.length() ? " (FAILED)" : "");
  }
}

static void handleUploadDone() {
  lastActivityMs = millis();
  if (uploadError.length() > 0 || uploadTarget.length() == 0) {
    sendMessagePage("Upload failed",
                    uploadError.length() ? uploadError : String("No file was received."));
    return;
  }
  String name = uploadTarget.substring(1);
  String msg = htmlEscapeFs(name) + " (" + sizeLabel(uploadBytes) + ") is on the SD card.";
  if (isMapName(name) && isSelectedMap(name)) {
    msg += " It's the selected map, so the vario will start using it.";
  } else if (isMapName(name)) {
    msg += " Choose it from the vario's <b>Map</b> menu to use it.";
  } else {
    msg += " Restart the vario to use it.";  // airspace, splash and voice load at start-up
  }
  sendMessagePage("Uploaded", msg);
}

// ---------------------------------------------------------------------------
// Plain-text list of what's on the card, one "path<TAB>size" per line, so a
// script loading a whole card can check every file arrived at its full size.
// Covers the root, /VOICE and /VOICE/<PROFILE>, the deepest the card uses.
// ---------------------------------------------------------------------------

#define FILE_LIST_MAX_ENTRIES 500

static void appendFileList(const char* dirPath, int depth, String& out, int& count) {
  File dir = SD_MMC.open(dirPath);
  if (!dir || !dir.isDirectory()) return;
  File f = dir.openNextFile();
  while (f && count < FILE_LIST_MAX_ENTRIES) {
    if (f.isDirectory()) {
      if (depth < 2) appendFileList(f.path(), depth + 1, out, count);
    } else {
      out += f.path();
      out += '\t';
      out += String((unsigned long)f.size());
      out += '\n';
      count++;
    }
    f = dir.openNextFile();
  }
}

static void handleList() {
  lastActivityMs = millis();
  if (!takeSd(1000)) {
    server.send(503, "text/plain", "SD card busy -- try again in a moment");
    return;
  }
  String out;
  out.reserve(6144);
  int count = 0;
  appendFileList("/", 0, out, count);
  giveSd();
  server.send(200, "text/plain", out);
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

  // Only flight recordings are served (the listing only shows .IGC files).
  if (!(filename.endsWith(".IGC") || filename.endsWith(".igc"))) {
    server.send(400, "text/plain", "Only .IGC files can be downloaded");
    return;
  }

  // Streaming a file holds the SD mutex for the whole transfer, which would
  // stall flight recording; never do that while a recording is open.
  if (igcRecording) {
    server.send(503, "text/plain", "Recording in progress -- try again after landing");
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

  static bool routesRegistered = false;  // routes persist across start/stop
  if (!routesRegistered) {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/download", HTTP_GET, handleDownload);
    server.on("/list", HTTP_GET, handleList);
    server.on("/delete", HTTP_POST, handleDelete);
    server.on("/upload", HTTP_POST, handleUploadDone, handleUploadChunk);
    server.onNotFound(handleNotFound);
    routesRegistered = true;
  }
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
