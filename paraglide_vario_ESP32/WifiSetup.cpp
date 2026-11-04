#include "WifiSetup.h"
#include "wifi_manager.h"  // addSavedWifiNetwork(), selectWifiNetwork(), wifiConnected
#include "settings.h"      // wifiEnabled
#include "FileServer.h"    // isFileServerRunning(), stopFileServer()
#include "menu.h"          // displayDirty

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <string.h>

// ---------------------------------------------------------------------------
// State shared between the menu (Core 1) and the portal task (Core 0).
// ---------------------------------------------------------------------------
static volatile bool setupActive = false;
static volatile bool stopRequested = false;
static volatile WifiSetupState setupState = WIFI_SETUP_IDLE;
static char savedSsidBuf[WIFI_SSID_MAX_LEN + 1] = "";
static TaskHandle_t setupTaskHandle = nullptr;

// Everything below is only touched from inside the portal task.
static WebServer setupServer(80);
static DNSServer dnsServer;
static unsigned long lastActivityMs = 0;
static int pendingSaveSlot = -1;
static unsigned long saveTimeMs = 0;
static bool rescanInProgress = false;
static bool serversStarted = false;

// How long the "Saved" page stays reachable before the hotspot closes,
// so the phone actually receives the confirmation.
#define WIFI_SETUP_CLOSE_DELAY_MS 3000UL

// ---------------------------------------------------------------------------
// Nearby networks, from the scan done just before the hotspot comes up
// (or a rescan requested from the page). Strongest first, no duplicates.
// ---------------------------------------------------------------------------
#define WIFI_SETUP_MAX_SCAN 20

struct ScanEntry {
  char ssid[WIFI_SSID_MAX_LEN + 1];
  int32_t rssi;
  bool open;
};

static ScanEntry scanList[WIFI_SETUP_MAX_SCAN];
static uint8_t scanCount = 0;

static void collectScanResults(int found) {
  scanCount = 0;
  for (int i = 0; i < found; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0 || ssid.length() > WIFI_SSID_MAX_LEN) continue;  // hidden / invalid
    if (ssid == WIFI_SETUP_AP_NAME) continue;

    int32_t rssi = WiFi.RSSI(i);
    bool open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);

    // Same SSID from several access points -- keep the strongest.
    int existing = -1;
    for (uint8_t j = 0; j < scanCount; j++) {
      if (strcmp(scanList[j].ssid, ssid.c_str()) == 0) { existing = j; break; }
    }
    if (existing >= 0) {
      if (rssi > scanList[existing].rssi) scanList[existing].rssi = rssi;
      continue;
    }
    if (scanCount >= WIFI_SETUP_MAX_SCAN) continue;

    strlcpy(scanList[scanCount].ssid, ssid.c_str(), sizeof(scanList[scanCount].ssid));
    scanList[scanCount].rssi = rssi;
    scanList[scanCount].open = open;
    scanCount++;
  }

  // Strongest signal first (simple insertion sort -- at most 20 items).
  for (uint8_t i = 1; i < scanCount; i++) {
    ScanEntry key = scanList[i];
    int j = i - 1;
    while (j >= 0 && scanList[j].rssi < key.rssi) {
      scanList[j + 1] = scanList[j];
      j--;
    }
    scanList[j + 1] = key;
  }
  WiFi.scanDelete();
  Serial.printf("[WIFI SETUP] Scan found %u networks\n", scanCount);
}

// ---------------------------------------------------------------------------
// HTML helpers
// ---------------------------------------------------------------------------
static String htmlEscape(const char* in) {
  String out;
  for (const char* p = in; *p; p++) {
    switch (*p) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += *p; break;
    }
  }
  return out;
}

static const char* signalLabel(int32_t rssi) {
  if (rssi >= -60) return "Strong";
  if (rssi >= -72) return "Good";
  if (rssi >= -82) return "Weak";
  return "Very weak";
}

static void appendPageHead(String& html, const char* title, bool autoRefresh) {
  html += "<!doctype html><html><head><meta charset=\"utf-8\"><title>";
  html += title;
  html += "</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  if (autoRefresh) html += "<meta http-equiv=\"refresh\" content=\"3\">";
  html += "<style>"
          "body{font-family:sans-serif;margin:1.2em;max-width:480px}"
          "h1{font-size:1.4em}"
          ".net{display:block;padding:.7em;margin:.4em 0;border:1px solid #bbb;border-radius:8px}"
          ".net small{color:#666;float:right}"
          "input[type=text],input[type=password]{width:100%;padding:.6em;font-size:1em;box-sizing:border-box}"
          "button{width:100%;padding:.8em;font-size:1.1em;margin-top:1em;border-radius:8px;"
          "border:0;background:#0b5ed7;color:#fff}"
          ".note{color:#666;font-size:.9em}"
          ".err{color:#b00020}"
          "</style></head><body>";
}

// ---------------------------------------------------------------------------
// Pages
// ---------------------------------------------------------------------------
static void handleRoot() {
  lastActivityMs = millis();

  String html;
  html.reserve(4096);
  appendPageHead(html, "Vario WiFi Setup", rescanInProgress);
  html += "<h1>Vario WiFi Setup</h1>";

  if (rescanInProgress) {
    html += "<p>Scanning for networks&hellip; this page will refresh.</p></body></html>";
    setupServer.send(200, "text/html", html);
    return;
  }

  html += "<form method=\"POST\" action=\"/save\">";
  html += "<p>Choose a network:</p>";

  if (scanCount == 0) {
    html += "<p class=\"note\">No networks found nearby.</p>";
  }
  for (uint8_t i = 0; i < scanCount; i++) {
    String ssid = htmlEscape(scanList[i].ssid);
    html += "<label class=\"net\"><input type=\"radio\" name=\"ssid\" value=\"";
    html += ssid;
    html += "\"";
    if (i == 0) html += " checked";
    html += "> ";
    html += ssid;
    html += "<small>";
    html += signalLabel(scanList[i].rssi);
    if (scanList[i].open) html += ", open";
    html += "</small></label>";
  }

  html += "<label class=\"net\"><input type=\"radio\" name=\"ssid\" value=\"__other__\"";
  if (scanCount == 0) html += " checked";
  html += "> Other / hidden network:"
          "<input type=\"text\" name=\"ssid_manual\" maxlength=\"32\" "
          "autocapitalize=\"none\" autocorrect=\"off\" placeholder=\"Network name\"></label>";

  html += "<p>Password:</p>"
          "<input type=\"password\" id=\"pw\" name=\"pass\" maxlength=\"63\" "
          "autocapitalize=\"none\" autocorrect=\"off\">"
          "<p><label><input type=\"checkbox\" "
          "onclick=\"document.getElementById('pw').type=this.checked?'text':'password'\">"
          " Show password</label></p>"
          "<p class=\"note\">Leave blank for an open network. Phone hotspots must be set to "
          "2.4&nbsp;GHz &ndash; the vario can't use 5&nbsp;GHz.</p>"
          "<button type=\"submit\">Save &amp; connect</button></form>"
          "<p><a href=\"/rescan\">Scan again</a></p>"
          "</body></html>";

  setupServer.send(200, "text/html", html);
}

static void handleRescan() {
  lastActivityMs = millis();
  if (!rescanInProgress) {
    // Async, so the page (and the hotspot) stay responsive meanwhile.
    if (WiFi.scanNetworks(/*async=*/true) == WIFI_SCAN_FAILED) {
      Serial.println("[WIFI SETUP] Rescan failed to start");
    } else {
      rescanInProgress = true;
    }
  }
  setupServer.sendHeader("Location", "/", true);
  setupServer.send(302, "text/plain", "");
}

static void sendErrorPage(const char* message) {
  String html;
  appendPageHead(html, "Vario WiFi Setup", false);
  html += "<h1>Not saved</h1><p class=\"err\">";
  html += message;
  html += "</p><p><a href=\"/\">Go back and try again</a></p></body></html>";
  setupServer.send(200, "text/html", html);
}

static void handleSave() {
  lastActivityMs = millis();
  if (pendingSaveSlot >= 0) {  // double-tapped Save -- already done
    sendErrorPage("A network has already been saved. The vario is connecting to it.");
    return;
  }

  String ssid = setupServer.arg("ssid");
  if (ssid == "__other__") {
    ssid = setupServer.arg("ssid_manual");
    ssid.trim();
  }
  String pass = setupServer.arg("pass");

  if (ssid.length() == 0) {
    sendErrorPage("Please choose a network or type its name.");
    return;
  }
  if (ssid.length() > WIFI_SSID_MAX_LEN) {
    sendErrorPage("That network name is too long (32 characters maximum).");
    return;
  }
  if (pass.length() != 0 && (pass.length() < 8 || pass.length() > 63)) {
    sendErrorPage("WiFi passwords are 8 to 63 characters. Leave it blank only for an open network.");
    return;
  }

  int slot = addSavedWifiNetwork(ssid.c_str(), pass.c_str());
  if (slot < 0) {
    sendErrorPage("The network could not be saved. Please check the name and password.");
    return;
  }

  strlcpy(savedSsidBuf, ssid.c_str(), sizeof(savedSsidBuf));
  pendingSaveSlot = slot;
  saveTimeMs = millis();
  setupState = WIFI_SETUP_SAVED;
  displayDirty = true;

  String html;
  appendPageHead(html, "Vario WiFi Setup", false);
  html += "<h1>Saved</h1><p><b>";
  html += htmlEscape(ssid.c_str());
  html += "</b> has been saved on the vario.</p>"
          "<p>The Vario-Setup hotspot will now close and the vario will connect to this "
          "network. Your phone will go back to its normal WiFi.</p>"
          "<p class=\"note\">If it doesn't connect, check the password and run "
          "Add Network again &ndash; saving the same network updates its password.</p>"
          "</body></html>";
  setupServer.send(200, "text/html", html);
}

// Anything else -- including the "is there internet?" checks phones make
// when they join a WiFi network -- is redirected to the setup page. That
// redirect is what makes most phones pop the page up automatically.
static void handleRedirect() {
  lastActivityMs = millis();
  setupServer.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  setupServer.send(302, "text/plain", "");
}

// ---------------------------------------------------------------------------
// Portal task (Core 0)
// ---------------------------------------------------------------------------
static void finishSetup() {
  if (serversStarted) {
    setupServer.stop();
    dnsServer.stop();
    serversStarted = false;
  }
  WiFi.softAPdisconnect(true);
  WiFi.scanDelete();
  WiFi.mode(WIFI_STA);

  int slot = pendingSaveSlot;
  setupActive = false;  // hand the radio back to wifi_manager

  if (slot >= 0) {
    Serial.printf("[WIFI SETUP] Closed -- connecting to '%s'\n", savedSsidBuf);
    // Also turns WiFi on if it was switched off (Connections > WiFi).
    selectWifiNetwork((uint8_t)slot);
  } else if (wifiEnabled) {
    Serial.println("[WIFI SETUP] Closed without saving -- resuming previous network");
    startWifiConnect();
  } else {
    Serial.println("[WIFI SETUP] Closed without saving -- WiFi stays off");
    WiFi.mode(WIFI_OFF);
  }

  if (setupState != WIFI_SETUP_SAVED && setupState != WIFI_SETUP_FAILED) {
    setupState = WIFI_SETUP_IDLE;
  }
  displayDirty = true;
}

static void setupTask(void*) {
  // 1. Scan while still a plain station -- the most reliable time to do it.
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  vTaskDelay(pdMS_TO_TICKS(100));
  int found = WiFi.scanNetworks();
  if (found > 0) {
    collectScanResults(found);
  } else {
    scanCount = 0;
    WiFi.scanDelete();
  }

  // 2. Bring up the open hotspot (station side kept on for rescans).
  WiFi.mode(WIFI_AP_STA);
  if (!WiFi.softAP(WIFI_SETUP_AP_NAME)) {
    Serial.println("[WIFI SETUP] Could not start the hotspot");
    setupState = WIFI_SETUP_FAILED;
    finishSetup();
    setupTaskHandle = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(200));
  IPAddress apIp = WiFi.softAPIP();

  // 3. DNS answers every lookup with our own address (captive portal),
  //    then the web server serves the setup page.
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", apIp);

  static bool routesRegistered = false;  // routes persist across sessions
  if (!routesRegistered) {
    setupServer.on("/", HTTP_GET, handleRoot);
    setupServer.on("/save", HTTP_POST, handleSave);
    setupServer.on("/rescan", HTTP_GET, handleRescan);
    setupServer.onNotFound(handleRedirect);
    routesRegistered = true;
  }
  setupServer.begin();
  serversStarted = true;

  setupState = WIFI_SETUP_WAITING;
  displayDirty = true;
  lastActivityMs = millis();
  Serial.printf("[WIFI SETUP] Hotspot '%s' up -- browse to http://%s/\n",
                WIFI_SETUP_AP_NAME, apIp.toString().c_str());

  unsigned long lastStationCheck = 0;

  while (!stopRequested) {
    dnsServer.processNextRequest();
    setupServer.handleClient();

    unsigned long now = millis();

    if (rescanInProgress) {
      int result = WiFi.scanComplete();
      if (result >= 0) {
        collectScanResults(result);
        rescanInProgress = false;
      } else if (result == WIFI_SCAN_FAILED) {
        rescanInProgress = false;
      }
    }

    // Show "phone connected" in the menu once someone joins the hotspot.
    if (setupState != WIFI_SETUP_SAVED && now - lastStationCheck >= 500) {
      lastStationCheck = now;
      WifiSetupState next = (WiFi.softAPgetStationNum() > 0)
                                ? WIFI_SETUP_PHONE_JOINED
                                : WIFI_SETUP_WAITING;
      if (next != setupState) {
        setupState = next;
        displayDirty = true;
      }
    }

    if (pendingSaveSlot >= 0 && now - saveTimeMs >= WIFI_SETUP_CLOSE_DELAY_MS) break;

    if (pendingSaveSlot < 0 && now - lastActivityMs >= WIFI_SETUP_TIMEOUT_MS) {
      Serial.println("[WIFI SETUP] Timed out with nothing saved");
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }

  finishSetup();
  setupTaskHandle = nullptr;
  vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void startWifiSetup() {
  if (setupActive) return;

  // Export Files also serves on port 80 -- only one can run at a time.
  if (isFileServerRunning()) stopFileServer();

  savedSsidBuf[0] = '\0';
  pendingSaveSlot = -1;
  rescanInProgress = false;
  stopRequested = false;
  wifiConnected = false;
  setupState = WIFI_SETUP_STARTING;
  setupActive = true;
  displayDirty = true;

  BaseType_t ok = xTaskCreatePinnedToCore(setupTask, "wifiSetup", 8192, nullptr, 1,
                                          &setupTaskHandle, 0);
  if (ok != pdPASS) {
    Serial.println("[WIFI SETUP] Could not create the setup task");
    setupTaskHandle = nullptr;
    setupActive = false;
    setupState = WIFI_SETUP_FAILED;
    if (wifiEnabled) startWifiConnect();
  }
}

void stopWifiSetup() {
  if (!setupActive) return;
  stopRequested = true;  // the task closes everything down itself
}

bool isWifiSetupActive() {
  return setupActive;
}

WifiSetupState wifiSetupState() {
  return setupState;
}

const char* wifiSetupSavedSsid() {
  return savedSsidBuf;
}
