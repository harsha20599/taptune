// Book Piano · update mode (wireless firmware updates)
//
// Normal play never touches Wi-Fi. Update mode is entered on demand
// (web piano "Update mode" button = MIDI CC 115, or serial 'u'; the parent
// passes it on to the child). In update mode the board:
//   1. joins the home Wi-Fi saved on it and waits for new firmware (ArduinoOTA,
//      password-protected, host name book-piano-parent / book-piano-child), or
//   2. if no Wi-Fi is saved or it can't connect, opens its own hotspot
//      "BookPiano-Setup-<role>" with a page to type the Wi-Fi name and password.
// It goes back to play mode after an update, or after UPDATE_TIMEOUT_MS.

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <DNSServer.h>

// Wireless update password: put your own in ota_secret.h (copy
// ota_secret.example.h; ota_secret.h is git-ignored so it never gets published).
#if __has_include("ota_secret.h")
#include "ota_secret.h"
#endif
#ifndef OTA_PASSWORD
#define OTA_PASSWORD "bookpiano"  // public default: set your own (anyone on your Wi-Fi could update a board in update mode)
#warning "Using the default OTA password. Copy ota_secret.example.h to ota_secret.h and set your own."
#endif

const uint32_t UPDATE_TIMEOUT_MS = 10UL * 60 * 1000;
const uint32_t WIFI_JOIN_MS      = 15000;

bool updateMode = false;
uint32_t updateStartedMs = 0;
WebServer setupServer(80);
DNSServer setupDns;
bool setupPortal = false;

const char *hostName() { return IS_CHILD ? "book-piano-child" : "book-piano-parent"; }

void updateStatus(const char *state, const String &extra = "") {
#if HAS_SERIAL0
  for (Print *out : { (Print *)&Serial, (Print *)&Serial0 })  // both cables
#else
  for (Print *out : { (Print *)&Serial })
#endif
    out->printf("{\"ev\":\"update\",\"state\":\"%s\",\"host\":\"%s\"%s}\n", state, hostName(), extra.c_str());
}

void startOta() {
  MDNS.begin(hostName());
  ArduinoOTA.setHostname(hostName());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { updateStatus("receiving"); });
  ArduinoOTA.onEnd([]() { updateStatus("done-restarting"); });
  ArduinoOTA.onError([](ota_error_t e) { updateStatus("error", ",\"code\":" + String((int)e)); });
  ArduinoOTA.begin();
  updateStatus("ready", ",\"ip\":\"" + WiFi.localIP().toString() + "\"");
}

// ---- first-time Wi-Fi setup page (the board's own hotspot) ----
const char SETUP_PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Book Piano setup</title>
<style>body{font:16px system-ui,sans-serif;max-width:420px;margin:24px auto;padding:0 16px}
input,button{font:inherit;width:100%;padding:10px;margin:6px 0 14px;box-sizing:border-box}
button{background:#2f6fdb;color:#fff;border:0;border-radius:8px}</style></head><body>
<h2>Book Piano Wi-Fi</h2><p>Enter the Wi-Fi this board should join for updates.</p>
<form method="post" action="/save"><label>Wi-Fi name<input name="s" required></label>
<label>Password<input name="p" type="password"></label><button>Save and restart</button></form>
</body></html>)HTML";

void startSetupPortal() {
  setupPortal = true;
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  String ap = String("BookPiano-Setup-") + (IS_CHILD ? "Child" : "Parent");
  WiFi.softAP(ap.c_str());
  setupDns.start(53, "*", WiFi.softAPIP());  // every name → this board, so phones show the page
  setupServer.on("/", []() { setupServer.send_P(200, "text/html", SETUP_PAGE); });
  setupServer.on("/save", HTTP_POST, []() {
    String s = setupServer.arg("s"), p = setupServer.arg("p");
    if (!s.length()) { setupServer.send(400, "text/plain", "Wi-Fi name missing"); return; }
    prefs.begin("bookpiano", false);
    prefs.putString("wifi_s", s);
    prefs.putString("wifi_p", p);
    prefs.putBool("upd_boot", true);  // come back up in update mode to try the new Wi-Fi
    prefs.end();
    setupServer.send(200, "text/html", "<p style='font:16px system-ui'>Saved. The board restarts and joins your Wi-Fi.</p>");
    delay(500);
    ESP.restart();
  });
  setupServer.onNotFound([]() { setupServer.sendHeader("Location", "/"); setupServer.send(302); });
  setupServer.begin();
  updateStatus("setup-hotspot", ",\"ap\":\"" + ap + "\",\"ip\":\"" + WiFi.softAPIP().toString() + "\"");
}

void enterUpdateMode() {
  if (updateMode) return;
  releaseLocal();
  if (!IS_CHILD) releaseChild();
  updateMode = true;
  updateStartedMs = millis();
  esp_now_deinit();

  prefs.begin("bookpiano", true);
  String s = prefs.getString("wifi_s", ""), p = prefs.getString("wifi_p", "");
  prefs.end();

  if (s.length()) {
    updateStatus("joining-wifi");
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostName());
    WiFi.begin(s.c_str(), p.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_JOIN_MS) delay(100);
    if (WiFi.status() == WL_CONNECTED) { startOta(); return; }
    updateStatus("wifi-failed");
  }
  startSetupPortal();
}

void updateLoop() {
  if (setupPortal) { setupDns.processNextRequest(); setupServer.handleClient(); }
  else ArduinoOTA.handle();
  if (millis() - updateStartedMs > UPDATE_TIMEOUT_MS) {
    updateStatus("timeout-restarting");
    delay(100);
    ESP.restart();  // back to play mode
  }
}

// Called at boot: after saving new Wi-Fi in the setup page, come straight back
// into update mode so the board tries it.
bool bootIntoUpdateMode() {
  prefs.begin("bookpiano", false);
  bool b = prefs.getBool("upd_boot", false);
  if (b) prefs.remove("upd_boot");
  prefs.end();
  return b;
}
