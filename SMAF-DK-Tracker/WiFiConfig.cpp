/**
* @file WiFiConfig.cpp
* @brief Declaration of the WiFi and MQTT configuration library.
*
* This file contains the declaration for the WiFiConfig library, which facilitates
* the indication of device status through a NeoPixel LED and audio feedback. The library provides separate functions
* to control the LED for displaying status in terms of colors, as well as functions to play various melodies for auditory feedback.
* It is designed to be easily integrated into Arduino projects for visualizing various device states.
*
* @license MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include "WiFiConfig.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Update.h>

AsyncWebServer server(80);  // HTTP server running on port 80.
AsyncWebSocket ws("/ws");   // WebSocket endpoint at /ws.
Preferences prefs;          // Preferences instance for reading/writing config.

// Global metadata
String deviceName = "";
String firmwareVersion = "";
String firmwareBuildDate = "";
String hardwareRevision = "";

/**
* Sets device metadata values shown on the configuration web interface.
*
* @param version Firmware version string.
* @param buildDate Firmware build date string.
* @param hwRev Hardware revision string.
*/
void setDeviceMetadata(const String &version, const String &buildDate, const String &hwRev) {
  firmwareVersion = version;
  firmwareBuildDate = buildDate;
  hardwareRevision = hwRev;
}

/**
* Handles incoming WebSocket messages based on the provided action.
* Supports configuration retrieval, saving new settings, and scanning for Wi-Fi networks.
*
* @param client Pointer to the WebSocket client that sent the message.
* @param data JSON-formatted message payload.
*/
void handleWebSocketMessage(AsyncWebSocketClient *client, String data) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, data);
  if (err) return;

  String action = doc["action"];

  if (action == "get_config") {
    JsonDocument response;
    response["action"] = "config_data";

    prefs.begin("wifi_config", true);
    response["ssidName"] = prefs.getString("ssidName", "");
    response["ssidPassword"] = prefs.getString("ssidPassword", "");
    response["mqttServer"] = prefs.getString("mqttServer", "");
    response["mqttServerPort"] = prefs.getInt("mqttServerPort", 1883);
    response["mqttUsername"] = prefs.getString("mqttUsername", "");
    response["mqttPassword"] = prefs.getString("mqttPassword", "");
    response["mqttClientId"] = prefs.getString("mqttClientId", "");
    response["mqttTopic"] = prefs.getString("mqttTopic", "");
    response["rgb"] = prefs.getBool("rgb", true);
    response["buzzer"] = prefs.getBool("buzzer", true);
    prefs.end();

    // Include global metadata.
    response["fw-version"] = firmwareVersion;
    response["fw-build-date"] = firmwareBuildDate;
    response["hw-revision"] = hardwareRevision;

    String json;
    serializeJson(response, json);
    client->text(json);
  }

  else if (action == "save_config") {
    prefs.begin("wifi_config", false);

    prefs.putString("ssidName", doc["ssidName"] | "");
    prefs.putString("ssidPassword", doc["ssidPassword"] | "");
    prefs.putString("mqttServer", doc["mqttServer"] | "");
    prefs.putInt("mqttServerPort", doc["mqttServerPort"] | 1883);
    prefs.putString("mqttUsername", doc["mqttUsername"] | "");
    prefs.putString("mqttPassword", doc["mqttPassword"] | "");
    prefs.putString("mqttClientId", doc["mqttClientId"] | "");
    prefs.putString("mqttTopic", doc["mqttTopic"] | "");
    prefs.putBool("rgb", doc["rgb"]);
    prefs.putBool("buzzer", doc["buzzer"]);
    prefs.end();

    client->text("{\"action\":\"save_ack\",\"status\":\"ok\"}");
    delay(1000);
    ESP.restart();
  }

  else if (action == "scan_wifi") {
    int n = WiFi.scanNetworks();
    JsonDocument response;
    response["action"] = "wifi_list";

    JsonArray ssids = response["ssids"].to<JsonArray>();
    for (int i = 0; i < n; ++i) {
      ssids.add(WiFi.SSID(i));
    }

    String json;
    serializeJson(response, json);
    client->text(json);
  }
}

/**
* Handles incoming WebSocket events.
* When data is received, it reconstructs the message and forwards it to the handler.
*
* @param server Pointer to the WebSocket server instance.
* @param client Pointer to the client sending the message.
* @param type Type of WebSocket event (connect, disconnect, data, etc.).
* @param arg Event-specific data (used for frame info in WS_EVT_DATA).
* @param data Pointer to the received data payload.
* @param len Length of the received data.
*/
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    String msg;
    for (size_t i = 0; i < len; i++) msg += (char)data[i];
    handleWebSocketMessage(client, msg);
  }
}

/**
* Starts the Wi-Fi access point and serves the configuration web interface.
* Mounts LittleFS, sets up WebSocket events, and serves HTML/CSS/JS files from storage.
*/
void setupWiFiConfig() {
  WiFi.softAP("SMAD-DK-SAP-Configuration", "0123456789");

  // Try to mount the LittleFS filesystem.
  // If it fails, print an error and stop setup by returning.
  // The 'true' argument will format the filesystem if mounting fails the first time.
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed.");
    return;
  }

  // Optional: list all files in the LittleFS filesystem
  // Uncomment to debug or verify the contents
  // File root = LittleFS.open("/");
  // Serial.println("Files in LittleFS:");
  // while (File file = root.openNextFile()) {
  //   Serial.println(file.name());
  // }

  // Try to start mDNS with the hostname "config".
  // This allows the device to be reached via 'http://config.local'
  // If it fails, print a warning but continue running the program.
  if (!MDNS.begin("config")) {
    Serial.println("mDNS setup failed. Continuing without mDNS.");
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Serve the HTML page.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", String(), false);
  });

  // Serve static files for web page.
  server.serveStatic("/style.css", LittleFS, "/style.css");
  server.serveStatic("/script.js", LittleFS, "/script.js");

  server.on(
    "/update", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      bool hasError = Update.hasError();

      request->send(200);  // Minimal and silent response

      // Restart after delay only if successful
      if (!hasError) {
        // Give time to flush and for browser to read response
        Serial.println("[OTA] Waiting 2 seconds before restart...");
        delay(2000);
        ESP.restart();
      }
    },
    [](AsyncWebServerRequest *request, String filename, size_t index,
       uint8_t *data, size_t len, bool final) {
      if (!index) {
        Serial.printf("\n[OTA] Start: %s\n", filename.c_str());

        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Serial.printf("[OTA] Begin failed: %s\n", Update.errorString());

          JsonDocument doc;
          doc["action"] = "ota_result";
          doc["status"] = "failure";
          doc["error"] = Update.errorString();

          String json;
          serializeJson(doc, json);
          ws.textAll(json);  // Notify all clients immediately
        }
      }

      if (Update.write(data, len) != len) {
        Serial.printf("[OTA] Write failed: %s\n", Update.errorString());
      } else {
        Serial.printf("[OTA] Written %u bytes at offset %u\n", len, index);
      }

      if (final) {
        if (Update.end(true)) {
          Serial.println("[OTA] Update completed successfully.");

          // Send WebSocket response to all connected clients
          JsonDocument doc;
          doc["action"] = "ota_result";
          doc["status"] = "success";

          String json;
          serializeJson(doc, json);
          ws.textAll(json);  // <--- sends to all connected WS clients
          delay(800);
        } else {
          Serial.printf("[OTA] Update failed: %s\n", Update.errorString());

          JsonDocument doc;
          doc["action"] = "ota_result";
          doc["status"] = "failure";
          doc["error"] = Update.errorString();

          String json;
          serializeJson(doc, json);
          ws.textAll(json);  // <--- notify failure
          delay(800);
        }
      }
    });

  server.begin();
}

/**
* Clears all stored Wi-Fi and MQTT configuration values from non-volatile storage.
* This removes all keys within the "wifi_config" namespace in Preferences.
*/
void clearWiFiConfig() {
  Preferences prefs;
  prefs.begin("wifi_config", false);  // false = write mode
  prefs.clear();                      // wipe all keys in this namespace
  prefs.end();
}

/**
* Loads Wi-Fi and MQTT configuration values from non-volatile storage (Preferences).
*
* @return WiFiConfig struct populated with stored configuration values.
*/
WiFiConfig loadWiFiConfig() {
  Preferences prefs;
  WiFiConfig config;

  prefs.begin("wifi_config", true);  // read-only
  config.ssidName = prefs.getString("ssidName", "");
  config.ssidPassword = prefs.getString("ssidPassword", "");
  config.mqttServer = prefs.getString("mqttServer", "");
  config.mqttServerPort = prefs.getInt("mqttServerPort", 1883);
  config.mqttUsername = prefs.getString("mqttUsername", "");
  config.mqttPassword = prefs.getString("mqttPassword", "");
  config.mqttClientId = prefs.getString("mqttClientId", "");
  config.mqttTopic = prefs.getString("mqttTopic", "");
  config.rgb = prefs.getBool("rgb", true);
  config.buzzer = prefs.getBool("buzzer", true);
  prefs.end();

  return config;
}