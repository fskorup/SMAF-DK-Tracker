/**
* @file WiFiConfig.h
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

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <Arduino.h>

/**
* Stores configuration parameters for Wi-Fi and MQTT connection.
*
* Includes SSID credentials, MQTT server details, and user preferences
* for visual (RGB) and audio (buzzer) notifications.
*/
struct WiFiConfig {
  String ssidName;
  String ssidPassword;
  String mqttServer;
  int mqttServerPort;
  String mqttUsername;
  String mqttPassword;
  String mqttClientId;
  String mqttTopic;
  bool rgb;
  bool buzzer;
};

// Global metadata used on the configuration web page.
extern String firmwareVersion;
extern String firmwareBuildDate;
extern String hardwareRevision;

/**
* Sets device metadata values shown on the configuration web interface.
*
* @param version Firmware version string.
* @param buildDate Firmware build date string.
* @param hwRev Hardware revision string.
*/
void setDeviceMetadata(const String& version, const String& buildDate, const String& hwRev);

/**
* Starts the Wi-Fi access point and serves the configuration web interface.
* Mounts LittleFS, sets up WebSocket events, and serves HTML/CSS/JS files from storage.
*/
void setupWiFiConfig();

/**
* Clears all stored Wi-Fi and MQTT configuration values from non-volatile storage.
* This removes all keys within the "wifi_config" namespace in Preferences.
*/
void clearWiFiConfig();

/**
* Loads Wi-Fi and MQTT configuration values from non-volatile storage (Preferences).
*
* @return WiFiConfig struct populated with stored configuration values.
*/
WiFiConfig loadWiFiConfig();

#endif  // WIFI_CONFIG_H