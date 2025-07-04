/**
* @file Structs.h
* @brief Declaration of structs used in main ino file.
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

#ifndef STRUCTS_H
#define STRUCTS_H

#include "Arduino.h"

struct PositioningData {
  bool gnssFixOk;
  uint8_t satellitesInRange;
  int32_t latitude;
  int32_t longitude;
  int32_t speed;
  int32_t heading;
  int32_t altitude;
  String timestamp;
};

struct AccelerometerData {
  float x;
  float y;
  float z;
  float magnitude;
};

struct GyroscopeData {
  float x;
  float y;
  float z;
};

struct PreferencesData {
  String networkName;
  String networkPassword;
  String mqttServerAddress;
  String mqttUsername;
  String mqttPassword;
  String mqttClientId;
  String mqttTopic;
  uint16_t mqttServerPort = 0;
  bool visualNotifications = false;
  bool audioNotifications = false;
};

#endif