/**
* @file SMAF-DK-Tracker.ino
* @brief Main Arduino sketch for the SMAF-DK-Tracker project.
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

#include "WiFi.h"
#include "ArduinoMqttClient.h"
#include "AudioVisualNotifications.h"
#include "WiFiConfig.h"
#include "Helpers.h"
#include "SparkFun_u-blox_GNSS_v3.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "ArduinoJson.h"
#include "Structs.h"

// Define constants for ESP32 core numbers.
#define ESP32_CORE_PRIMARY 0    // Numeric value representing the primary core.
#define ESP32_CORE_SECONDARY 1  // Numeric value representing the secondary core.

// Pin assignments for I2C.
const struct {
  const int SDA = 1;  // IO01, SDA
  const int SCL = 2;  // IO02, SCL
} I2CPins;

// Pin assignments for outputs.
const struct {
  const int NeoPixel = 4;  // IO04, ADC1-CH0
  const int Speaker = 5;   // IO05, ADC1-CH4
} OutputPins;

// Pin assignments for inputs.
const struct {
  const int UserButton = 6;  // IO06
} InputPins;

// Enum to represent different device statuses.
enum DeviceStatusEnum : byte {
  NOT_READY,        // Device is not ready.
  READY_TO_SEND,    // Device is ready to send data.
  WAITING_GNSS,     // Device is waiting for GNSS data.
  MAINTENANCE_MODE  // Device is in maintenance mode.
};

// Variable to store the current device status.
DeviceStatusEnum deviceStatus = NOT_READY;  // Initial state is set to NONE.

// Function prototype for the DeviceStatusThread function.
void DeviceStatusThread(void* pvParameters);

/**
* @brief WiFiClient and PubSubClient instances for establishing MQTT communication.
* 
* The WiFiClient instance, named wifiClient, is used to manage the Wi-Fi connection.
* The PubSubClient instance, named mqtt, relies on the WiFiClient for MQTT communication.
*/
WiFiClient wifiClient;              // Manages Wi-Fi connection.
MqttClient mqttClient(wifiClient);  // Uses WiFiClient for MQTT communication.

/**
* @brief Constructs an instance of the AudioVisualNotifications class.
*
* Initializes an instance of the AudioVisualNotifications class with the provided configurations.
* The NeoPixel pin should be set up as OUTPUT before calling this constructor.
*
* @param neoPixelPin The pin connected to the NeoPixel LED strip.
* @param neoPixelCount The number of NeoPixels in the LED strip.
* @param neoPixelBrightness The brightness level of the NeoPixels (0-255).
* @param speakerPin The pin connected to the speaker for audio feedback.
*/
AudioVisualNotifications notifications(OutputPins.NeoPixel, 2, 40, OutputPins.Speaker);

// GNSS/GPS (U-Blox SAM-M10Q) library.
SFE_UBLOX_GNSS gnss;

// IMU (Bosch BMI270) library.
BMI270 imu;

// Structs definition from 'Struct.h' file.
PreferencesData preferencesData;
AccelerometerData accelerometerData;
GyroscopeData gyroscopeData;
PositioningData actualPositioningData;
PositioningData fallbackPositioningData;

/**
* @brief Initializes the SMAF-Development-Kit and runs once at the beginning.
*
* This function is responsible for the initial setup of the SMAF-Development-Kit.
* It is executed once when the Arduino board starts or is reset.
*
*/
void setup() {
  // Create a new task (DeviceStatusThread) and assign it to the primary core (ESP32_CORE_PRIMARY).
  xTaskCreatePinnedToCore(
    DeviceStatusThread,    // Function to implement the task.
    "DeviceStatusThread",  // Name of the task.
    8000,                  // Stack size in words.
    NULL,                  // Task input parameter (e.g., delay).
    1,                     // Priority of the task.
    NULL,                  // Task handle.
    ESP32_CORE_SECONDARY   // Core where the task should run.
  );

  // Initialize serial communication at a baud rate of 115200.
  Serial.begin(115200);

  // Set Wire library custom I2C pins and start the Wire library.
  // Example usage: Wire.setPins(SDA_PIN_NUMBER, SCL_PIN_NUMBER);
  Wire.setPins(I2CPins.SDA, I2CPins.SCL);
  Wire.begin();

  // Set the pin mode for the configuration button to INPUT.
  pinMode(InputPins.UserButton, INPUT);

  // Load and check configuration.
  WiFiConfig config = loadWiFiConfig();

  preferencesData.networkName = config.ssidName;
  preferencesData.networkPassword = config.ssidPassword;
  preferencesData.mqttServerAddress = config.mqttServer;
  preferencesData.mqttUsername = config.mqttUsername;
  preferencesData.mqttPassword = config.mqttPassword;
  preferencesData.mqttClientId = config.mqttClientId;
  preferencesData.mqttServerPort = config.mqttServerPort;
  preferencesData.mqttTopic = config.mqttTopic;
  preferencesData.visualNotifications = config.rgb ? true : false;
  preferencesData.audioNotifications = config.buzzer ? true : false;

  // Check and store flag if configuration is valid.
  static bool isConfigurationValid = preferencesData.networkName.length() > 0 && preferencesData.mqttServerAddress.length() > 0 && preferencesData.mqttClientId.length() > 0 && preferencesData.mqttTopic.length() > 0 && preferencesData.mqttServerPort > 0;

  // Initialize visualization library neo pixels.
  // This does not light up neo pixels.
  notifications.visual.initializePixels();

  // Play intro melody on speaker if enabled in preferences.
  if (preferencesData.audioNotifications) {
    notifications.audio.introMelody();
  }

  // Delay to stabilise things.
  delay(1600);

  // Print a formatted welcome message with build information.
  String firmwareVersion = "v0.005";
  String firmwareBuildDate = "Q3, 2025.";
  String hardwareRevision = "R01";

  setDeviceMetadata(firmwareVersion, firmwareBuildDate, hardwareRevision);
  Serial.printf("\n\rSMAF-DK-Tracker, Crafted with love in Europe.\n\rFirmware version: %s\n\rFirmware build date: %s\n\rHardware revison: %s\n\r\n\r", firmwareVersion, firmwareBuildDate, hardwareRevision);

  // Print loaded configuration data.
  debug(CMD, "Loading WiFi/MQTT Configuration");
  debug(SCS, "Loaded WiFi/MQTT Configuration");
  debug(LOG, "SSID Name: %s", preferencesData.networkName.c_str());
  debug(LOG, "SSID Password: %s", preferencesData.networkPassword.c_str());
  debug(LOG, "MQTT Server: %s", preferencesData.mqttServerAddress.c_str());
  debug(LOG, "MQTT Port: %s", String(preferencesData.mqttServerPort));
  debug(LOG, "MQTT Username: %s", preferencesData.mqttUsername.c_str());
  debug(LOG, "MQTT Password: %s", preferencesData.mqttPassword.c_str());
  debug(LOG, "MQTT Client ID: %s", preferencesData.mqttClientId.c_str());
  debug(LOG, "MQTT Topic: %s", preferencesData.mqttTopic.c_str());
  debug(LOG, "Visual Notifications: %s", String(preferencesData.visualNotifications ? "true" : "false"));
  debug(LOG, "Audio Notifications: %s", String(preferencesData.audioNotifications ? "true" : "false"));

  // Print status of configuration data.
  if (isConfigurationValid) {
    debug(SCS, "Configuration is valid. All required configuration data is populated.");
  } else {
    debug(ERR, "Configuration is incomplete. Some required fields are missing.");
  }

  // Trigger configuration if config button pressed or configuration is invalid.
  if ((digitalRead(InputPins.UserButton) == LOW) || (!isConfigurationValid)) {
    debug(CMD, "Starting WiFi configuration.");

    // Set device status to Maintenance Mode.
    deviceStatus = MAINTENANCE_MODE;

    // Start configuration server.
    setupWiFiConfig();

    // Play maintenance melody on speaker even if disabled in preferences.
    notifications.audio.maintenanceMelody();

    // Block here until config is done and ESP restarts.
    while (true) {
      vTaskDelay(80 / portTICK_PERIOD_MS);
    }
  }

  // Set device status to notready mode.
  deviceStatus = NOT_READY;

  // Start IMU.
  while (imu.beginI2C(BMI2_I2C_PRIM_ADDR) != BMI2_OK) {
    debug(ERR, "IMU not found on I2C bus. Check wiring.");
    delay(800);
  }

  // Log successful IMU initialization.
  debug(SCS, "IMU module detected on I2C lines.");

  // Start GNSS/GPS.
  while (!gnss.begin()) {
    debug(ERR, "GNSS module not detected on I2C lines.");
    delay(800);
  }

  // Log successful GNSS/GPS initialization.
  debug(SCS, "GNSS module detected on I2C lines.");

  // Set the I2C port to output UBX only for GNSS/GPS (turn off NMEA noise).
  gnss.setI2COutput(COM_TYPE_UBX);

  // Setup hardware Watchdog timer. Bark Bark.
  initWatchdog(30, true);
}

/**
* @brief Main execution loop for the SMAF-Development-Kit.
*
* This function runs repeatedly in a loop after the initial setup.
* It is the core of your Arduino program, where continuous tasks and operations should be placed.
* Be mindful of keeping the loop efficient and avoiding long blocking operations.
*
*/
void loop() {
  // Attempt to connect to the Wi-Fi network.
  connectToNetwork();

  // Attempt to connect to the MQTT broker.
  connectToMqttBroker();

  // Get measurements from the IMU sensor. This must be called before accessing the sensor data, otherwise it will never update.
  imu.getSensorData();

  // Accelerometer data.
  accelerometerData.x = imu.data.accelX;
  accelerometerData.y = imu.data.accelY;
  accelerometerData.z = imu.data.accelZ;
  // accelerometerData.magnitude = sqrt(accelerometerData.x * accelerometerData.x + accelerometerData.y * accelerometerData.y + accelerometerData.z * accelerometerData.z);
  accelerometerData.magnitude = sqrt(accelerometerData.x * accelerometerData.x + accelerometerData.y * accelerometerData.y);

  // Gyroscope data.
  gyroscopeData.x = imu.data.gyroX;
  gyroscopeData.y = imu.data.gyroY;
  gyroscopeData.z = imu.data.gyroZ;

  // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (gnss.getPVT() == true) {
    actualPositioningData.gnssFixOk = gnss.getGnssFixOk();
    actualPositioningData.satellitesInRange = gnss.getSIV();
    actualPositioningData.latitude = gnss.getLatitude();
    actualPositioningData.longitude = gnss.getLongitude();
    actualPositioningData.speed = gnss.getGroundSpeed();
    actualPositioningData.heading = gnss.getHeading();
    actualPositioningData.altitude = gnss.getAltitudeMSL();
    actualPositioningData.timestamp = getUtcTimeString();

    // If the device is ready to send, publish a message to the MQTT broker.
    if (actualPositioningData.gnssFixOk && actualPositioningData.latitude != 0 && actualPositioningData.longitude != 0) {
      deviceStatus = READY_TO_SEND;
      debug(LOG, "Device is ready to post messages, %d satellites locked.", actualPositioningData.satellitesInRange);
      debug(CMD, "Posting new message to MQTT broker '%s' on topic '%s'.", preferencesData.mqttServerAddress.c_str(), preferencesData.mqttTopic.c_str());

      // Construct MQTT message ready for publishing.
      String mqttData = constructMqttMessage(actualPositioningData, accelerometerData, gyroscopeData);
      debug(LOG, "Message: %s", mqttData.c_str());

      mqttClient.beginMessage(preferencesData.mqttTopic.c_str(), true);  // true = retain flag.
      mqttClient.print(mqttData);                                        // Send as string.
      mqttClient.endMessage();                                           // End message.
    } else {
      deviceStatus = WAITING_GNSS;
      debug(LOG, "Device is not ready to post messages. Searching for satellites, %d locked.", actualPositioningData.satellitesInRange);
    }
  }

  // Check for incoming data on defined MQTT topic.
  // This is hard core connection check.
  // If no data on topic is received, we are not connected to internet or server and watchdog will reset the device.
  mqttClient.poll();
}

/**
* @brief Handles the server response received on a specific MQTT topic.
*
* This function logs the server response using debug output. If the device status is not
* in maintenance mode, it also resets the watchdog timer to prevent system reset.
*
* @param topic The MQTT topic on which the server response was received.
* @param payload Pointer to the payload data received from the server.
* @param length Length of the payload data.
*/
void onMqttMessage(int messageSize) {
  debug(LOG, "Server '%s' responded with %s message on '%s' topic.", preferencesData.mqttServerAddress.c_str(), (mqttClient.messageRetain() ? "RETAINED" : "NEW"), mqttClient.messageTopic().c_str());

  // Read the payload from the client
  String payload;
  while (mqttClient.available()) {
    char c = (char)mqttClient.read();
    payload += c;
  }

  debug(LOG, "Payload from server: %s", payload.c_str());

  if (mqttClient.messageRetain()) {
    /*
    JsonDocument jsonDocument;
    DeserializationError deserializationError = deserializeJson(jsonDocument, payload);

    if (!deserializationError) {
      debug(LOG, "Received MQTT message is valid JSON.");

      // Modify a key.
      // jsonDocument["isStale"] = true;

      // Serialize back to string.
      //  String modifiedJsonData = String();  // clear previous.
      // serializeJson(jsonDocument, modifiedJsonData);

      // Print modified result.
      // debug(LOG, "Modified JSON: %s", modifiedJsonData.c_str());
    } else {
      debug(ERR, "Received MQTT message was not valid JSON.");
    }
    */
  }

  // Reset WDT.
  if (deviceStatus != MAINTENANCE_MODE) {
    resetWatchdog();
  }
}

/**
* @brief Attempt to connect SMAF-DK to the configurationured Wi-Fi network.
*
* If SMAF-DK is not connected to the Wi-Fi network, this function tries to establish
* a connection using the settings from the WiFiconfiguration instance.
*
* @warning This function may delay for extended periods while attempting to connect
* to the Wi-Fi network.
*/
void connectToNetwork() {
  if (WiFi.status() != WL_CONNECTED) {
    // Set initial device status.
    deviceStatus = NOT_READY;

    // Disable auto-reconnect and set Wi-Fi mode to station mode.
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA);

    // Log an error if not connected to the configurationured SSID.
    debug(ERR, "Device not connected to '%s'.", preferencesData.networkName.c_str());

    // Keep attempting to connect until successful.
    while (WiFi.status() != WL_CONNECTED) {
      debug(CMD, "Connecting device to '%s'", preferencesData.networkName.c_str());

      // Attempt to connect to the Wi-Fi network using configurationured credentials.
      WiFi.begin(preferencesData.networkName.c_str(), preferencesData.networkPassword.c_str());
      delay(6400);
    }

    // Log successful connection and set device status.
    debug(SCS, "Device connected to '%s'.", preferencesData.networkName.c_str());
  }
}

/**
* @brief Attempt to connect to the configurationured MQTT broker.
*
* If the MQTT client is not connected, this function tries to establish a connection
* to the MQTT broker using the settings from the WiFiconfiguration instance.
*
* @note Assumes that MQTT configurationuration parameters (server address, port, client ID,
* username, password) have been previously set in the WiFiconfiguration instance.
*
* @warning This function may delay for extended periods while attempting to connect
* to the MQTT broker.
*/
void connectToMqttBroker() {
  if (!mqttClient.connected()) {
    deviceStatus = NOT_READY;

    mqttClient.setUsernamePassword(preferencesData.mqttUsername.c_str(), preferencesData.mqttPassword.c_str());
    mqttClient.setId(preferencesData.mqttClientId.c_str());
    mqttClient.setTxPayloadSize(1024);  // Optional: increase if JSON is large.

    // Set the message receive callback.
    mqttClient.onMessage(onMqttMessage);

    debug(ERR, "Device not connected to MQTT broker '%s'.", preferencesData.mqttServerAddress.c_str());

    while (!mqttClient.connected()) {
      debug(CMD, "Connecting device to MQTT broker '%s'.", preferencesData.mqttServerAddress.c_str());

      if (mqttClient.connect(preferencesData.mqttServerAddress.c_str(), preferencesData.mqttServerPort)) {
        debug(SCS, "Device connected to MQTT broker '%s'.", preferencesData.mqttServerAddress.c_str());
        mqttClient.subscribe(preferencesData.mqttTopic.c_str());
        deviceStatus = WAITING_GNSS;
      } else {
        delay(4000);  // Retry every 4s.
      }
    }
  }
}

/**
* @brief Retrieves the current UTC time as a formatted string.
*
* This function retrieves the current UTC time using the system time. If successful,
* it formats the time into a UTC date time string (e.g., "2024-06-20T20:56:59Z").
* If the UTC time cannot be obtained, it returns "unknown".
*
* @return A String containing the current UTC time in the specified format, or "unknown" if the time cannot be retrieved.
*/
String getUtcTimeString() {
  if (!gnss.getTimeValid()) {
    return "unknown";
  }

  char isoTime[25];
  snprintf(isoTime, sizeof(isoTime), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           gnss.getYear(),
           gnss.getMonth(),
           gnss.getDay(),
           gnss.getHour(),
           gnss.getMinute(),
           gnss.getSecond());

  return String(isoTime);
}

/**
* Constructs an MQTT message string containing GNSS, IMU, and timestamp data.
*
* Constructs a JSON-formatted MQTT message string containing GNSS-related data
* (satellites, longitude, latitude, altitude, speed, heading, timestamp),
* accelerometer and gyroscope measurements.
*
* @param positioningData      Struct containing GNSS data and timestamp from 'Structs.h'.
* @param accelerometerData    Struct containing accelerometer axis values and magnitude from 'Structs.h'.
* @param gyroscopeData        Struct containing gyroscope axis values from 'Structs.h'.
* @return A String containing the constructed MQTT message in JSON format.
*/
String constructMqttMessage(const PositioningData& positioningData, const AccelerometerData& accelerometerData, const GyroscopeData& gyroscopeData) {
  JsonDocument jsonDocument;

  jsonDocument["timestamp"] = positioningData.timestamp;
  jsonDocument["isStale"] = positioningData.gnssFixOk;
  jsonDocument["satellites"] = positioningData.satellitesInRange;

  JsonObject lon = jsonDocument["longitude"].to<JsonObject>();
  lon["value"] = positioningData.longitude * 1e-7;
  lon["unit"] = "deg";

  JsonObject lat = jsonDocument["latitude"].to<JsonObject>();
  lat["value"] = positioningData.latitude * 1e-7;
  lat["unit"] = "deg";

  JsonObject alt = jsonDocument["altitude"].to<JsonObject>();
  alt["value"] = round((positioningData.altitude / 1000.0) * 100.0) / 100.0;
  alt["unit"] = "m";

  JsonObject spd = jsonDocument["speed"].to<JsonObject>();
  spd["value"] = round(((positioningData.speed / 1000.0) * 3.6) * 100.0) / 100.0;
  spd["unit"] = "km/h";

  JsonObject hdg = jsonDocument["heading"].to<JsonObject>();
  hdg["value"] = round((positioningData.heading * 1e-5) * 100.0) / 100.0;
  hdg["unit"] = "deg";

  JsonObject imu = jsonDocument["imu"].to<JsonObject>();
  JsonObject acc = imu["accelerometer"].to<JsonObject>();
  JsonObject accVals = acc["values"].to<JsonObject>();
  accVals["x"] = round(accelerometerData.x * 100.0) / 100.0;
  accVals["y"] = round(accelerometerData.y * 100.0) / 100.0;
  accVals["z"] = round(accelerometerData.z * 100.0) / 100.0;
  acc["unit"] = "m/s^2";

  JsonObject accel = imu["acceleration"].to<JsonObject>();
  accel["value"] = round(accelerometerData.magnitude * 100.0) / 100.0;
  accel["unit"] = "G";

  JsonObject gyr = imu["gyroscope"].to<JsonObject>();
  JsonObject gyrVals = gyr["values"].to<JsonObject>();
  gyrVals["x"] = round(gyroscopeData.x * 100.0) / 100.0;
  gyrVals["y"] = round(gyroscopeData.y * 100.0) / 100.0;
  gyrVals["z"] = round(gyroscopeData.z * 100.0) / 100.0;
  gyr["unit"] = "deg/s";

  String output;
  serializeJson(jsonDocument, output);
  return output;
}

/**
* @brief Thread function for handling device status indications through an RGB LED.
*
* This thread continuously updates the RGB LED status based on the current device status.
* It uses the DeviceStatusEnum values to determine the appropriate LED indication.
*
* @param pvParameters Pointer to task parameters (not used in this function).
*/
void DeviceStatusThread(void* pvParameters) {
  while (true) {
    // Always show rainbow in MAINTENANCE_MODE.
    if (deviceStatus == MAINTENANCE_MODE) {
      notifications.visual.rainbowMode();
    } else if (preferencesData.visualNotifications) {
      // Clear the NeoPixel LED strip.
      notifications.visual.clearAllPixels();

      switch (deviceStatus) {
        case NOT_READY:
          notifications.visual.notReadyMode();
          break;
        case READY_TO_SEND:
          notifications.visual.readyToSendMode();
          break;
        case WAITING_GNSS:
          notifications.visual.waitingGnssFixMode();
          break;
        default:
          break;
      }
    }

    // Prevent WDT timeout.
    vTaskDelay(8 / portTICK_PERIOD_MS);
  }
}
