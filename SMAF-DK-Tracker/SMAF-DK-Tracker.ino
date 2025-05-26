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
#include "PubSubClient.h"
#include "AudioVisualNotifications.h"
#include "WiFiConfig.h"
#include "Helpers.h"
#include "time.h"
#include "SparkFun_u-blox_GNSS_v3.h"
#include "SparkFun_BMI270_Arduino_Library.h"

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

// Preferences variables.
String networkName = String();
String networkPass = String();
String mqttServerAddress = String();
String mqttUsername = String();
String mqttPass = String();
String mqttClientId = String();
String mqttTopic = String();
uint16_t mqttServerPort = 0;
bool visualNotifications = false;
bool audioNotifications = false;

/**
* @brief WiFiClient and PubSubClient instances for establishing MQTT communication.
* 
* The WiFiClient instance, named wifiClient, is used to manage the Wi-Fi connection.
* The PubSubClient instance, named mqtt, relies on the WiFiClient for MQTT communication.
*/
WiFiClient wifiClient;          // Manages Wi-Fi connection.
PubSubClient mqtt(wifiClient);  // Uses WiFiClient for MQTT communication.

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

// Accelerometer and gyroscope readings.
float accelerometerX, accelerometerY, accelerometerZ, accelerometerMagnitude;
float gyroscopeX, gyroscopeY, gyroscopeZ;

// NTP Server configuration.
const char* ntpServer = "europe.pool.ntp.org";  // Global - pool.ntp.org
const long gmtOffset = 0;
const int dstOffset = 0;

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
  // Example usage:
  // Wire.setPins(SDA_PIN_NUMBER, SCL_PIN_NUMBER);
  Wire.setPins(I2CPins.SDA, I2CPins.SCL);
  Wire.begin();

  // Set the pin mode for the configuration button to INPUT.
  pinMode(InputPins.UserButton, INPUT);

  // Load and check configuration.
  WiFiConfig config = loadWiFiConfig();

  networkName = config.ssidName;
  networkPass = config.ssidPassword;
  mqttServerAddress = config.mqttServer;
  mqttUsername = config.mqttUsername;
  mqttPass = config.mqttPassword;
  mqttClientId = config.mqttClientId;
  mqttServerPort = config.mqttServerPort;
  mqttTopic = config.mqttTopic;
  visualNotifications = config.rgb ? true : false;
  audioNotifications = config.buzzer ? true : false;

  // Check and store flag if configuration is valid.
  static bool isConfigurationValid = config.ssidName.length() > 0 && config.mqttServer.length() > 0 && config.mqttClientId.length() > 0 && config.mqttTopic.length() > 0 && config.mqttServerPort > 0;

  // Initialize visualization library neo pixels.
  // This does not light up neo pixels.
  notifications.visual.initializePixels();

  // Play intro melody on speaker if enabled in preferences.
  if (audioNotifications) {
    notifications.audio.introMelody();
  }

  // Delay to stabilise things.
  delay(1600);

  // Print a formatted welcome message with build information.
  String buildVersion = "v0.001";
  String buildDate = "Q2, 2025.";
  Serial.printf("\n\rSMAF-DK-Tracker, Crafted with love in Europe.\n\rBuild version: %s\n\rBuild date: %s\n\r\n\r", buildVersion, buildDate);

  // Print loaded configuration data.
  debug(CMD, "Loading WiFi/MQTT Configuration");
  debug(SCS, "Loaded WiFi/MQTT Configuration");
  debug(LOG, "SSID Name: %s", config.ssidName.c_str());
  debug(LOG, "SSID Password: %s", config.ssidPassword.c_str());
  debug(LOG, "MQTT Server: %s", config.mqttServer.c_str());
  debug(LOG, "MQTT Port: %s", String(config.mqttServerPort));
  debug(LOG, "MQTT Username: %s", config.mqttUsername.c_str());
  debug(LOG, "MQTT Password: %s", config.mqttPassword.c_str());
  debug(LOG, "MQTT Client ID: %s", config.mqttClientId.c_str());
  debug(LOG, "MQTT Topic: %s", config.mqttTopic.c_str());
  debug(LOG, "Visual Notifications: %s", String(config.rgb ? "true" : "false"));
  debug(LOG, "Audio Notifications: %s", String(config.buzzer ? "true" : "false"));

  // Print status of configuration data.
  if (isConfigurationValid) {
    debug(SCS, "Configuration is valid. All required configuration data is present.");
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

  // MQTT Client message buffer size.
  // Default is set to 256.
  mqtt.setBufferSize(1024);

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
  accelerometerX = imu.data.accelX;
  accelerometerY = imu.data.accelY;
  accelerometerZ = imu.data.accelZ;
  accelerometerMagnitude = sqrt(accelerometerX * accelerometerX + accelerometerY * accelerometerY + accelerometerZ * accelerometerZ);

  // Gyroscope data.
  gyroscopeX = imu.data.gyroX;
  gyroscopeY = imu.data.gyroY;
  gyroscopeZ = imu.data.gyroZ;

  // Log IMU data.
  debug(LOG, "IMU acceleorometer: X %sm/s^2, Y %sm/s^2, Z %sm/s^2, Magnitude %sm/s^2.", String(accelerometerX).c_str(), String(accelerometerY).c_str(), String(accelerometerZ).c_str(), String(accelerometerMagnitude).c_str());
  debug(LOG, "IMU gyroscope: X %sdeg/s, Y %sdeg/s, Z %sdeg/s", String(gyroscopeX).c_str(), String(gyroscopeY).c_str(), String(gyroscopeZ).c_str());

  // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (gnss.getPVT() == true) {
    bool gnssFixOk = gnss.getGnssFixOk();
    uint8_t satellitesInRange = gnss.getSIV();
    int32_t latitude = gnss.getLatitude();
    int32_t longitude = gnss.getLongitude();
    int32_t speed = gnss.getGroundSpeed();
    int32_t heading = gnss.getHeading();
    int32_t altitude = gnss.getAltitudeMSL();
    String timestamp = getUtcTimeString();

    // Store MQTT data here.
    String mqttData = constructMqttMessage(
      satellitesInRange,
      longitude,
      latitude,
      altitude,
      speed,
      heading,
      timestamp,
      accelerometerX,
      accelerometerY,
      accelerometerY,
      accelerometerMagnitude,
      gyroscopeX,
      gyroscopeY,
      gyroscopeZ);

    // If the device is ready to send, publish a message to the MQTT broker.
    if (gnssFixOk && latitude != 0 && longitude != 0) {
      deviceStatus = READY_TO_SEND;
      debug(SCS, "Device is ready to post data, %d satellites locked.", satellitesInRange);
      debug(CMD, "Posting data package to MQTT broker '%s' on topic '%s'.", mqttServerAddress.c_str(), mqttTopic.c_str());
      mqtt.publish(mqttTopic.c_str(), mqttData.c_str(), true);
    } else {
      deviceStatus = WAITING_GNSS;
      debug(ERR, "Device is not ready to post data. Searching for satellites, %d locked.", satellitesInRange);
    }
  }

  // Check for incoming data on defined MQTT topic.
  // This is hard core connection check.
  // If no data on topic is received, we are not connected to internet or server and watchdog will reset the device.
  mqtt.loop();
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
    debug(ERR, "Device not connected to '%s'.", networkName.c_str());

    // Keep attempting to connect until successful.
    while (WiFi.status() != WL_CONNECTED) {
      debug(CMD, "Connecting device to '%s'", networkName.c_str());

      // Attempt to connect to the Wi-Fi network using configurationured credentials.
      WiFi.begin(networkName.c_str(), networkPass.c_str());
      delay(6400);
    }

    // Log successful connection and set device status.
    debug(SCS, "Device connected to '%s'.", networkName.c_str());

    // Initialize NTP server time configuration.
    configTime(gmtOffset, dstOffset, ntpServer);
    debug(SCS, "NTP Server configured");
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
  if (!mqtt.connected()) {
    // Set initial device status.
    deviceStatus = NOT_READY;

    // Set MQTT server and connection parameters.
    mqtt.setServer(mqttServerAddress.c_str(), mqttServerPort);
    mqtt.setCallback(serverResponse);

    // Log an error if not connected.
    debug(ERR, "Device not connected to MQTT broker '%s'.", mqttServerAddress.c_str());

    // Keep attempting to connect until successful.
    while (!mqtt.connected()) {
      debug(CMD, "Connecting device to MQTT broker '%s'.", mqttServerAddress.c_str());

      if (mqtt.connect(mqttClientId.c_str(), mqttUsername.c_str(), mqttPass.c_str())) {
        // Log successful connection and set device status.
        debug(SCS, "Device connected to MQTT broker '%s'.", mqttServerAddress.c_str());

        // Subscribe to MQTT topic.
        mqtt.subscribe(mqttTopic.c_str());

        deviceStatus = WAITING_GNSS;
      } else {
        // Retry after a delay if connection failed.
        delay(4000);
      }
    }
  }
}

/**
* @brief Retrieves the current UTC time as a formatted string.
*
* This function retrieves the current UTC time using the system time. If successful,
* it formats the time into a UTC date time string (e.g., "2024-06-20T20:56:59Z").
* If the UTC time cannot be obtained, it returns "Unknown".
*
* @return A String containing the current UTC time in the specified format, or "Unknown" if the time cannot be retrieved.
*/
String getUtcTimeString() {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo)) {
    return "Unknown";
  }

  // Create a buffer to hold the formatted time string
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buffer);
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
void serverResponse(char* topic, byte* payload, unsigned int length) {
  debug(SCS, "Server '%s' responded.", mqttServerAddress.c_str());

  // Reset WDT.
  if (deviceStatus != MAINTENANCE_MODE) {
    resetWatchdog();
  }
}

/**
* @brief Constructs an MQTT message string containing GPS and time-related data.
*
* Constructs a JSON-formatted MQTT message string containing various GPS-related data
* (satellites in range, longitude, latitude, speed, heading, altitude) and time-related
* information (timestamp, GMT offset, DST offset).
*
* @param timestamp Human-readable timestamp in UTC format.
* @param satellitesInRange Number of satellites currently in range.
* @param longitude Longitude value in microdegrees (degrees * 1E-7).
* @param latitude Latitude value in microdegrees (degrees * 1E-7).
* @param altitude Altitude value in meters.
* @param speed Speed value in meters per second.
* @param heading Heading direction in microdegrees (degrees * 1E-5).
* @return A String containing the constructed MQTT message in JSON format.
*/
String constructMqttMessage(uint8_t satellitesInRange, int32_t longitude, int32_t latitude, int32_t altitude, int32_t speed, int32_t heading, String timestamp, float accelerometerX, float accelerometerY, float accelerometerZ, float accelerometerMagnitude, float gyroscopeX, float gyroscopeY, float gyroscopeZ) {
  String message;

  message += "{";
  message += quotation("timestamp") + ":" + quotation(timestamp) + ",";
  message += quotation("satellites") + ":" + String(satellitesInRange) + ",";
  message += quotation("longitude") + ":";
  message += "{";
  message += quotation("value") + ":" + String((longitude * 1E-7), 6) + ",";
  message += quotation("unit") + ":" + quotation("deg");
  message += "},";
  message += quotation("latitude") + ":";
  message += "{";
  message += quotation("value") + ":" + String((latitude * 1E-7), 6) + ",";
  message += quotation("unit") + ":" + quotation("deg");
  message += "},";
  message += quotation("altitude") + ":";
  message += "{";
  message += quotation("value") + ":" + String(int((altitude / 1000.0))) + ",";
  message += quotation("unit") + ":" + quotation("m");
  message += "},";
  message += quotation("speed") + ":";
  message += "{";
  message += quotation("value") + ":" + String(int((speed / 1000.0) * 3.6)) + ",";
  message += quotation("unit") + ":" + quotation("km/h");
  message += "},";
  message += quotation("heading") + ":";
  message += "{";
  message += quotation("value") + ":" + String((heading * 1E-5), 0) + ",";
  message += quotation("unit") + ":" + quotation("deg");
  message += "},";
  message += quotation("imu") + ":{";
  message += quotation("accelerometer") + ":{";
  message += quotation("values") + ":{";
  message += quotation("x") + ":" + String(accelerometerX, 2) + ",";
  message += quotation("y") + ":" + String(accelerometerY, 2) + ",";
  message += quotation("z") + ":" + String(accelerometerZ, 2);
  message += "},";
  message += quotation("unit") + ":" + quotation("m/s^2");
  message += "},";
  message += quotation("acceleration") + ":";
  message += "{";
  message += quotation("value") + ":" + String(accelerometerMagnitude, 2) + ",";
  message += quotation("unit") + ":" + quotation("G");
  message += "},";
  message += quotation("gyroscope") + ":{";
  message += quotation("values") + ":{";
  message += quotation("x") + ":" + String(gyroscopeX, 2) + ",";
  message += quotation("y") + ":" + String(gyroscopeY, 2) + ",";
  message += quotation("z") + ":" + String(gyroscopeZ, 2);
  message += "},";
  message += quotation("unit") + ":" + quotation("deg/s");
  message += "}";
  message += "}";
  message += "}";

  return message;
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
    } else if (visualNotifications) {
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
