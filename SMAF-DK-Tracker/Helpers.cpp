/**
* Helpers.cpp
* Implementation of helper functions for Arduino project.
*
* This file contains the implementation of helper functions used in the Arduino project.
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

#include "Arduino.h"
#include "Helpers.h"
#include "esp_task_wdt.h"

// Define the variable for message type.
MessageTypeEnum messageType = LOG;

/**
* Sends a formatted debug message to the Serial monitor with the specified message type.
* This function formats the message based on the provided message type and arguments.
* 
* @param messageType The type of message to log (LOG, ERR, SCS, CMD).
* @param format The format string for the message.
* @param ... Additional arguments for formatting the message.
*/
void debug(MessageTypeEnum messageType, const char* format, ...) {
  // Set up an empty string to store the message type as a string
  String messageTypeStr;

  // Switch statement to determine the message type string based on the input byte
  switch (messageType) {
    case LOG:
      messageTypeStr = "LOG";  // For LOG, set the message type string to "LOG".
      break;
    case ERR:
      messageTypeStr = "ERROR";  // For ERR, set the message type string to "ERROR".
      break;
    case SCS:
      messageTypeStr = "OK";  // For SCS, set the message type string to "OK".
      break;
    case CMD:
      messageTypeStr = "CMD";  // For CMD, set the message type string to "CMD".
      break;
  }

  // Format the variable arguments directly.
  va_list args;
  va_start(args, format);
  char buffer[256];  // Adjust the buffer size according to your needs.
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  // Print the formatted debug message to the Serial monitor.
  Serial.printf("CORE-%02d | %5s | %s\n\r", xPortGetCoreID(), messageTypeStr.c_str(), buffer);
}

/**
* Initializes the watchdog timer with the specified timeout and panic settings.
* This function configures the watchdog based on the ESP32 core version.
* 
* @param timeout The timeout duration in seconds before the watchdog triggers.
* @param panic Indicates whether to trigger a panic if the watchdog timer is not reset.
*/
void initWatchdog(uint32_t timeout, bool panic) {
  // Check if the ESP32 core version is 3.0.0 or lower.
  // ESP32 Core changed initialization methods for watchdog timers in version 3.0.0 or higher.
#if (VERSION_CHECK(ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH) < VERSION_CHECK(3, 0, 0))
  // ESP32 Arduino Core < 3.0
  // Initialize the watchdog.
  esp_task_wdt_init(timeout, panic);
  esp_task_wdt_add(NULL);
#else
  // ESP32 Arduino Core >= 3.0
  // Define the configuration structure.
  esp_task_wdt_config_t config = {
    .timeout_ms = timeout * 1000,  // Timeout
    .trigger_panic = panic         // Trigger panic if watchdog timer is not reset
  };

  // Initialize the watchdog timer with the configuration structure.
  esp_task_wdt_reconfigure(&config);
  esp_task_wdt_add(NULL);
#endif

  // Log the status in the terminal.
  debug(LOG, "Watchdog timer intialized.");
}

/**
* Resets the watchdog timer to prevent it from triggering.
* This function is used to keep the watchdog timer active during long-running tasks.
*/
void resetWatchdog() {
  // Reset WDT.
  esp_task_wdt_reset();

  // Log the status in the terminal.
  debug(LOG, "Watchdog reset.");
}

/**
* Suspends the watchdog timer to prevent it from triggering.
* This function disables the watchdog, allowing for longer execution of tasks without resets.
*/
void suspendWatchdog() {
  // Disable WDT.
  esp_task_wdt_delete(NULL);

  // Log the status in the terminal.
  debug(LOG, "Watchdog suspended.");
}

/**
* Checks if a given C-string is empty or null.
* This function returns true if the string is null or has no characters.
* 
* @param str The C-string to check.
* @return true if the string is null or empty; false otherwise.
*/
bool isEmpty(const char* str) {
  return str == nullptr || str[0] == '\0';
}

/**
* Wraps a given string in double quotes.
* This function adds quotation marks around the provided string.
* 
* @param data The string to be quoted.
* @return A new string with double quotes surrounding the input string.
*/
String quotation(String data) {
  return "\"" + data + "\"";
}