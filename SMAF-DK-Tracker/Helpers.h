/**
* Helpers.h
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

#ifndef HELPERS_H
#define HELPERS_H

#include "Arduino.h"
#include "esp_task_wdt.h"
#include "esp_system.h"

// Define a macro for comparing version numbers.
#define VERSION_CHECK(major, minor, patch) ((major)*10000 + (minor)*100 + (patch))

/**
* Enum representing the different types of messages for logging.
* This enumeration categorizes messages to facilitate logging and debugging.
*/
enum MessageTypeEnum : byte {
  LOG,  // Info type. INFO message type displayed.
  ERR,  // Error type. ERROR message type displayed.
  SCS,  // Success type. OK message type displayed.
  CMD   // Command type. CMD message type displayed.
};

extern MessageTypeEnum messageType;  // Declare the variable.

/**
* Sends a formatted debug message to the Serial monitor with the specified message type.
* This function formats the message based on the provided message type and arguments.
* 
* @param messageType The type of message to log (LOG, ERR, SCS, CMD).
* @param format The format string for the message.
* @param ... Additional arguments for formatting the message.
*/
void debug(MessageTypeEnum messageType, const char *format, ...);

/**
* Initializes the watchdog timer with the specified timeout and panic settings.
* This function configures the watchdog based on the ESP32 core version.
* 
* @param timeout The timeout duration in seconds before the watchdog triggers.
* @param panic Indicates whether to trigger a panic if the watchdog timer is not reset.
*/
void initWatchdog(uint32_t timeout, bool panic);

/**
* Resets the watchdog timer to prevent it from triggering.
* This function is used to keep the watchdog timer active during long-running tasks.
*/
void resetWatchdog();

/**
* Suspends the watchdog timer to prevent it from triggering.
* This function disables the watchdog, allowing for longer execution of tasks without resets.
*/
void suspendWatchdog();

/**
* Checks if a given C-string is empty or null.
* This function returns true if the string is null or has no characters.
* 
* @param str The C-string to check.
* @return true if the string is null or empty; false otherwise.
*/
bool isEmpty(const char* str);

/**
* Wraps a given string in double quotes.
* This function adds quotation marks around the provided string.
* 
* @param data The string to be quoted.
* @return A new string with double quotes surrounding the input string.
*/
String quotation(String data);

#endif