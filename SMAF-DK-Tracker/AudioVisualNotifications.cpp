/**
* AudioVisualNotifications.cpp
* Declaration of the AudioVisualNotifications library for RGB LED and audio status indication.
*
* This file contains the declaration for the AudioVisualNotifications library, which facilitates
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

#include "Arduino.h"
#include "AudioVisualNotifications.h"
#include "Adafruit_NeoPixel.h"

/**
* Constructs an AudioVisualNotifications object with specified parameters.
* Initializes the NeoPixel and speaker pin settings for audio-visual notifications.
* 
* @param neoPixelPin The GPIO pin connected to the NeoPixel.
* @param neoPixelCount The number of NeoPixels in the strip.
* @param neoPixelBrightness The brightness level of the NeoPixels (0-255).
* @param speakerPin The GPIO pin connected to the speaker.
*/
AudioVisualNotifications::AudioVisualNotifications(int neoPixelPin, int neoPixelCount, int neoPixelBrightness, int speakerPin)
  : _neoPixelPin(neoPixelPin),
    _neoPixelCount(neoPixelCount),
    _neoPixelBrightness(neoPixelBrightness),
    _speakerPin(speakerPin),
    _neoPixel(neoPixelCount, neoPixelPin, NEO_GRB + NEO_KHZ800),
    audio(*this),
    visual(*this) {
}

/**
* Plays an introductory audio notification sequence through the speaker.
* This function produces a series of tones to signal the start of notifications.
*/
void AudioVisualNotifications::Audio::introMelody() {
  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);

  tone(_parent._speakerPin, NOTE_F6);
  delay(120);
  noTone(_parent._speakerPin);

  tone(_parent._speakerPin, NOTE_G6);
  delay(320);
  noTone(_parent._speakerPin);
}

/**
* Plays a maintenance audio notification sequence through the speaker.
* This function produces a series of tones to signal maintenance notifications.
*/
void AudioVisualNotifications::Audio::maintenanceMelody() {
  // First part.
  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);

  delay(80);

  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);

  delay(80);

  tone(_parent._speakerPin, NOTE_F6);
  delay(120);
  noTone(_parent._speakerPin);

  delay(80);

  tone(_parent._speakerPin, NOTE_G6);
  delay(280);
  noTone(_parent._speakerPin);

  // Second part.
  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);

  tone(_parent._speakerPin, NOTE_F6);
  delay(120);
  noTone(_parent._speakerPin);

  tone(_parent._speakerPin, NOTE_G6);
  delay(320);
  noTone(_parent._speakerPin);
}

/**
* Produces a single short beep using the speaker.
* The tone is played on the speaker connected to the specified pin.
*/
void AudioVisualNotifications::Audio::beep() {
  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);
}

/**
* Produces a double beep using the speaker.
* This function generates two consecutive tones, 
* each lasting 120 milliseconds, with an 80-millisecond pause between them.
* The tones are played on the speaker connected to the specified pin.
*/
void AudioVisualNotifications::Audio::doubleBeep() {
  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);

  delay(80);

  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);
}

/**
* Produces a double beep using the speaker.
* This function generates two consecutive tones, 
* each lasting 120 milliseconds, with an 80-millisecond pause between them.
* The tones are played on the speaker connected to the specified pin.
*/
void AudioVisualNotifications::Audio::tripleBeep() {
  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);

  delay(80);

  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);

  delay(80);

  tone(_parent._speakerPin, NOTE_E6);
  delay(120);
  noTone(_parent._speakerPin);
}

/**
* Initializes the visual notifications by setting up the NeoPixel strip.
* This function must be called to prepare the NeoPixel for use. 
*/
void AudioVisualNotifications::Visual::initializePixels() {
  _parent._neoPixel.begin();
  _parent._neoPixel.clear();
  _parent._neoPixel.show();
  _parent._neoPixel.setBrightness(_parent._neoPixelBrightness);
}

/**
* Clears all visual notifications on the NeoPixel strip.
* This function resets the NeoPixel strip to its default state.
*/
void AudioVisualNotifications::Visual::clearAllPixels() {
  _parent._neoPixel.clear();
  _parent._neoPixel.show();
}

/**
* Displays a visual notification indicating that the system is not ready.
* This function uses the NeoPixel strip to show a red color on one pixel while turning off another.
*/
void AudioVisualNotifications::Visual::notReadyMode() {
  uint32_t interval = 240;

  _parent._neoPixel.setPixelColor(0, _parent._neoPixel.Color(255, 0, 0));
  _parent._neoPixel.setPixelColor(1, _parent._neoPixel.Color(0, 0, 0));
  _parent._neoPixel.show();

  delay(interval);

  _parent._neoPixel.setPixelColor(0, _parent._neoPixel.Color(0, 0, 0));
  _parent._neoPixel.setPixelColor(1, _parent._neoPixel.Color(255, 0, 0));
  _parent._neoPixel.show();

  // clearNeoPixel();
  delay(interval);
}

/**
* Displays a visual notification indicating that the system is ready to send data.
* This function blinks two NeoPixels in green for a specified number of times to signal readiness.
*/
void AudioVisualNotifications::Visual::readyToSendMode() {
  uint32_t delayBeforeNextBurst = 1200;
  int blinkCount = 4;

  // Loop through the specified number of blinks in one burst.
  for (int i = 0; i < blinkCount; ++i) {
    _parent._neoPixel.setPixelColor(0, _parent._neoPixel.Color(0, 255, 0));
    _parent._neoPixel.setPixelColor(1, _parent._neoPixel.Color(0, 255, 0));
    _parent._neoPixel.show();

    delay(40);
    clearAllPixels();
    delay(40);
  }

  // Add a delay before starting the next burst.
  delay(delayBeforeNextBurst);
}

/**
* Displays a visual notification indicating that the system is waiting for a GNSS fix.
* This function uses the NeoPixel strip to show a blue color on one pixel while turning off another.
*/
void AudioVisualNotifications::Visual::waitingGnssFixMode() {
  uint32_t interval = 240;

  _parent._neoPixel.setPixelColor(0, _parent._neoPixel.Color(0, 0, 255));
  _parent._neoPixel.setPixelColor(1, _parent._neoPixel.Color(0, 0, 0));
  _parent._neoPixel.show();

  delay(interval);

  _parent._neoPixel.setPixelColor(0, _parent._neoPixel.Color(0, 0, 0));
  _parent._neoPixel.setPixelColor(1, _parent._neoPixel.Color(0, 0, 255));
  _parent._neoPixel.show();

  // clearNeoPixel();
  delay(interval);
}

/**
* Displays a visual notification indicating that the system is loading.
* This function uses the NeoPixel strip to show a magenta color on one pixel while turning off another.
*/
void AudioVisualNotifications::Visual::loadingMode() {
  uint32_t interval = 240;

  _parent._neoPixel.setPixelColor(0, _parent._neoPixel.Color(255, 0, 255));
  _parent._neoPixel.setPixelColor(1, _parent._neoPixel.Color(0, 0, 0));
  _parent._neoPixel.show();

  delay(interval);

  _parent._neoPixel.setPixelColor(0, _parent._neoPixel.Color(0, 0, 0));
  _parent._neoPixel.setPixelColor(1, _parent._neoPixel.Color(255, 0, 255));
  _parent._neoPixel.show();

  // clearNeoPixel();
  delay(interval);
}

/**
* Displays a visual notification indicating that maintenance is required.
* This function uses the NeoPixel strip to show a magenta color on two pixels briefly.
*/
void AudioVisualNotifications::Visual::maintenanceMode() {
  uint32_t interval = 240;

  _parent._neoPixel.setPixelColor(0, _parent._neoPixel.Color(255, 0, 255));
  _parent._neoPixel.setPixelColor(1, _parent._neoPixel.Color(255, 0, 255));
  _parent._neoPixel.show();

  delay(interval);
  clearAllPixels();
  delay(interval);
}

/**
* Initializes a specific NeoPixel with the given color values.
* This function sets the color of a specified pixel in the NeoPixel strip.
* 
* @param pixel The index of the NeoPixel to be initialized.
* @param red The red color value (0-255).
* @param green The green color value (0-255).
* @param blue The blue color value (0-255).
*/
void AudioVisualNotifications::Visual::singlePixel(int pixel, int red, int green, int blue) {
  _parent._neoPixel.setPixelColor(pixel, _parent._neoPixel.Color(red, green, blue));
  _parent._neoPixel.show();
}

/**
* Displays a rainbow animation on the NeoPixel strip.
* This function cycles through the color wheel, creating a smooth rainbow animation
* across all NeoPixels. The animation consists of 1280 passes through the loop,
* with a short delay between updates to achieve a smooth transition.
*
* The animation uses the built-in `rainbow` function of the NeoPixel library, 
* which generates a sequence of colors based on the hue.
*
* Note: The brightness and gamma correction are set to default values.
*/
void AudioVisualNotifications::Visual::rainbowMode() {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    _parent._neoPixel.rainbow(firstPixelHue);
    _parent._neoPixel.show();
    delay(12);
  }
}