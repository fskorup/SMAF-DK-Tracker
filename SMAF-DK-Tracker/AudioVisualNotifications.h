/**
* AudioVisualNotifications.h
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

#ifndef AUDIO_VISUAL_NOTIFICATIONS_H
#define AUDIO_VISUAL_NOTIFICATIONS_H

#include "Arduino.h"
#include "Adafruit_NeoPixel.h"

// Define piano notes.
#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978

class AudioVisualNotifications {
public:
  /**
  * Constructs an AudioVisualNotifications object with specified parameters.
  * Initializes the NeoPixel and speaker pin settings for audio-visual notifications.
  * 
  * @param neoPixelPin The GPIO pin connected to the NeoPixel.
  * @param neoPixelCount The number of NeoPixels in the strip.
  * @param neoPixelBrightness The brightness level of the NeoPixels (0-255).
  * @param speakerPin The GPIO pin connected to the speaker.
  */
  AudioVisualNotifications(int neoPixelPin, int neoPixelCount, int neoPixelBrightness, int speakerPin);

  // Nested class for audio notifications.
  class Audio {
  public:
    explicit Audio(AudioVisualNotifications& parent)
      : _parent(parent) {}

    /**
    * Plays an introductory audio notification sequence through the speaker.
    * This function produces a series of tones to signal the start of notifications.
    */
    void introMelody();

    /**
    * Plays a maintenance audio notification sequence through the speaker.
    * This function produces a series of tones to signal maintenance notifications.
    */
    void maintenanceMelody();

    /**
    * Produces a single short beep using the speaker.
    * The tone is played on the speaker connected to the specified pin.
    */
    void beep();

    /**
    * Produces a double beep using the speaker.
    * This function generates two consecutive tones, 
    * each lasting 120 milliseconds, with an 80-millisecond pause between them.
    * The tones are played on the speaker connected to the specified pin.
    */
    void doubleBeep();

    /**
    * Produces a double beep using the speaker.
    * This function generates two consecutive tones, 
    * each lasting 120 milliseconds, with an 80-millisecond pause between them.
    * The tones are played on the speaker connected to the specified pin.
    */
    void tripleBeep();
  private:
    AudioVisualNotifications& _parent;  // Reference to parent
  };

  // Nested class for visual notifications.
  class Visual {
  public:
    explicit Visual(AudioVisualNotifications& parent)
      : _parent(parent) {}

    /**
    * Initializes the visual notifications by setting up the NeoPixel strip.
    * This function must be called to prepare the NeoPixel for use. 
    */
    void initializePixels();

    /**
    * Clears all visual notifications on the NeoPixel strip.
    * This function resets the NeoPixel strip to its default state.
    */
    void clearAllPixels();

    /**
    * Displays a visual notification indicating that the system is not ready.
    * This function uses the NeoPixel strip to show a red color on one pixel while turning off another.
    */
    void notReadyMode();

    /**
    * Displays a visual notification indicating that the system is ready to send data.
    * This function blinks two NeoPixels in green for a specified number of times to signal readiness.
    */
    void readyToSendMode();

    /**
    * Displays a visual notification indicating that the system is waiting for a GNSS fix.
    * This function uses the NeoPixel strip to show a blue color on one pixel while turning off another.
    */
    void waitingGnssFixMode();

    /**
    * Displays a visual notification indicating that the system is loading.
    * This function uses the NeoPixel strip to show a magenta color on one pixel while turning off another.
    */
    void loadingMode();

    /**
    * Displays a visual notification indicating that maintenance is required.
    * This function uses the NeoPixel strip to show a magenta color on two pixels briefly.
    */
    void maintenanceMode();

    /**
    * Initializes a specific NeoPixel with the given color values.
    * This function sets the color of a specified pixel in the NeoPixel strip.
    * 
    * @param pixel The index of the NeoPixel to be initialized.
    * @param red The red color value (0-255).
    * @param green The green color value (0-255).
    * @param blue The blue color value (0-255).
    */
    void singlePixel(int pixel, int red, int green, int blue);

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
    void rainbowMode();
  private:
    AudioVisualNotifications& _parent;  // Reference to parent
  };

  Audio audio;
  Visual visual;
private:
  int _neoPixelPin;
  int _neoPixelCount;
  int _neoPixelBrightness;
  int _speakerPin;
  Adafruit_NeoPixel _neoPixel;  // Declare neoPixel as a member variable.
};

#endif
