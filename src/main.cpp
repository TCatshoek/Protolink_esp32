#include <Arduino.h>
#include "FastLED.h"

#define NUM_LEDS 4 * 4 * 3
CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<NEOPIXEL, 16>(leds, NUM_LEDS);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (uint i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Red; 
    FastLED.show(); 
    delay(100); 
    leds[i] = CRGB::Black; 
    FastLED.show(); 
  }
 
}