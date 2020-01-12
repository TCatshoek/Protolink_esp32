#include <Arduino.h>
#include "protolink.h"
#include <WiFi.h>
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include "FastLED.h"
#include "credentials.h"
#include <BluetoothSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// #define BRIGHTNESS  5
// #define NUM_LEDS 4 * 4 * 3
// CRGB leds[NUM_LEDS];
// uint8_t bs[1024];

// extern const char* ssid;
// extern const char* password;

// WiFiServer server(8080);


BluetoothSerial SerialBT;
uint8_t buf[112];
ProtoLink<BluetoothSerial> protolink(buf, 112);

void setup() {
  esp_log_level_set("*", ESP_LOG_VERBOSE);

 // Setup screen
  Wire.begin(5, 4);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false); // Address 0x3C for 128x32
  display.display();
  delay(1000);

  Serial.begin(9600);
  
  SerialBT.begin("Beeper");
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Clear the buffer.
  display.clearDisplay();
  display.display();

}

inline bool getbit(const uint8_t* buf, int pos) {
    int byteidx = pos / (sizeof(uint8_t) * 8);
    int bitidx = pos % (sizeof(uint8_t) * 8);

    uint8_t tmp = buf[byteidx];

    int bit = (tmp >> bitidx) & 1;

    return (bool)bit;
}

void copyToScreen() {
  uint x;
  uint y;

  uint bitcounter = 0;

  // Right eye
  for (y = 0; y < 8; y++) {
    for(x = 0; x < 32; x += 2) { 
      display.drawPixel(x, y, getbit(buf, bitcounter));
      bitcounter++;
    }
  }

  // Left eye
  for (y = 0; y < 8; y++) {
    for(x = 128 - 32; x < 128; x += 2) { 
      display.drawPixel(x, y, getbit(buf, bitcounter));
      bitcounter++;
    }
  }

  // Right nostril
  for (y = 15; y < 23; y++) {
    for(x = 47; x < 63; x += 2) { 
      display.drawPixel(x, y, getbit(buf, bitcounter));
      bitcounter++;
    }
  }

  // Left nostril
  for (y = 15; y < 23; y++) {
    for(x = 49 + 16; x < 65 + 16; x += 2) { 
      display.drawPixel(x, y, getbit(buf, bitcounter));
      bitcounter++;
    }
  }

  // Right mouth back
  for (y = 32 - 8; y < 32; y++) {
    for(x = 0; x < 32; x += 2) { 
      display.drawPixel(x, y, getbit(buf, bitcounter));
      bitcounter++;
    }
  }
  // Right mouth front
  for (y = 32 - 8; y < 32; y++) {
    for(x = 32; x < 64; x += 2) { 
      display.drawPixel(x, y, getbit(buf, bitcounter));
      bitcounter++;
    }
  }
  
  // Left mouth front
  for (y = 32 - 8; y < 32; y++) {
    for(x = 64; x < 64 + 32; x += 2) { 
      display.drawPixel(x, y, getbit(buf, bitcounter));
      bitcounter++;
    }
  }
  // Left mouth back
  for (y = 32 - 8; y < 32; y++) {
    for(x = 64 + 32; x < 128; x += 2) { 
      display.drawPixel(x, y, getbit(buf, bitcounter));
      bitcounter++;
    }
  }
 
}

void loop() {

  display.display();
  delay(100);

  if (SerialBT.available()) {
    Serial.println("Client connected");
    ProtoLink<BluetoothSerial>::State state = protolink.init(&SerialBT);

    while(SerialBT.connected() && state != ProtoLink<BluetoothSerial>::State::ERR) {
      state = protolink.run();
   
      if (state == ProtoLink<BluetoothSerial>::State::RECV_LEN) {
        SerialBT.write((uint8_t)state);
        copyToScreen();
        display.display();
        display.clearDisplay();
      }
      yield();
    }

    Serial.println("Client disconnected");
    if (protolink.error_msg != NULL && state == ProtoLink<BluetoothSerial>::State::ERR) {
      Serial.print("Error: ");
      Serial.println(protolink.error_msg);
    }
  }
}