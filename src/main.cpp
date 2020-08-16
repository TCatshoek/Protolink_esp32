#include <Arduino.h>
#include "protolink.h"
//#include <WiFi.h>
#include "credentials.h"
#include <BluetoothSerial.h>
#include "ringbuffer.h"

// Robo ears
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include <FastLED.h>
#define NUM_LEDS 25
#define DATA_PIN1 5
#define DATA_PIN2 26
CRGB leds[NUM_LEDS];

//Face
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>
#define NUMBER_OF_DEVICES 7 //number of led matrix connect in series
#define CS_PIN1 21
#define CLK_PIN1 17
#define MISO_PIN 2 //we do not use this pin just fill to match constructor
#define MOSI_PIN1 16
#define CS_PIN2 19
#define CLK_PIN2 23
#define MOSI_PIN2 18

SPIClass hspi(HSPI);
SPIClass vspi(VSPI);

Max72xxPanel face_right = Max72xxPanel(
  CLK_PIN1,
  MISO_PIN,
  MOSI_PIN1,
  CS_PIN1,
  &hspi,
  NUMBER_OF_DEVICES, 1
);

Max72xxPanel face_left = Max72xxPanel(
  CLK_PIN2,
  MISO_PIN,
  MOSI_PIN2,
  CS_PIN2,
  &vspi,
  NUMBER_OF_DEVICES, 1
);

#define FACEBUFSIZE 8*14
byte facebuf[FACEBUFSIZE];

RingBuffer rb(FACEBUFSIZE, 5);
// extern const char* ssid;
// extern const char* password;

// WiFiServer server(8080);
ProtoLink<BluetoothSerial> protolink((uint8_t*)&facebuf, FACEBUFSIZE);

BluetoothSerial SerialBT;

TaskHandle_t displayTask;

inline bool getbit(const uint8_t* buf, int pos) {
    int byteidx = pos / (sizeof(uint8_t) * 8);
    int bitidx = pos % (sizeof(uint8_t) * 8);

    uint8_t tmp = buf[byteidx];

    int bit = (tmp >> bitidx) & 1;

    return (bool)bit;
}

void refreshDisplay() {
  FastLED.show();

  //Serial.println("Attempting to refresh display");

  byte* facebuf = rb.read();
  if (facebuf == NULL) {
   Serial.println("No frame ready yet");
    return;
  }

  // Serial.println("refreshing display");

  uint bitcounter = 0;

  // Right eye
  for(int y = 0; y < 8; y++){
    for(int x = 4*8; x < 6*8; x++) {
      face_right.drawPixel(x, y, getbit(facebuf, bitcounter));
      bitcounter++;
    }
  }
  // Left eye
  for(int y = 0; y < 8; y++){
    for(int x = 1*8; x < 3*8; x++) {
      face_left.drawPixel(x, y, getbit(facebuf, bitcounter));
      bitcounter++;
    }
  }
  // Right nostril
  for(int y = 0; y < 8; y++){
    for(int x = 6*8; x < 7*8; x++) {
      face_right.drawPixel(x, y, getbit(facebuf, bitcounter));
      bitcounter++;
    }
  }
  // Left nostril
  for(int y = 0; y < 8; y++){
    for(int x = 0*8; x < 1*8; x++) {
      face_left.drawPixel(x, y, getbit(facebuf, bitcounter));
      bitcounter++;
    }
  }
  // Right mouth back
  for(int y = 0; y < 8; y++){
    for(int x = 0*8; x < 2*8; x++) {
      face_right.drawPixel(x, y, getbit(facebuf, bitcounter));
      bitcounter++;
    }
  }
  // Right mouth front
  for(int y = 0; y < 8; y++){
    for(int x = 2*8; x < 4*8; x++) {
      face_right.drawPixel(x, y, getbit(facebuf, bitcounter));
      bitcounter++;
    }
  }
  // Left mouth front
  for(int y = 0; y < 8; y++){
    for(int x = 3*8; x < 5*8; x++) {
      face_left.drawPixel(x, y, getbit(facebuf, bitcounter));
      bitcounter++;
    }
  }
   // Left mouth back
  for(int y = 0; y < 8; y++){
    for(int x = 5*8; x < 7*8; x++) {
      face_left.drawPixel(x, y, getbit(facebuf, bitcounter));
      bitcounter++;
    }
  }

  face_right.write();
  face_left.write();
}

void refreshDisplayTaskCode(void * pvParameters) {
  while(true) {
    //Serial.println("FRUIT");
    refreshDisplay();
    yield();
    delay(16);
    //Serial.println("LOOPZ");
  }
}

void copyFBtoRB() {
  rb.write(facebuf);
}

void startupcheck() {
  for(int i = 0; i < 3; i++) {
    face_left.fillScreen(1);
    face_right.fillScreen(1);
    face_left.write();
    face_right.write();
    delay(100);
    face_left.fillScreen(0);
    face_right.fillScreen(0);
    face_left.write();
    face_right.write();
    delay(100);
  }
}

void setup() {
  esp_log_level_set("*", ESP_LOG_VERBOSE);

  // Ear config
  FastLED.addLeds<NEOPIXEL, DATA_PIN1>(leds, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(leds, NUM_LEDS);

  // Face config
  face_left.setIntensity(4);
  face_right.setIntensity(4);
  face_left.write();
  face_right.write();
  // Fix face left rotation and position
  face_left.setRotation(0, 1);
  for(int i = 1; i < NUMBER_OF_DEVICES; i++) {
    face_left.setRotation(i, 3);
  }
  face_left.setPosition(1, 2, 0);
  face_left.setPosition(2, 1, 0);
  face_left.setPosition(3, 6, 0);
  face_left.setPosition(4, 5, 0);
  face_left.setPosition(5, 4, 0);
  face_left.setPosition(6, 3, 0);
  // Fix face right rotation and position
  for(int i = 0; i < NUMBER_OF_DEVICES; i++) {
    face_right.setRotation(i, 1);
  }

  // Set ear LEDs to red
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Red;
  }

  Serial.begin(115200);

  SerialBT.enableSSP();
  SerialBT.begin("Beeper");
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Protolink handlers
  protolink.onFBReceived([](){
   //refreshDisplay();
     copyFBtoRB();
  });

  // Setup task to refresh the display
  xTaskCreate(
    refreshDisplayTaskCode,
    "displaytask",
    10000,
    NULL,
    1,
    &displayTask
  );

  startupcheck();
}

// uint8_t icon01[] =     {B00100000,B01111000,B11011110,B11000111,B11111111,B00000000,B00000000,B00000000};
// uint8_t icon02[] =     {B00000000,B00000000,B00000000,B10000000,B11100000,B01111000,B00011110,B00000111};
// uint8_t icon03[] =     {B00000000,B00000000,B00000000,B00000000,B00000111,B00011110,B01111000,B11100000};
// uint8_t icon04[] =     {B00000000,B00000000,B00000000,B11100000,B11111000,B00011110,B00000111,B00000001};
// #define LED_ON 1

void loop() {
  //face_right.clear();
  // face_right.drawBitmap(0, 0, icon01, 8, 8, LED_ON);
  // face_right.drawBitmap(8, 0, icon02, 8, 8, LED_ON);
  // face_right.drawBitmap(16, 0, icon03, 8, 8, LED_ON);
  // face_right.drawBitmap(24, 0, icon04, 8, 8, LED_ON);
  // face_right.write();
  // //refreshDisplay();

  delay(100);

  if (SerialBT.available()) {
    Serial.println("Client connected");
    ProtoLink<BluetoothSerial>::State state = protolink.init(&SerialBT);
    //Serial.println(protolink.stateToString());

    while(SerialBT.connected() && state != ProtoLink<BluetoothSerial>::State::ERR) {
      delay(1);
      state = protolink.run();
      Serial.println(protolink.stateToString());
   
      if (state == ProtoLink<BluetoothSerial>::State::RECV_LEN) {
        SerialBT.write((uint8_t)state);
        //refreshDisplay();
      }
      yield();
      Serial.println(ESP.getFreeHeap());
    }

    Serial.println("Client disconnected");
    if (protolink.error_msg != NULL && state == ProtoLink<BluetoothSerial>::State::ERR) {
      Serial.print("Error: ");
      Serial.println(protolink.error_msg);
    }
  }
}