#include <Arduino.h>
#include "protolink.h"
#include <WiFi.h>
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include "FastLED.h"
#include "credentials.h"
#include <BluetoothSerial.h>

#define BRIGHTNESS  5
#define NUM_LEDS 4 * 4 * 3
CRGB leds[NUM_LEDS];
uint8_t bs[1024];

// extern const char* ssid;
// extern const char* password;

// WiFiServer server(8080);
ProtoLink<BluetoothSerial> protolink((uint8_t*)&leds, sizeof(CRGB) * NUM_LEDS + 1024);

BluetoothSerial SerialBT;

void setup() {
  esp_log_level_set("*", ESP_LOG_VERBOSE);

  FastLED.addLeds<NEOPIXEL, 13>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  Serial.begin(9600);
  // pinMode(5, OUTPUT);      // set the LED pin mode

  // delay(10);

  // // We start by connecting to a WiFi network
  // Serial.println();
  // Serial.println();
  // Serial.print("Connecting to ");
  // Serial.println(ssid);

  // WiFi.begin(ssid, password);

  // while (WiFi.status() != WL_CONNECTED) {
  //     delay(500);
  //     Serial.print(".");
  // }

  // Serial.println("");
  // Serial.println("WiFi connected.");
  // Serial.println("IP address: ");
  // Serial.println(WiFi.localIP());
  
  // server.begin();
  SerialBT.enableSSP();
  SerialBT.begin("Beeper");
  Serial.println("The device started, now you can pair it with bluetooth!");

}

void loop() {
  FastLED.show();
  delay(100);

  //WiFiClient client = server.available(); 

  if (SerialBT.available()) {
    Serial.println("Client connected");
    ProtoLink<BluetoothSerial>::State state = protolink.init(&SerialBT);
    //Serial.println(protolink.stateToString());

    while(SerialBT.connected() && state != ProtoLink<BluetoothSerial>::State::ERR) {
      state = protolink.run();
      //Serial.println(protolink.stateToString());
   
      if (state == ProtoLink<BluetoothSerial>::State::RECV_LEN) {
        SerialBT.write((uint8_t)state);
        FastLED.show();
      }
      yield();
      //Serial.println(ESP.getFreeHeap());
    }

    Serial.println("Client disconnected");
    if (protolink.error_msg != NULL && state == ProtoLink<BluetoothSerial>::State::ERR) {
      Serial.print("Error: ");
      Serial.println(protolink.error_msg);
    }
  }
}