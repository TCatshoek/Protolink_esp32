#include <Arduino.h>
#include "protolink.h"
#include <WiFi.h>
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include "FastLED.h"
#include "credentials.h"

#define BRIGHTNESS  5
#define NUM_LEDS 4 * 4 * 3
CRGB leds[NUM_LEDS];

extern const char* ssid;
extern const char* password;

WiFiServer server(8080);
ProtoLink<WiFiClient> protolink((uint8_t*)&leds, sizeof(CRGB) * NUM_LEDS);

void setup() {
esp_log_level_set("*", ESP_LOG_VERBOSE);

  FastLED.addLeds<NEOPIXEL, 13>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  Serial.begin(9600);
  pinMode(5, OUTPUT);      // set the LED pin mode

  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  server.begin();

}

void loop() {
  FastLED.show();
  delay(100);

  WiFiClient client = server.available(); 

  if (client) {
    Serial.println("Client connected");
    ProtoLink<WiFiClient>::State state = protolink.init(&client);
    Serial.println(protolink.stateToString());

    while(client.connected() && state != ProtoLink<WiFiClient>::State::ERR) {
      state = protolink.run();
      //Serial.println(protolink.stateToString());
      FastLED.show();
      yield();
      if (state == ProtoLink<WiFiClient>::State::RECV_LEN) {
        client.write((uint8_t)state);
      }
      //Serial.println(ESP.getFreeHeap());
    }

    Serial.println("Client disconnected");
    if (protolink.error_msg != NULL && state == ProtoLink<WiFiClient>::State::ERR) {
      Serial.print("Error: ");
      Serial.println(protolink.error_msg);
    }
  }
}