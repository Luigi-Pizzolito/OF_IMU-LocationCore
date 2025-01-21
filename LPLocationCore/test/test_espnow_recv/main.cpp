#include <Arduino.h>

#include <WiFi.h>
#include <esp_wifi.h>
#include <QuickEspNow.h>

#include <LiteLED.h>
LiteLED led(LED_STRIP_WS2812, 0);
static const crgb_t L_GREEN = 0xff0000;
static const crgb_t L_RED = 0x00ff00;
static const crgb_t L_BLUE = 0x0000ff;

void dataReceived (uint8_t* address, uint8_t* data, uint8_t len, signed int rssi, bool broadcast) {
    // Serial.println(millis());
    // Serial.print("Received: ");
    // Serial.printf("%.*s\n", len, data);
    // Serial.printf("RSSI: %d dBm\n", rssi);
    // Serial.printf("From: " MACSTR "\n", MAC2STR (address));
    // Serial.printf("%s\n", broadcast ? "Broadcast" : "Unicast");
    // Serial.println();
    led.brightness(255,1);
    Serial.printf("%d %.*s\n", millis(), len, data);
    Serial.println();
    led.brightness(0,1);
}

void setup () {
    led.begin(21,1);
    led.brightness(30);
    led.setPixel(0,L_GREEN,1);

    Serial.begin(115200);
    delay(2000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    quickEspNow.onDataRcvd(dataReceived);
    quickEspNow.begin(1);
    delay(100);

    Serial.println("Ready to receive data on channel 1");
    led.brightness(0,1);
    led.setPixel(0,L_BLUE);
}

void loop () {
    delay(100);
}