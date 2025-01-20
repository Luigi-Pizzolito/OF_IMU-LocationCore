#include <Arduino.h>

#include <WiFi.h>
#include <esp_wifi.h>
#include <QuickEspNow.h>

void dataReceived (uint8_t* address, uint8_t* data, uint8_t len, signed int rssi, bool broadcast) {
    Serial.print ("Received: ");
    Serial.printf ("%.*s\n", len, data);
    Serial.printf ("RSSI: %d dBm\n", rssi);
    Serial.printf ("From: " MACSTR "\n", MAC2STR (address));
    Serial.printf ("%s\n", broadcast ? "Broadcast" : "Unicast");
}

static const String msg = "Hello ESP-NOW!";
#define DEST_ADDR ESPNOW_BROADCAST_ADDRESS

void setup() {
    Serial.begin(115200);
    delay(2000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    quickEspNow.onDataRcvd(dataReceived);
    quickEspNow.begin(1);
    delay(100);

    Serial.println("Ready to send data");
}

void loop() {
    static unsigned int count = 0;
    Serial.print(millis());
    String message = String(msg) + " " + String(count++);
    if (quickEspNow.send(DEST_ADDR, (uint8_t*)message.c_str(), message.length())) {
        Serial.println(" Data sent");
    } else {
        Serial.println(" Data not sent");
    }
    delay(1000);
}
