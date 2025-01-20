#include <Arduino.h>
#include <pmw3610_driver.h>

const int CS_PIN = 40; // Chip Select pin
const int SCK_PIN = 41; // Serial Clock pin
const int DIO_PIN = 42; // Data I/O pin
const int RST_PIN = 39; // Reset pin
const int IRQ_PIN = 38; // Interrupt pin

PMW3610Driver pmw;

#include <WiFi.h>
#include <esp_wifi.h>
#include <QuickEspNow.h>

#include <LiteLED.h>
LiteLED led(LED_STRIP_WS2812, 0);
static const crgb_t L_RED = 0xff0000;
static const crgb_t L_GREEN = 0x00ff00;
static const crgb_t L_BLUE = 0x0000ff;

void setup() {
    led.begin(16,1);
    led.brightness(30);
    led.setPixel(0,L_GREEN,1);

    Serial.begin(115200); // Initialize serial communication
    delay(2000); // Wait for the serial monitor to open

    Serial.println("Starting pmw communication...");
    if (
        pmw.begin(SCK_PIN, DIO_PIN, CS_PIN, RST_PIN, IRQ_PIN) // Initialize the sensor
    ) {
        Serial.println("PMW3610 sensor initialized successfully!");
    } else {
        Serial.println("PMW3610 sensor initialization failed!");
        while (1); // Stop the program if sensor initialization fails
    }

    delay(1);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // quickEspNow.onDataRcvd(dataReceived);
    quickEspNow.begin(1);
    delay(100);

    Serial.println("Ready to receive data on channel 1");
    led.brightness(0,1);
    led.setPixel(0,L_BLUE);

}

void loop() {
    led.brightness(255,1);
    String msg = pmw.read_test(); // Read sensor data
    quickEspNow.send(ESPNOW_BROADCAST_ADDRESS, (uint8_t*)msg.c_str(), msg.length()); // Send sensor data over ESP-NOW
    led.brightness(0,1);
    delay(50); // Wait for 1 second before reading again
}