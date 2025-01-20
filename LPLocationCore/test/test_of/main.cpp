#include <Arduino.h>
#include <pmw3610_driver.h>

const int CS_PIN = 40; // Chip Select pin
const int SCK_PIN = 41; // Serial Clock pin
const int DIO_PIN = 42; // Data I/O pin
const int RST_PIN = 39; // Reset pin
const int IRQ_PIN = 38; // Interrupt pin

PMW3610Driver pmw;

void setup() {
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

}

void loop() {
    pmw.read_test(); // Read sensor data

    delay(250); // Wait for 1 second before reading again
}