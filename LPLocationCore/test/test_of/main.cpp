#include <Arduino.h>
#include <spi_bitbang_3wire.h>

const int CS_PIN = 40; // Chip Select pin
const int SCK_PIN = 41; // Serial Clock pin
const int DIO_PIN = 42; // Data I/O pin
const int RST_PIN = 39; // Reset pin

SPIBitBang3Wire spi;

void setup() {
    Serial.begin(115200); // Initialize serial communication
    delay(2000); // Wait for the serial monitor to open

    Serial.println("Starting SPI communication...");
    spi.begin(SCK_PIN, DIO_PIN, CS_PIN, RST_PIN); // Initialize the SPI communication
    spi.reset(); // Reset the device

}

void loop() {
    uint8_t reg0 = spi.read(0x00);
    uint8_t reg1 = spi.read(0x01);
    uint8_t reg2 = spi.read(0x02);
    uint8_t reg3 = spi.read(0x03);
    uint8_t reg4 = spi.read(0x04);
    uint8_t reg5 = spi.read(0x05);

    // Print the register values to the serial monitor
    Serial.print("Register 0x00: ");
    Serial.println(reg0, HEX);
    Serial.print("Register 0x01: ");
    Serial.println(reg1, HEX);
    Serial.print("Register 0x02: ");
    Serial.println(reg2, HEX);
    Serial.print("Register 0x03: ");
    Serial.println(reg3, HEX);
    Serial.print("Register 0x04: ");
    Serial.println(reg4, HEX);
    Serial.print("Register 0x05: ");
    Serial.println(reg5, HEX);

    delay(1000); // Wait for 1 second before reading again
}