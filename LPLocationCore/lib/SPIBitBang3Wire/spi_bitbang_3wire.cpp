#include "spi_bitbang_3wire.h"

#define BASE_DELAY 5000 // Base delay in microseconds

void SPIBitBang3Wire::begin(int sckPin, int mosiMisoPin, int csPin, int resetPin) {
    _sckPin = sckPin;
    _mosiMisoPin = mosiMisoPin;
    _csPin = csPin;
    _resetPin = resetPin;
    setPinModes();
}

void SPIBitBang3Wire::setPinModes() {
    pinMode(_sckPin, OUTPUT);
    pinMode(_mosiMisoPin, OUTPUT);
    pinMode(_csPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    digitalWrite(_csPin, HIGH); // Deselect
    digitalWrite(_resetPin, HIGH); // Reset inactive
    delay(10); // Wait for power to stabilize
}

void SPIBitBang3Wire::select() {
    digitalWrite(_csPin, LOW); // Select
    delayMicroseconds(BASE_DELAY*10);
}

void SPIBitBang3Wire::deselect() {
    digitalWrite(_csPin, HIGH); // Deselect
    digitalWrite(_sckPin, HIGH); // Clock high
    delayMicroseconds(BASE_DELAY*10);
}

void SPIBitBang3Wire::reset() {
    digitalWrite(_resetPin, LOW); // Activate reset
    delay(10); // Hold reset for 10ms
    digitalWrite(_resetPin, HIGH); // Deactivate reset
}

void SPIBitBang3Wire::write(uint8_t address, uint8_t data) {
    address |= 0x80; // Ensure MSB of the address is 1 for write operation
    pinMode(_mosiMisoPin, OUTPUT); // Set MOSI/MISO as output to send address and data
    select();
    // Send address
    for (int i = 0; i < 8; i++) {
        digitalWrite(_mosiMisoPin, (address & 0x80) ? HIGH : LOW); // Write MSB first
        address <<= 1;
        digitalWrite(_sckPin, LOW); // Clock high
        delayMicroseconds(BASE_DELAY);        // Small delay
        digitalWrite(_sckPin, HIGH);  // Clock low
        delayMicroseconds(BASE_DELAY);        // Small delay
    }
    // Send data
    for (int i = 0; i < 8; i++) {
        digitalWrite(_mosiMisoPin, (data & 0x80) ? HIGH : LOW); // Write MSB first
        data <<= 1;
        digitalWrite(_sckPin, LOW); // Clock high
        delayMicroseconds(BASE_DELAY);        // Small delay
        digitalWrite(_sckPin, HIGH);  // Clock low
        delayMicroseconds(BASE_DELAY);        // Small delay
    }
    deselect();
}

uint8_t SPIBitBang3Wire::read(uint8_t address) {
    uint8_t data = 0;
    address &= 0x7F; // Ensure MSB of the address is 0 for read operation
    pinMode(_mosiMisoPin, OUTPUT); // Set MOSI/MISO as output to send address
    select();
    // Send address
    for (int i = 0; i < 8; i++) {
        digitalWrite(_mosiMisoPin, (address & 0x80) ? HIGH : LOW); // Write MSB first
        address <<= 1;
        digitalWrite(_sckPin, LOW); // Clock high
        delayMicroseconds(BASE_DELAY);        // Small delay
        digitalWrite(_sckPin, HIGH);  // Clock low
        delayMicroseconds(BASE_DELAY);        // Small delay
    }
    pinMode(_mosiMisoPin, INPUT); // Set MOSI/MISO as input to read data
    
    delayMicroseconds(5*BASE_DELAY); // Handover delay

    // Read data
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        digitalWrite(_sckPin, LOW); // Clock high
        delayMicroseconds(BASE_DELAY);        // Small delay
        if (digitalRead(_mosiMisoPin)) {
            data |= 0x01;
        }
        digitalWrite(_sckPin, HIGH);  // Clock low
        delayMicroseconds(BASE_DELAY);        // Small delay
    }
    deselect();
    pinMode(_mosiMisoPin, OUTPUT); // Set MOSI/MISO back to output
    return data;
}