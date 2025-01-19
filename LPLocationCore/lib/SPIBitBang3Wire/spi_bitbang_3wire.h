#pragma once

#include <Arduino.h>

// 3-wire SPI bit-bang implementation
// CPOL and CPHA are 1, so clock is normally high and data is sampled on the rising edge
// Default working frequency is 500kHz

class SPIBitBang3Wire {
public:
    void begin(int sckPin, int mosiMisoPin, int csPin, int resetPin, uint32_t spiSpeeddelayUs = 1);
    void reset();
    void write(uint8_t address, uint8_t data);
    uint8_t read(uint8_t address);
    
private:
    int _sckPin;
    int _mosiMisoPin;
    int _csPin;
    int _resetPin;
    uint32_t _spiSpeedDelayUs;
    
    void setPinModes();
    void select();
    void deselect();
};