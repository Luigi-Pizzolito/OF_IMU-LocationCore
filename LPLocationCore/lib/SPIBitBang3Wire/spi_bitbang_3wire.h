#ifndef SPI_BITBANG_3WIRE_H
#define SPI_BITBANG_3WIRE_H

#include <Arduino.h>

class SPIBitBang3Wire {
public:
    void begin(int sckPin, int mosiMisoPin, int csPin, int resetPin);
    void reset();
    void write(uint8_t address, uint8_t data);
    uint8_t read(uint8_t address);
    
private:
    int _sckPin;
    int _mosiMisoPin;
    int _csPin;
    int _resetPin;
    
    void setPinModes();
    void select();
    void deselect();
};

#endif // SPI_BITBANG_3WIRE_H