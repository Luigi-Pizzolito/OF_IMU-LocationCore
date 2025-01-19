#pragma once

#include "pmw3610.h"
#include <Arduino.h>

// PMW3610 driver class, using bit-banged 3-wire SPI implementation

class PMW3610Driver {
public:
    /* PMW3610 driver implementation */
    bool begin(int sckPin, int mosiMisoPin, int csPin, int irqPin, int resetPin);
    void read_test();

private:
    /* Bit-banged 3-wire SPI implementation & functions */
    int _sckPin;
    int _mosiMisoPin;
    int _csPin;
    int _resetPin;
    int _irqPin;
    uint32_t _spiSpeedDelayUs;

    void _SPI__setPinModes();
    void _SPI__select();
    void _SPI__deselect();

    void _SPI_begin(int sckPin, int mosiMisoPin, int csPin, int resetPin, uint32_t spiSpeedDelayUs = 1);
    void _SPI_reset();
    void _SPI_write(uint8_t address, uint8_t data);
    uint8_t _SPI_read(uint8_t address);

    /* PMW3610 driver functions */
    bool _check_product_id();
    bool _self_test();
};