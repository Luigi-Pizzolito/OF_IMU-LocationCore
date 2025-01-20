#pragma once

#include "pmw3610.h"
#include <Arduino.h>

// PMW3610 driver class, using bit-banged 3-wire SPI implementation
#define DEBUG

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
    void __SPI_write(uint8_t address, uint8_t data);
    void _SPI_write(uint8_t address, uint8_t data);
    uint8_t _SPI_read(uint8_t address);
    void _SPI_write_burst(const uint8_t *addresses, const uint8_t *datas, size_t len);

    /* PMW3610 driver functions */
    bool _check_product_id();
    bool _self_test();

    bool _set_cpi(uint32_t cpi);
    bool _set_downshift_time(uint8_t reg_addr, uint32_t downshift_time);
    bool _set_sample_time(uint8_t reg_addr, uint32_t sample_time);
    bool _configure();

    bool _motion_burst_read(uint8_t *motion_data, size_t len);
};