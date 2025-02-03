#pragma once

#include "PMW3610.h"
#include <Arduino.h>
#if PMW3610_USE_PIN_ISR
    #include <FunctionalInterrupt.h>
#endif

// PMW3610 driver class, using bit-banged 3-wire SPI implementation

/* Class hard-configurations */
#define PMW3610_MOTION_DATA_UPDATE_RATE_MS  5
#define PMW3610_TASK_PRIORITY               1
#define PMW3610_TASK_CORE                   1
#define PMW3610_TASK_STACK_SIZE             2048
#define PMW3610_USE_PIN_ISR                 false

// #define DEBUG

// Struct to hold sensor data
struct PMW3610Data {
    bool     motion;
    int16_t  delta_x_counts;
    int16_t  delta_y_counts;
    float    delta_x;
    float    delta_y;
    uint16_t squal;
    bool     err;
    bool     ovf;
};

// PMW3610 driver class
class PMW3610Driver {
   public:
    /* PMW3610 driver implementation */
    PMW3610Driver();
    ~PMW3610Driver();

    PMW3610Data data;
    #ifdef PMW3610_ENABLE_FRAME_CAPTURE
        uint8_t     frame_data[484];
    #endif

    bool        begin(int sckPin, int mosiMisoPin, int csPin, int irqPin, int resetPin, bool autocapture = true);
    void        update();
    void        printData();
    #ifdef PMW3610_ENABLE_FRAME_CAPTURE
        void    capture_frame();
        void    print_frame_as_pgm();
    #endif

   private:
    /* Bit-banged 3-wire SPI implementation & functions */
    int      _sckPin;
    int      _mosiMisoPin;
    int      _csPin;
    int      _resetPin;
    int      _irqPin;
    uint32_t _spiSpeedDelayUs;

    void _SPI__setPinModes();
    void _SPI__select();
    void _SPI__deselect();

    void    _SPI_begin(int sckPin, int mosiMisoPin, int csPin, int resetPin, uint32_t spiSpeedDelayUs = 1);
    void    _SPI_reset();
    void    __SPI_write(uint8_t address, uint8_t data);
    void    _SPI_write(uint8_t address, uint8_t data);
    uint8_t _SPI_read(uint8_t address);
    void    _SPI_write_burst(const uint8_t *addresses, const uint8_t *datas, size_t len);

    /* PMW3610 driver functions */
    bool _check_product_id();
    bool _self_test();

    bool _set_cpi(uint32_t cpi);
    bool _set_downshift_time(uint8_t reg_addr, uint32_t downshift_time);
    bool _set_sample_time(uint8_t reg_addr, uint32_t sample_time);
    bool _configure();

#ifdef PMW3610_SMART_ALGORITHM
    bool _smart_algorithm_flag = true;
#endif
    bool _motion_burst_read(uint8_t *motion_data, size_t len);
    bool _motion_burst_parse();

#if PMW3610_USE_PIN_ISR
    volatile bool         _intPinLow = false;
    static PMW3610Driver *_instance;
    static void IRAM_ATTR _intISR();
#endif
    TaskHandle_t _intTaskHandle = NULL;
    static void  _intTask(void *pvParameters);
};