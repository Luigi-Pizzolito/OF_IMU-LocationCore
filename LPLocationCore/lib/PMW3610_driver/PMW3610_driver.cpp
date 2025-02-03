#include "PMW3610_driver.h"

/* Constructor */
#if PMW3610_USE_PIN_ISR
PMW3610Driver *PMW3610Driver::_instance = nullptr;
PMW3610Driver::PMW3610Driver() {
    _instance = this;
}
#else
PMW3610Driver::PMW3610Driver() {}
#endif
PMW3610Driver::~PMW3610Driver() {
    // Clean up
    // Stop task
    vTaskDelete(_intTaskHandle);
    // Detach interrupt
    detachInterrupt(digitalPinToInterrupt(_irqPin));
}

/* Bit-banged 3-wire SPI functions */
// CPOL and CPHA are 1, so clock is normally high and data is sampled on the rising edge
// Default working frequency is 500kHz

void PMW3610Driver::_SPI_begin(int sckPin, int mosiMisoPin, int csPin, int resetPin, uint32_t spiSpeedDelayUs) {
    // Save configuration
    _sckPin          = sckPin;
    _mosiMisoPin     = mosiMisoPin;
    _csPin           = csPin;
    _resetPin        = resetPin;
    _spiSpeedDelayUs = spiSpeedDelayUs;

    // Initialize pins
    _SPI__setPinModes();
}

void PMW3610Driver::_SPI__setPinModes() {
    // Set pin modes
    pinMode(_sckPin, OUTPUT);
    pinMode(_mosiMisoPin, OUTPUT);
    pinMode(_csPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);

    // Set initial states
    digitalWrite(_csPin, HIGH);     // Deselect
    digitalWrite(_resetPin, HIGH);  // Reset inactive
    digitalWrite(_sckPin, HIGH);    // Clock high idle

    delay(PMW3610_WAKEUP_TIME_MS * 5);  // Wait for power to stabilize (50ms on initial power-up)
}

void PMW3610Driver::_SPI_reset() {
    // Reset the sensor
    digitalWrite(_resetPin, LOW);                   // Activate reset
    delayMicroseconds(PMW3610_RESET_HOLD_TIME_US);  // Hold reset
    digitalWrite(_resetPin, HIGH);                  // Deactivate reset
    delay(PMW3610_WAKEUP_TIME_MS);                  // Wait for reset to complete
}

void PMW3610Driver::_SPI__select() {
    digitalWrite(_csPin, LOW);  // Select
    delayMicroseconds(PMW3610_NCS_SETUP_TIME_US);
}

void PMW3610Driver::_SPI__deselect() {
    delayMicroseconds(PMW3610_NCS_SETUP_TIME_US);
    digitalWrite(_csPin, HIGH);   // Deselect
    digitalWrite(_sckPin, HIGH);  // Clock high idle
}

uint8_t PMW3610Driver::_SPI_read(uint8_t address) {
    // Check if the address is a valid
    if (address > 0x7F) {
        Serial.println("Invalid address for read operation!");
        return 0;
    }
    uint8_t data = 0;
    address &= 0x7F;                // Ensure MSB of the address is 0 for read operation
    pinMode(_mosiMisoPin, OUTPUT);  // Set MOSI/MISO as output to send address
    _SPI__select();

    // Send address, sensor reads on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        digitalWrite(_sckPin, LOW);           // Clock low
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay

        // Shift-out address
        digitalWrite(_mosiMisoPin, (address & 0x80) ? HIGH : LOW);  // Write MSB first
        address <<= 1;

        digitalWrite(_sckPin, HIGH);          // Clock high
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay
    }

    pinMode(_mosiMisoPin, INPUT);  // Set MOSI/MISO as input to read data

    delayMicroseconds(PMW3610_HANDOVER_TIME_US);  // Handover delay

    // Read data, sensor sends on high to low CLK transition, read on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        digitalWrite(_sckPin, LOW);           // Clock low
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay

        if (digitalRead(_mosiMisoPin)) {  // Shift-in read data
            data |= 0x01;
        }

        digitalWrite(_sckPin, HIGH);          // Clock high
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay
    }

    _SPI__deselect();
    delayMicroseconds(PMW3610_HOLD_TIME_US);  // Hold time after a read operation
    pinMode(_mosiMisoPin, OUTPUT);            // Set MOSI/MISO back to output

    delayMicroseconds(PMW3610_INTERREAD_TIME_US);  // Inter-command read minimum delay

    return data;
}

void PMW3610Driver::__SPI_write(uint8_t address, uint8_t data) {
    address |= 0x80;                // Ensure MSB of the address is 1 for write operation
    pinMode(_mosiMisoPin, OUTPUT);  // Set MOSI/MISO as output to send address and data
    _SPI__select();

    // Send address, sensor reads on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        digitalWrite(_sckPin, LOW);           // Clock low
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay

        // Shift-out address
        digitalWrite(_mosiMisoPin, (address & 0x80) ? HIGH : LOW);  // Write MSB first
        address <<= 1;

        digitalWrite(_sckPin, HIGH);          // Clock high
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay
    }

    // Send data, sensor reads on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        digitalWrite(_sckPin, LOW);           // Clock low
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay

        // Shift-out write data
        digitalWrite(_mosiMisoPin, (data & 0x80) ? HIGH : LOW);  // Write MSB first
        data <<= 1;

        digitalWrite(_sckPin, HIGH);          // Clock high
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay
    }

    delayMicroseconds(PMW3610_HOLD_TIME_US * 10);  // Hold time after a write operation
    _SPI__deselect();

    delayMicroseconds(PMW3610_INTERWRITE_TIME_US);  // Inter-command write minimum delay
}

void PMW3610Driver::_SPI_write(uint8_t address, uint8_t data) {
    // Check if the address is a valid
    if (address > 0x7F) {
        Serial.println("Invalid address for write operation!");
        return;
    }

    // Enable device SPI clock
    __SPI_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    delayMicroseconds(1);

    // Write data
    __SPI_write(address, data);

    // Disable device SPI clock (to save power)
    __SPI_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
}

void PMW3610Driver::_SPI_write_burst(const uint8_t *addresses, const uint8_t *datas, size_t len) {
    // Enable device SPI clock
    __SPI_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);

    // Write multiple registers in burst mode
    for (size_t i = 0; i < len; i++) {
        __SPI_write(addresses[i], datas[i]);
    }

    // Disable device SPI clock (to save power)
    __SPI_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
}

/* PMW3610 Driver private functions */

bool PMW3610Driver::_check_product_id() {
    // Check product ID
    uint8_t productID = _SPI_read(PMW3610_REG_PRODUCT_ID);
    if (productID != PMW3610_PRODUCT_ID) {
        Serial.println("PMW3610 product ID mismatch!");
        return false;
    }

    // Check revision ID
    uint8_t revisionID = _SPI_read(PMW3610_REG_REVISION_ID);
    if (revisionID != PMW3610_REVISION_ID) {
        Serial.println("PMW3610 revision ID mismatch!");
        return false;
    }

    return true;
}

bool PMW3610Driver::_self_test() {
    // Perform self-test

    // Step 1: Clear observation register (OB1)
    _SPI_write(PMW3610_REG_OBSERVATION, 0x00);

    // Wait for the sensor to clear the observation register
    // and perform the self-test
    delay(PMW3610_INIT_SELF_TEST_MS);

    // Step 2: Check observation register (OB1)
    uint8_t observation = _SPI_read(PMW3610_REG_OBSERVATION);
    // passed if lower nibble is all set
    if ((observation & 0x0F) != 0x0F) {
        Serial.println("PMW3610 self-test failed!");
        Serial.print("Observation register: ");
        Serial.println(observation, HEX);
        return false;
    }

#ifdef PMW3610_EXTENDED_SELF_TEST
    // Perform extended self-test
    // Step 1: Reset
    _SPI_write(PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    delay(PMW3610_WAKEUP_TIME_MS);
    // Step 2: Start extended self-test
    _SPI_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    delayMicroseconds(1);
    _SPI_write(PMW3610_REG_SMART_EN, 0x10);
    _SPI_write(PMW3610_REG_SELF_TEST, 0x01);
    delay(64 * 4);  // Wait for 64 frames
    delay(PMW3610_INIT_SELF_TEST_MS);
    // Step 3: Check results
    uint8_t crc[4] = {0x00, 0x00, 0x00};
    crc[0]         = _SPI_read(PMW3610_REG_CRC0);
    crc[1]         = _SPI_read(PMW3610_REG_CRC1);
    crc[2]         = _SPI_read(PMW3610_REG_CRC2);
    crc[3]         = _SPI_read(PMW3610_REG_CRC3);
    if (crc[0] != 0x73 || crc[1] != 0xc4 || crc[2] != 0xc1 || crc[3] != 0xEA) {
        Serial.println("PMW3610 extended self-test failed!");
        Serial.print("CRC0: ");
        Serial.print(crc[0], HEX);
        Serial.print(", CRC1: ");
        Serial.print(crc[1], HEX);
        Serial.print(", CRC2: ");
        Serial.print(crc[2], HEX);
        Serial.print(", CRC3: ");
        Serial.println(crc[3], HEX);
        return false;
    } else {
        _SPI_reset();
        delay(PMW3610_WAKEUP_TIME_MS);
    }
#endif

    return true;
}

bool PMW3610Driver::_set_cpi(uint32_t cpi) {
    /* Set resolution with CPI step of 200 cpi (200-3200 cpi)
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * 0x4: 800 cpi
     * 0x5: 1000 cpi
     * 0x6: 1200 cpi
     * 0x7: 1400 cpi
     * 0x8: 1600 cpi
     * 0x9: 1800 cpi
     * 0xA: 2000 cpi
     * 0xB: 2200 cpi
     * 0xC: 2400 cpi
     * 0xD: 2600 cpi
     * 0xE: 2800 cpi
     * 0xF: 3000 cpi
     * 0x10: 3200 cpi (maximum cpi)
     */

    // Check bounds
    if (cpi < PMW3610_MIN_CPI) {
        Serial.print("Requested invalid CPI: ");
        Serial.print(cpi);
        Serial.print(", min CPI is: ");
        Serial.println(PMW3610_MIN_CPI);
        return false;
    } else if (cpi > PMW3610_MAX_CPI) {
        Serial.print("Requested invalid CPI: ");
        Serial.print(cpi);
        Serial.print(", max CPI is: ");
        Serial.println(PMW3610_MIN_CPI);
        return false;
    }

    // Convert CPI to register value
    uint8_t cpiReg = (cpi / 200);
#ifdef DEBUG
    Serial.print("Setting CPI to ");
    Serial.print(cpi);
    Serial.print(" cpi (register value: ");
    Serial.print(cpiReg, HEX);
    Serial.println(")");
#endif

    // Set CPI
    uint8_t addresses[] = {PMW3610_REG_SPI_PAGE0, PMW3610_REG_RES_STEP, PMW3610_REG_SPI_PAGE0};
    uint8_t datas[]     = {0xFF, cpiReg, 0x00};
    _SPI_write_burst(addresses, datas, sizeof(addresses));

    return true;
}

bool PMW3610Driver::_set_downshift_time(uint8_t reg_addr, uint32_t downshift_time) {
    /* Set downshift time in ms. */
    // NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
    // The pos-mode rate is configured in pmw3610_async_init_configure

    // Compute bounds
    uint32_t max_time;
    uint32_t min_time;
    switch (reg_addr) {
        case PMW3610_REG_RUN_DOWNSHIFT:
            /*
             * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
             *                      * 8 * pos-rate (fixed to 4ms)
             */
            max_time = 32 * 255;
            min_time = 32;
            break;
        case PMW3610_REG_REST1_DOWNSHIFT:
            /*
             * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
             *                        * 16 * Rest1_sample_period (default 40 ms)
             */
            max_time = 255 * 16 * PMW3610_DEFAULT_REST1_SAMPLE_TIME_MS;
            min_time = 16 * PMW3610_DEFAULT_REST1_SAMPLE_TIME_MS;
            break;
        case PMW3610_REG_REST2_DOWNSHIFT:
            /*
             * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
             *                        * 128 * Rest2 rate (default 100 ms)
             */
            max_time = 255 * 128 * PMW3610_DEFAULT_REST2_SAMPLE_TIME_MS;
            min_time = 128 * PMW3610_DEFAULT_REST2_SAMPLE_TIME_MS;
            break;
        default:
            Serial.println("Invalid register address for downshift time!");
            return false;
    }

    // Check bounds
    if (((downshift_time > max_time) || (downshift_time < min_time)) ||
        !((min_time > 0) && (max_time / min_time <= UINT8_MAX))) {
        Serial.print("Requested invalid downshift time for reg [");
        Serial.print(reg_addr, HEX);
        Serial.print("]: ");
        Serial.print(downshift_time);
        Serial.print(", min downshift time is: ");
        Serial.print(min_time);
        Serial.print(", max downshift time is: ");
        Serial.println(max_time);
        return false;
    }

    // Convert downshift time to register value
    uint8_t downshift_time_reg = (downshift_time / min_time);
#ifdef DEBUG
    Serial.print("Setting downshift time for reg [");
    Serial.print(reg_addr, HEX);
    Serial.print("] to ");
    Serial.print(downshift_time);
    Serial.print(" ms (register value: ");
    Serial.print(downshift_time_reg, HEX);
    Serial.println(")");
#endif

    // Set downshift time
    _SPI_write(reg_addr, downshift_time_reg);

    return true;
}

bool PMW3610Driver::_set_sample_time(uint8_t reg_addr, uint32_t sample_time) {
    /* Set sampling rate for each mode in ms. */

    // Check bounds
    if (sample_time < PWM3610_MIN_SAMPLE_TIME_MS) {
        Serial.print("Requested invalid sample time: ");
        Serial.print(sample_time);
        Serial.print(", min sample time is: ");
        Serial.println(PWM3610_MIN_SAMPLE_TIME_MS);
        return false;
    } else if (sample_time > PWM3610_MAX_SAMPLE_TIME_MS) {
        Serial.print("Requested invalid sample time: ");
        Serial.print(sample_time);
        Serial.print(", max sample time is: ");
        Serial.println(PWM3610_MAX_SAMPLE_TIME_MS);
        return false;
    }

    // Convert sample time to register value
    uint8_t sample_time_reg = (sample_time / PWM3610_MIN_SAMPLE_TIME_MS);
#ifdef DEBUG
    Serial.print("Setting sample time for reg [");
    Serial.print(reg_addr, HEX);
    Serial.print("] to ");
    Serial.print(sample_time);
    Serial.print(" ms (register value: ");
    Serial.print(sample_time_reg, HEX);
    Serial.println(")");
#endif

    // Set sample time
    _SPI_write(reg_addr, sample_time_reg);

    return true;
}

bool PMW3610Driver::_configure() {
    // Configure sensor

    // 1. Clear motion registers
    //      PMW3610_REG_MOTION
    //      PMW3610_REG_DELTA_X_L
    //      PMW3610_REG_DELTA_Y_L
    //      PMW3610_REG_DELTA_XY_H
    for (uint8_t reg = PMW3610_REG_MOTION; reg <= PMW3610_REG_DELTA_XY_H; reg++) {
        _SPI_read(reg);
    }

    // 2. Set CPI (counts per inch)
    if (!_set_cpi(PMW3610_DEFAULT_CPI)) {
        return false;
    }

    // 3. Set performance register: run mode, vel_rate, poshi_rate, poslo_rate
    _SPI_write(PMW3610_REG_PERFORMANCE, PMW3610_DEFAULT_PERFORMANCE);

    // 4. Set downshift and rate registers
    if (!_set_downshift_time(PMW3610_REG_RUN_DOWNSHIFT, PMW3610_DEFAULT_RUN_DOWNSHIFT_TIME_MS)) {
        return false;
    }
    if (!_set_sample_time(PMW3610_REG_REST1_PERIOD, PMW3610_DEFAULT_REST1_SAMPLE_TIME_MS)) {
        return false;
    }
    if (!_set_downshift_time(PMW3610_REG_REST1_DOWNSHIFT, PMW3610_DEFAULT_REST1_DOWNSHIFT_TIME_MS)) {
        return false;
    }
#if PMW3610_DEFAULT_REST2_DOWNSHIFT_TIME_MS > 0
    if (!_set_downshift_time(PMW3610_REG_REST2_DOWNSHIFT, PMW3610_DEFAULT_REST2_DOWNSHIFT_TIME_MS)) {
        return false;
    }
#endif
#if PMW3610_DEFAULT_REST2_SAMPLE_TIME_MS >= 10
    if (!_set_sample_time(PMW3610_REG_REST2_PERIOD, PMW3610_DEFAULT_REST2_SAMPLE_TIME_MS)) {
        return false;
    }
#endif
#if PMW3610_DEFAULT_REST3_SAMPLE_TIME_MS >= 10
    if (!_set_sample_time(PMW3610_REG_REST3_PERIOD, PMW3610_DEFAULT_REST3_SAMPLE_TIME_MS)) {
        return false;
    }
#endif

    return true;
}

bool PMW3610Driver::_motion_burst_read(uint8_t *motion_data, size_t len) {
    // Check bounds
    if (len > PMW3610_MAX_BURST_SIZE) {
        Serial.print("Requested burst read size exceeds maximum burst size: ");
        Serial.print(len);
        Serial.print(", max burst size is: ");
        Serial.println(PMW3610_MAX_BURST_SIZE);
        return false;
    }

    // Read motion burst data
    pinMode(_mosiMisoPin, OUTPUT);  // Set MOSI/MISO as output to send address
    _SPI__select();

    // Send motion burst address
    uint8_t address = PMW3610_REG_MOTION_BURST & 0x7F;  // Ensure MSB of the address is 0 for read operation
    // Send address, sensor reads on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        digitalWrite(_sckPin, LOW);           // Clock low
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay

        // Shift-out address
        digitalWrite(_mosiMisoPin, (address & 0x80) ? HIGH : LOW);  // Write MSB first
        address <<= 1;

        digitalWrite(_sckPin, HIGH);          // Clock high
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay
    }

    pinMode(_mosiMisoPin, INPUT);  // Set MOSI/MISO as input to read data

    delayMicroseconds(PMW3610_HANDOVER_TIME_US);  // Handover delay

    // Read motion burst data
    for (size_t i = 0; i < len; i++) {
        motion_data[i] = 0;
        // Read data, sensor sends on high to low CLK transition, read on low to high CLK transition
        for (int j = 0; j < 8; j++) {
            motion_data[i] <<= 1;
            digitalWrite(_sckPin, LOW);           // Clock low
            delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay

            if (digitalRead(_mosiMisoPin)) {  // Shift-in read data
                motion_data[i] |= 0x01;
            }

            digitalWrite(_sckPin, HIGH);          // Clock high
            delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay
        }
    }

    _SPI__deselect();
    delayMicroseconds(PMW3610_HOLD_TIME_US);  // Hold time after a read operation
    pinMode(_mosiMisoPin, OUTPUT);            // Set MOSI/MISO back to output

    delayMicroseconds(PMW3610_INTERREAD_TIME_US);  // Inter-command read minimum delay

#ifdef PMW3610_SMART_ALGORITHM
                                                   // Smart algorithm to extends sensor tracking across a wider range of surfaces
#    ifdef DEBUG
    Serial.print("Shutter: ");
    Serial.print(motion_data[PMW3610_SHUTTER_H_POS]);
    Serial.print(" ");
    Serial.print((motion_data[PMW3610_SHUTTER_H_POS] == 0x00) ? "YES" : "NO");
    Serial.print(" ");
    Serial.print(motion_data[PMW3610_SHUTTER_L_POS]);
    Serial.print(" ");
    Serial.println((motion_data[PMW3610_SHUTTER_L_POS] < 45) ? "<45" : ">45");
#    endif

    if (_smart_algorithm_flag && motion_data[PMW3610_SHUTTER_H_POS] == 0x00 && motion_data[PMW3610_SHUTTER_L_POS] < 45) {
// Smart enable
#    ifdef DEBUG
        Serial.println("Smart algorithm enabled!");
#    endif

        _SPI_write(PMW3610_REG_SMART_EN, 0x00);
        _smart_algorithm_flag = false;
    }
    if (!_smart_algorithm_flag && motion_data[PMW3610_SHUTTER_H_POS] == 0x00 && motion_data[PMW3610_SHUTTER_L_POS] > 45) {
// Smart disable
#    ifdef DEBUG
        Serial.println("Smart algorithm disabled!");
#    endif

        _SPI_write(PMW3610_REG_SMART_EN, 0x80);
        _smart_algorithm_flag = true;
    }
#endif

    return true;
}

bool PMW3610Driver::_motion_burst_parse() {
    // Parse motion burst data
    uint8_t motion_data[PMW3610_BURST_SIZE];

    if (_motion_burst_read(motion_data, PMW3610_BURST_SIZE)) {
        // Parse motion data
        uint8_t motion_reg = motion_data[PMW3610_MOTION_POS];
        //      BIT 7: MOTION
        //      BIT 4: OVF
        //      BIT 3: LP_VALID
        //      BIT 2: LSR_FAULT
        // 1. Check motion bit
        data.motion = motion_reg & 0x80;
        // 2. Check error and overflow bits
        data.err = (motion_reg & 0x10) || !(motion_reg & 0x08) || (motion_reg & 0x04);
        data.ovf = motion_reg & 0x10;
        // 3. Parse delta X and Y
        data.delta_x_counts =
            TOINT16((motion_data[PMW3610_X_L_POS] + ((motion_data[PMW3610_XY_H_POS] & 0xF0) << 4)), 12) / PMW3610_DEFAULT_CPI_DIVISOR;
        data.delta_y_counts =
            TOINT16((motion_data[PMW3610_Y_L_POS] + ((motion_data[PMW3610_XY_H_POS] & 0x0F) << 8)), 12) / PMW3610_DEFAULT_CPI_DIVISOR;
        // Convert to delta mm
        data.delta_x = data.delta_x_counts * 25.4 / PMW3610_DEFAULT_CPI;
        data.delta_y = data.delta_y_counts * 25.4 / PMW3610_DEFAULT_CPI;
        // 4. Parse SQUAL
        data.squal = static_cast<uint16_t>(motion_data[PMW3610_SQUAL_POS]) << 1;

        return true;
    }

    return false;
}

/* PMW3610 Driver interrupt functions */
#if PMW3610_USE_PIN_ISR
// Interrupt service routine for pin-change interrupt
IRAM_ATTR void PMW3610Driver::_intISR() {
    if (_instance) {
        // Set flag
        _instance->_intPinLow = digitalRead(_instance->_irqPin) == LOW;
        // #ifdef DEBUG
        //     Serial.print("Interrupt detected! Pin state: ");
        //     Serial.println(_instance->_intPinLow ? "LOW" : "HIGH");
        // #endif
        // Notify task
        xTaskNotifyFromISR(_instance->_intTaskHandle, 0, eNoAction, NULL);
    }
}
#endif
// Task to periodically update motion data if motion is detected
// todo: add error handling on _motion_burst_parse
void PMW3610Driver::_intTask(void *pvParameters) {
    PMW3610Driver *driver = (PMW3610Driver *)pvParameters;
#ifdef DEBUG
    Serial.println("PMW3610 update task started!");
#endif

    while (true) {
#if PMW3610_USE_PIN_ISR
        // Wait for interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#    ifdef DEBUG
        Serial.println("PMW3610 update task received ISR notification!");
#    endif

        // Check if the interrupt pin is low (motion detected)
        while (driver->_intPinLow) {
            // Read motion data every Xms, when moving
            driver->_motion_burst_parse();
            // Delay
            vTaskDelay(pdMS_TO_TICKS(PMW3610_MOTION_DATA_UPDATE_RATE_MS));
        }

        // If pin is high (no motion detected), update motion data once
        // if (!driver->_intPinLow) {
        driver->_motion_burst_parse();
        // }

#else
        // Read motion data every Xms, always
        driver->_motion_burst_parse();

        // Delay
        vTaskDelay(pdMS_TO_TICKS(PMW3610_MOTION_DATA_UPDATE_RATE_MS));
#endif
    }
}

/* PMW3610 Driver public functions */

bool PMW3610Driver::begin(int sckPin, int mosiMisoPin, int csPin, int irqPin, int resetPin, bool autocapture) {
#ifdef DEBUG
    Serial.println("Initializing PMW3610 sensor...");
#endif

    // Step 1: Power up reset
    // Start SPI communication
    _SPI_begin(sckPin, mosiMisoPin, csPin, resetPin);
    // Reset the sensor via hardware reset pin
    _SPI_reset();
    // Alternatively, reset via software SPI command:
    // _SPI_write(PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    // Check product ID
    if (!_check_product_id()) {
        return false;
    }
#ifdef DEBUG
    Serial.println("PMW3610 product ID and revision ID match!");
#endif

    // Step 2: Perform self-test
    if (!_self_test()) {
        return false;
    }
#ifdef DEBUG
    Serial.println("PMW3610 self-test passed!");
#endif

// Step 4: Configure sensor
#ifdef DEBUG
    Serial.println("Configuring PMW3610 sensor...");
#endif
    if (!_configure()) {
        return false;
    }

    if (autocapture) {
            // Step 5: Set up interrupt & start task
        // todo: add error handling on interrupt & task setup
        // Setup interrupt pin
        _irqPin = irqPin;
        pinMode(_irqPin, INPUT);
    #if PMW3610_USE_PIN_ISR
        // Attach interrupt
        // attachInterrupt(digitalPinToInterrupt(_irqPin), std::bind(&PMW3610Driver::_intISR, this), CHANGE);
        attachInterrupt(digitalPinToInterrupt(_irqPin), &PMW3610Driver::_intISR, CHANGE);
    #endif

        // Create task
        BaseType_t xReturned;
        xReturned = xTaskCreatePinnedToCore(_intTask, "PMW3610INTPinMonitor", PMW3610_TASK_STACK_SIZE, this, PMW3610_TASK_PRIORITY, &_intTaskHandle, PMW3610_TASK_CORE);
        if (xReturned != pdPASS) {
            Serial.println("PMW3610 update task creation failed!");
            return false;
        }
    }

#ifdef DEBUG
    Serial.println("PMW3610 sensor configured successfully!");
#endif

    return true;
}

void PMW3610Driver::update() {
    // Update motion data
    _motion_burst_parse();
}

void PMW3610Driver::printData() {
    Serial.print("{");
    Serial.print("\"motion\": ");
    Serial.print(data.motion ? "true" : "false");
    Serial.print(", ");
    Serial.print("\"delta_x\": ");
    Serial.print(data.delta_x);
    Serial.print(", ");
    Serial.print("\"delta_y\": ");
    Serial.print(data.delta_y);
    Serial.print(", ");
    Serial.print("\"squal\": ");
    Serial.print(data.squal);
    Serial.print(", ");
    Serial.print("\"error\": ");
    Serial.print(data.err ? "true" : "false");
    Serial.print(", ");
    Serial.print("\"overflow\": ");
    Serial.print(data.ovf ? "true" : "false");
    Serial.println("}");
}

#ifdef PMW3610_ENABLE_FRAME_CAPTURE
void PMW3610Driver::capture_frame() {
    // Capture frame data

    // Step 1: Pause task
    vTaskSuspend(_intTaskHandle);

    // Step 2: Send frame grab command
    __SPI_write(PMW3610_REG_PERFORMANCE, 0x11);
    __SPI_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    delayMicroseconds(1);
    __SPI_write(PMW3610_REG_SMART_EN, 0x10);
    delayMicroseconds(1);
    __SPI_write(PMW3610_REG_FRAME_GRAB, 0x80);
    delay(10);

    // Step 3: Read frame data
    pinMode(_mosiMisoPin, OUTPUT);  // Set MOSI/MISO as output to send address
    _SPI__select();

    // Send frame read address
    uint8_t address = PMW3610_REG_MOTION_BURST & 0x7F;  // Ensure MSB of the address is 0 for read operation
    // Send address, sensor reads on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        digitalWrite(_sckPin, LOW);           // Clock low
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay

        // Shift-out address
        digitalWrite(_mosiMisoPin, (address & 0x80) ? HIGH : LOW);  // Write MSB first
        address <<= 1;

        digitalWrite(_sckPin, HIGH);          // Clock high
        delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay
    }

    pinMode(_mosiMisoPin, INPUT);  // Set MOSI/MISO as input to read data

    delayMicroseconds(PMW3610_HANDOVER_TIME_US);  // Handover delay

    // Read frame data
    for (size_t i = 0; i < 484; i++) {
        // Read data, sensor sends on high to low CLK transition, read on low to high CLK transition
        for (int j = 0; j < 8; j++) {
            frame_data[i] <<= 1;
            digitalWrite(_sckPin, LOW);           // Clock low
            delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay

            if (digitalRead(_mosiMisoPin)) {  // Shift-in read data
                frame_data[i] |= 0x01;
            }

            digitalWrite(_sckPin, HIGH);          // Clock high
            delayMicroseconds(_spiSpeedDelayUs);  // Clock half-cycle delay
        }
    }

    _SPI__deselect();
    delayMicroseconds(PMW3610_HOLD_TIME_US);  // Hold time after a read operation
    pinMode(_mosiMisoPin, OUTPUT);            // Set MOSI/MISO back to output

    delayMicroseconds(PMW3610_INTERREAD_TIME_US);  // Inter-command read minimum delay

    // Step 4: Reset sensor
    _SPI_reset();
    delay(PMW3610_WAKEUP_TIME_MS);

    // Step 5: Reconfgure sensor
    _configure();

    // Step 6: Resume task
    vTaskResume(_intTaskHandle);
}

void PMW3610Driver::print_frame_as_pgm() {
    Serial.println("P2");
    Serial.println("22 22");  // Frame is 22x22 pixels
    Serial.println("255");    // Max grayscale value

    for (size_t j = 0; j < 22; j++) {
        for (size_t k = 0; k < 22; k++) {
            Serial.print(frame_data[j * 22 + k], DEC);
            Serial.print(" ");
        }
        Serial.println();
    }

    Serial.println();
}
#endif