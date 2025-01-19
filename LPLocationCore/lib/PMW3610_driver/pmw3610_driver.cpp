#include "pmw3610_driver.h"

/* Bit-banged 3-wire SPI functions */
// CPOL and CPHA are 1, so clock is normally high and data is sampled on the rising edge
// Default working frequency is 500kHz

void PMW3610Driver::_SPI_begin(int sckPin, int mosiMisoPin, int csPin, int resetPin, uint32_t spiSpeedDelayUs) {
    // Save configuration
    _sckPin = sckPin;
    _mosiMisoPin = mosiMisoPin;
    _csPin = csPin;
    _resetPin = resetPin;
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

    delay(PMW3610_WAKEUP_TIME_MS*5); // Wait for power to stabilize (50ms on initial power-up)
}

void PMW3610Driver::_SPI_reset() {
    // Reset the sensor
    digitalWrite(_resetPin, LOW); // Activate reset
    delayMicroseconds(PMW3610_RESET_HOLD_TIME_US); // Hold reset
    digitalWrite(_resetPin, HIGH); // Deactivate reset
    delay(PMW3610_WAKEUP_TIME_MS); // Wait for reset to complete
}

void PMW3610Driver::_SPI__select() {
    digitalWrite(_csPin, LOW); // Select
    delayMicroseconds(PMW3610_NCS_SETUP_TIME_US);
}

void PMW3610Driver::_SPI__deselect() {
    delayMicroseconds(PMW3610_NCS_SETUP_TIME_US);
    digitalWrite(_csPin, HIGH); // Deselect
    digitalWrite(_sckPin, HIGH); // Clock high idle
}

uint8_t PMW3610Driver::_SPI_read(uint8_t address) {
    uint8_t data = 0;
    address &= 0x7F; // Ensure MSB of the address is 0 for read operation
    pinMode(_mosiMisoPin, OUTPUT); // Set MOSI/MISO as output to send address
    _SPI__select();

    // Send address, sensor reads on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        digitalWrite(_sckPin, LOW);                 // Clock low
        delayMicroseconds(_spiSpeedDelayUs);        // Clock half-cycle delay

                                                    // Shift-out address
        digitalWrite(_mosiMisoPin, (address & 0x80) ? HIGH : LOW); // Write MSB first
        address <<= 1;

        digitalWrite(_sckPin, HIGH);                // Clock high
        delayMicroseconds(_spiSpeedDelayUs);        // Clock half-cycle delay
    }

    pinMode(_mosiMisoPin, INPUT); // Set MOSI/MISO as input to read data
    
    delayMicroseconds(PMW3610_HANDOVER_TIME_US); // Handover delay

    // Read data, sensor sends on high to low CLK transition, read on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        digitalWrite(_sckPin, LOW);                 // Clock low
        delayMicroseconds(_spiSpeedDelayUs);        // Clock half-cycle delay

        if (digitalRead(_mosiMisoPin)) {            // Shift-in read data
            data |= 0x01;
        }

        digitalWrite(_sckPin, HIGH);                // Clock high
        delayMicroseconds(_spiSpeedDelayUs);        // Clock half-cycle delay
    }

    _SPI__deselect();
    delayMicroseconds(PMW3610_HOLD_TIME_US); // Hold time after a read operation
    pinMode(_mosiMisoPin, OUTPUT); // Set MOSI/MISO back to output

    delayMicroseconds(PMW3610_INTERREAD_TIME_US); // Inter-command read minimum delay

    return data;
}

void PMW3610Driver::_SPI_write(uint8_t address, uint8_t data) {
    address |= 0x80; // Ensure MSB of the address is 1 for write operation
    pinMode(_mosiMisoPin, OUTPUT); // Set MOSI/MISO as output to send address and data
    _SPI__select();

    // Send address, sensor reads on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        digitalWrite(_sckPin, LOW);                 // Clock low
        delayMicroseconds(_spiSpeedDelayUs);        // Clock half-cycle delay

                                                    // Shift-out address
        digitalWrite(_mosiMisoPin, (address & 0x80) ? HIGH : LOW); // Write MSB first
        address <<= 1;

        digitalWrite(_sckPin, HIGH);                // Clock high
        delayMicroseconds(_spiSpeedDelayUs);        // Clock half-cycle delay
    }

    // Send data, sensor reads on low to high CLK transition
    for (int i = 0; i < 8; i++) {
        digitalWrite(_sckPin, LOW);                 // Clock low
        delayMicroseconds(_spiSpeedDelayUs);        // Clock half-cycle delay
        
                                                    // Shift-out write data
        digitalWrite(_mosiMisoPin, (data & 0x80) ? HIGH : LOW); // Write MSB first
        data <<= 1;

        digitalWrite(_sckPin, HIGH);                // Clock high
        delayMicroseconds(_spiSpeedDelayUs);        // Clock half-cycle delay
    }

    delayMicroseconds(PMW3610_HOLD_TIME_US*10); // Hold time after a write operation
    _SPI__deselect();

    delayMicroseconds(PMW3610_INTERWRITE_TIME_US); // Inter-command write minimum delay
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
    if ( (observation & 0x0F) != 0x0F ) {
        Serial.println("PMW3610 self-test failed!");
        Serial.print("Observation register: ");
        Serial.println(observation, HEX);
        return false;
    }

    return true;
}

/* PMW3610 Driver public functions */

bool PMW3610Driver::begin(int sckPin, int mosiMisoPin, int csPin, int irqPin, int resetPin) {
    _irqPin = irqPin;
    pinMode(_irqPin, INPUT);
    
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

    // Step 2: Perform self-test
    if (!_self_test()) {
        return false;
    }

    // Step 4: Configure sensor

    return true;
}

void PMW3610Driver::read_test() {
    uint8_t reg1 = _SPI_read(0x01);
    uint8_t reg0 = _SPI_read(0x00);
    uint8_t reg2 = _SPI_read(0x02);
    uint8_t reg3 = _SPI_read(0x03);
    uint8_t reg4 = _SPI_read(0x04);
    uint8_t reg5 = _SPI_read(0x05);

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
}