#include "ICM20948_driver.h"

/* ICM20948 driver implementation */
bool ICM20948Driver::begin(int sda, int scl) {
    // Step 1: Initialise I2C
    Wire.begin(sda, scl);
    // Step 2: Init communication
    if (!_IMU.init()) {
        return false;
    }
    if (!_IMU.initMagnetometer()) {
        return false;
    }
    #ifdef DEBUG
    #endif
    // Step 3: Check ID
    if (!_check_product_id()) {
        return false;
    }
    #ifdef DEBUG
    #endif
    // Step 4: Calibrate
    _calibrate();
    // Step 5: Configure
    _configure();
    // Step 6: Set up task
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(_refreshTask, "ICM20948RefreshData", ICM20948_TASK_STACK_SIZE, this, ICM20948_TASK_PRIORITY, &_refreshTaskHandle, ICM20948_TASK_CORE);
    if (xReturned != pdPASS) {
        Serial.println("ICM20948 update task creation failed!");
        return false;
    }

    #ifdef DEBUG
    #endif
}

void ICM20948Driver::printData() {
    //todo:
}

/* ICM20948 driver functions */
bool ICM20948Driver::_check_product_id() {
    uint8_t id = _IMU.whoAmI();
    if (id != 0xEA) {
        return false;
    }

    return true;
}

void ICM20948Driver::_calibrate() {
    #ifdef DEBUG
    #endif
    _IMU.autoOffsets();
}

void ICM20948Driver::_configure() {
    // Accelerometer
    _IMU.enableAcc(true);
    _IMU.setAccRange(ICM20948_ACC_RANGE);
    _IMU.setAccDLPF(ICM20948_ACC_DLPF);
    //? set sample rate divider?
    // Gyroscope
    _IMU.enableGyr(true);
    _IMU.setGyrRange(ICM20948_GYR_RANGE);
    _IMU.setGyrDLPF(ICM20948_GYR_DLPF);
    //? set sample rate divider?
    // Temperature
    _IMU.setTempDLPF(ICM20948_TEMP_DLPF);
    // Magnetometer
    _IMU.setMagOpMode(ICM20948_MAG_MODE);
    switch (ICM20948_MAG_MODE) {
        case AK09916_CONT_MODE_100HZ:
            delay(1000/100);
            break;
        case AK09916_CONT_MODE_50HZ:
            delay(1000/50);
            break;
        case AK09916_CONT_MODE_20HZ:
            delay(1000/20);
            break;
        case AK09916_CONT_MODE_10HZ:
            delay(1000/10);
            break;

        default:
            break;
    }
}

void ICM20948Driver::_read_data() {
    _IMU.readSensor();
    _IMU.getGValues(&data.acel);
    _IMU.getGyrValues(&data.gyrl);
    _IMU.getMagValues(&data.magt);
    _IMU.getAngles(&data.angle);
    data.pitch = _IMU.getPitch();
    data.roll = _IMU.getRoll();
    data.temp = _IMU.getTemperature();
}

void ICM20948Driver::_refreshTask(void *pvParameters) {
    ICM20948Driver *driver = (ICM20948Driver *)pvParameters;
    #ifdef DEBUG
    #endif
    while (true) {
        // Read motion data every Xms, always
        driver->_read_data();
        // Delay
        vTaskDelay(pdMS_TO_TICKS(ICM20948_MOTION_DATA_UPDATE_RATE_MS));
    }
}