#include "ICM20948_driver.h"

/* ICM20948 driver implementation */
bool ICM20948Driver::begin(int sda, int scl) {
    // Step 1: Initialise I2C
    Wire.begin(sda, scl);
    Wire.setClock(ICM20948_SPEED);
    delay(10);

    // Step 2: Init communication, calibrate and set up sensor
    _imu.init(_icmSettings);
    #ifdef DEBUG
        Serial.println("ICM20948 sensor initialised!");
    #endif

    // Step 3: Set up task
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(_refreshTask, "ICM20948RefreshData", ICM20948_TASK_STACK_SIZE, this, ICM20948_TASK_PRIORITY, &_refreshTaskHandle, ICM20948_TASK_CORE);
    if (xReturned != pdPASS) {
        Serial.println("ICM20948 update task creation failed!");
        return false;
    }

    #ifdef DEBUG
        Serial.println("ICM20948 sensor configured successfully!");
    #endif
    
    return true;
}

void ICM20948Driver::printData() {
    String output = "{";
    output += "\"gyro\":{\"x\":" + String(data.gyro[0]) + ",\"y\":" + String(data.gyro[1]) + ",\"z\":" + String(data.gyro[2]) + "},";
    output += "\"accel\":{\"x\":" + String(data.acel[0]) + ",\"y\":" + String(data.acel[1]) + ",\"z\":" + String(data.acel[2]) + "},";
    output += "\"mag\":{\"x\":" + String(data.magt[0]) + ",\"y\":" + String(data.magt[1]) + ",\"z\":" + String(data.magt[2]) + "},";
    output += "\"linear_accel\":{\"x\":" + String(data.l_acel[0]) + ",\"y\":" + String(data.l_acel[1]) + ",\"z\":" + String(data.l_acel[2]) + "},";
    output += "\"quat9\":{\"w\":" + String(data.quat9[0]) + ",\"x\":" + String(data.quat9[1]) + ",\"y\":" + String(data.quat9[2]) + ",\"z\":" + String(data.quat9[3]) + "},";
    output += "\"euler9\":{\"roll\":" + String(data.quat9_e[0]) + ",\"pitch\":" + String(data.quat9_e[1]) + ",\"yaw\":" + String(data.quat9_e[2]) + "}";
    output += "}";

    Serial.println(output);
}

/* ICM20948 driver functions */
void ICM20948Driver::_read_data() {
    _imu.task();

    if (_imu.gyroDataIsReady()) {
        _imu.readGyroData(&data.gyro[0], &data.gyro[1], &data.gyro[2]);
    }
    if (_imu.accelDataIsReady()) {
        _imu.readAccelData(&data.acel[0], &data.acel[1], &data.acel[2]);
    }
    if (_imu.magDataIsReady()) {
        _imu.readMagData(&data.magt[0], &data.magt[1], &data.magt[2]);
    }

    if (_imu.linearAccelDataIsReady()) {
        _imu.readLinearAccelData(&data.l_acel[0], &data.l_acel[1], &data.l_acel[2]);
    }
    if (_imu.quat9DataIsReady()) {
        _imu.readQuat9Data(&data.quat9[0], &data.quat9[1], &data.quat9[2], &data.quat9[3]);
    }
    if (_imu.euler9DataIsReady()) {
        _imu.readEuler9Data(&data.quat9_e[0], &data.quat9_e[1], &data.quat9_e[2]);
    }
}

void ICM20948Driver::_refreshTask(void *pvParameters) {
    ICM20948Driver *driver = (ICM20948Driver *)pvParameters;
    #ifdef DEBUG
        Serial.println("ICM20948 update task started!");
    #endif
    while (true) {
        // Read motion data every Xms, always
        driver->_read_data();
        // Delay
        vTaskDelay(pdMS_TO_TICKS(ICM20948_MOTION_DATA_UPDATE_RATE_MS));
    }
}