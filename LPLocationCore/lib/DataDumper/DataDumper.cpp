#include "DataDumper.h"

// Global config

/* PMW3610 pins */
#define PMW3610_CS_PIN  40
#define PMW3610_SCK_PIN 41
#define PMW3610_DIO_PIN 42
#define PMW3610_RST_PIN 39
#define PMW3610_IRQ_PIN 38

/* ICM20948 pins */
#define ICM20948_SDA_PIN 5
#define ICM20948_SCL_PIN 4

// DataDumper class

DataDumper::DataDumper() : 
    _pmw(), _imu(),
    _accel_filter(ACCEL_ALPHA, 3),
    _of_filter(OF_ALPHA, 2),
    _orion_filter(ORION_ALPHA, 4) {
    _predict_task_h = NULL;
    _update_task_h = NULL;
}

DataDumper::~DataDumper() {
    vTaskDelete(_predict_task_h);
    vTaskDelete(_update_task_h);
}

void DataDumper::begin() {
    bool suc = true;

    // Start OF sensor
    #ifdef DEBUG
    Serial.println("Starting pmw communication...");
    #endif
    // Initialize the sensor
    suc = _pmw.begin(PMW3610_SCK_PIN, PMW3610_DIO_PIN, PMW3610_CS_PIN,
                    PMW3610_RST_PIN, PMW3610_IRQ_PIN, true);
    if (suc) {
        #ifdef DEBUG
        Serial.println("PMW3610 sensor initialized successfully!");
        #endif
    } else {
        Serial.println("PMW3610 sensor initialization failed!");
        // while (1) {  // Stop the program if sensor initialization fails
        // }
        delay(1000);
        ESP.restart();
    }

    // Start IMU sensor
    suc = _imu.begin(ICM20948_SDA_PIN, ICM20948_SCL_PIN, true);
    if (suc) {
        #ifdef DEBUG
        Serial.println("ICM20948 sensor initialised successfully!");
        #endif
    } else {
        Serial.println("ICM20948 sensor initialization failed!");
        // while (1) {  // Stop the program if sensor initialization fails
        // }
        delay(1000);
        ESP.restart();
    }

    // wait for IMU to stabalise
    delay(2000);

    Serial.println("Starting Tasks");

    // Start the predict task
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(_predict_task, "predict_task", KAL_TASK_STACK_SIZE, this, KAL_TASK_PRIORITY, &_predict_task_h, KAL_TASK_CORE);
    if (xReturned != pdPASS) {
        Serial.println("Kalman filter predict task creation failed!");
        while (1);
    }

    // Start the update task
    xReturned = xTaskCreatePinnedToCore(_update_task, "update_task", KAL_TASK_STACK_SIZE, this, KAL_TASK_PRIORITY, &_update_task_h, KAL_TASK_CORE);
    if (xReturned != pdPASS) {
        Serial.println("Kalman filter update task creation failed!");
        while (1);
    }
}

void DataDumper::_rotateVectorByQuat(const float* quat, const float* vec, float* result) {
    // Quaternion components
    float q0 = quat[0];
    float q1 = quat[1];
    float q2 = quat[2];
    float q3 = quat[3];

    // Vector components
    float vx = vec[0];
    float vy = vec[1];
    float vz = vec[2];

    // Calculate the rotated vector
    result[0] = (1 - 2*q2*q2 - 2*q3*q3) * vx + (2*q1*q2 - 2*q0*q3) * vy + (2*q1*q3 + 2*q0*q2) * vz;
    result[1] = (2*q1*q2 + 2*q0*q3) * vx + (1 - 2*q1*q1 - 2*q3*q3) * vy + (2*q2*q3 - 2*q0*q1) * vz;
    result[2] = (2*q1*q3 - 2*q0*q2) * vx + (2*q2*q3 + 2*q0*q1) * vy + (1 - 2*q1*q1 - 2*q2*q2) * vz;
}

void DataDumper::_predict_task(void* pvParameters) {
    DataDumper* kf = (DataDumper*) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / PREDICT_TASK_FREQUENCY_HZ);

    unsigned long prev_time = micros();

    #ifdef DEBUG
    Serial.println("Predict task started!");
    #endif

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        xLastWakeTime = xTaskGetTickCount();
        // Fetch sensor data
        // kf->_imu.update();
        // kf->_imu.printData();
        // Filter sensor data
        kf->_orion_filter.filter(kf->_imu.data.quat9, kf->_quat);
        kf->_accel_filter.filter(kf->_imu.data.l_acel, kf->_accel);

        // Dispatch the predict function
        unsigned long curr_time = micros();
        float dt = (curr_time - prev_time) / 1000000.0f;
        prev_time = curr_time;
        // kf->_predict(dt);
        if (kf->started) {
            // Print the filtered accelerometer data
            Serial.printf("%lu,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,\r\n", 
                          millis(), 
                          kf->_accel[0], kf->_accel[1], kf->_accel[2], 
                          kf->_quat[0], kf->_quat[1], kf->_quat[2], kf->_quat[3]);
        }
    }
}

void DataDumper::_update_task(void* pvParameters) {
    DataDumper* kf = (DataDumper*) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / UPDATE_TASK_FREQUENCY_HZ);

    unsigned long prev_time = micros();

    #ifdef DEBUG
    Serial.println("Update task started!");
    #endif

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        xLastWakeTime = xTaskGetTickCount();

        // Get sensor data
        // kf->_pmw.update();
        // kf->_pmw.printData();
        // Filter sensor data
        float d_xy[2] = {kf->_pmw.data.delta_x, kf->_pmw.data.delta_y};
        kf->_of_filter.filter(d_xy, kf->_of);

        // Dispatch the update function
        unsigned long curr_time = micros();
        float dt = (curr_time - prev_time) / 1000000.0f;
        prev_time = curr_time;
        // kf->_update(dt);
        if (kf->started) {
            Serial.printf("%lu,,,,,,,,%.8f,%.8f\r\n", millis(), kf->_of[0], kf->_of[1]);
        }
    }
}

void DataDumper::serial_recv_print_millis() {
    // if (Serial.available() > 0) {
        // Serial.read();  // Read the incoming byte
        Serial.print("START FLAG: ");
        Serial.println(millis());
        started = true;
    // }
}