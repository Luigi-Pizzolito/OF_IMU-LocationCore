#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "ICM20948.h"

// ICM20948 driver class

/* Class hard-configurations */
#define ICM20948_MOTION_DATA_UPDATE_RATE_MS  5
#define ICM20948_TASK_PRIORITY               1
#define ICM20948_TASK_CORE                   1
#define ICM20948_TASK_STACK_SIZE             2048

#define DEBUG

// Struct to hold sensor data
struct ICM20948Data {
    xyzFloat acel;      // g (x,y,z)
    xyzFloat gyrl;      // deg/s (x,y,z)
    xyzFloat magt;      // uT (x,y,z)
    xyzFloat angle;     // +-pi rad (x,y,z)
    float pitch;        // +-pi rad
    float roll;         // +-pi rad
    float temp;         // C
};

// ICM20948 driver class
class ICM20948Driver {
    public:
    /* ICM20948 driver implementation */
    ICM20948Data data;
    bool begin(int sda, int scl);
    void printData();

    private:
    /* I2C driver lib */
    ICM20948_WE _IMU = ICM20948_WE(ICM20948_ADDR);

    /* ICM20948 driver functions */
    bool _check_product_id();
    void _calibrate();
    void _configure();

    void _read_data();

    TaskHandle_t _refreshTaskHandle = NULL;
    static void _refreshTask(void *pvParameters);
};
