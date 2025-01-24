#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "ICM20948.h"

// ICM20948 driver class

/* Class hard-configurations */
#define ICM20948_MOTION_DATA_UPDATE_RATE_MS  5
#define ICM20948_TASK_PRIORITY               1
#define ICM20948_TASK_CORE                   1
#define ICM20948_TASK_STACK_SIZE             4096

#define DEBUG

// Struct to hold sensor data
struct ICM20948Data {
    /*
    xyzFloat acel;      // g (x,y,z)
    xyzFloat gyrl;      // deg/s (x,y,z)
    xyzFloat magt;      // uT (x,y,z)
    xyzFloat angle;     // +-pi rad (x,y,z)
    float pitch;        // +-pi rad
    float roll;         // +-pi rad
    float temp;         // C
    **/
   float gyro[3];
   float acel[3];
   float magt[3];

   float l_acel[3];
   float quat9[4];
   float quat9_e[3];
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
    ArduinoICM20948 _imu;
    ArduinoICM20948Settings _icmSettings = {
        .i2c_speed = ICM20948_SPEED,
        .is_SPI = false,
        .cs_pin = -1,
        .spi_speed = 7000000,
        .mode = 1,

        .enable_gyroscope = true,
        .enable_accelerometer = true,
        .enable_magnetometer = true,

        .enable_gravity = false,
        .enable_linearAcceleration = true,
        .enable_quaternion6 = false,
        .enable_quaternion9 = true,

        .enable_har = false,
        .enable_steps = false,

        .gyroscope_frequency = 200,
        .accelerometer_frequency = 200,
        .magnetometer_frequency = 70,

        .gravity_frequency = 1,
        .linearAcceleration_frequency = 200,
        .quaternion6_frequency = 50,
        .quaternion9_frequency = 200,

        .har_frequency = 50,
        .steps_frequency = 50
    };

    /* ICM20948 driver functions */
    void _calibrate();

    void _read_data();

    TaskHandle_t _refreshTaskHandle = NULL;
    static void _refreshTask(void *pvParameters);
};
