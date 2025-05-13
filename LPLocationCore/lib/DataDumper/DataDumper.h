#pragma once

#include <Arduino.h>

#include <PMW3610_driver.h>
#include <ICM20948_driver.h>

#include <IIR1stOrderVecF.h>

#define KAL_TASK_CORE 1
#define KAL_TASK_PRIORITY 1
#define KAL_TASK_STACK_SIZE 4096

#define PREDICT_TASK_FREQUENCY_HZ 100
#define UPDATE_TASK_FREQUENCY_HZ 10

#define ACCEL_ALPHA 0.5 //0.05
#define OF_ALPHA 0.02
#define ORION_ALPHA 0.5 //0.02

class DataDumper {
    public:

        DataDumper();
        ~DataDumper();

        void begin();
        void serial_recv_print_millis();

        bool started = false;

    private:

        // FreeRTOS primitives
        // SemaphoreHandle_t _mutex;
        TaskHandle_t      _predict_task_h;
        TaskHandle_t      _update_task_h;

        // Sensor data
        PMW3610Driver                      _pmw;
        ICM20948Driver                     _imu;
        IIR1stOrderVecF                    _accel_filter;
        IIR1stOrderVecF                    _of_filter;
        IIR1stOrderVecF                    _orion_filter;
        float __attribute__((aligned(16))) _accel[3];
        float __attribute__((aligned(16))) _quat[4];
        float __attribute__((aligned(16))) _of[2];

        // Task functions
        static void _predict_task(void* pvParams);
        static void _update_task(void* pvParams);

        // Utils
        void _rotateVectorByQuat(const float* quat, const float* vec, float* result);
};
