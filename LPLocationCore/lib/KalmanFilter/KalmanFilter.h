#pragma once

#include <Arduino.h>

#include <PMW3610_driver.h>
#include <ICM20948_driver.h>

#include "esp_dsp.h"
#include <IIR1stOrderVecF.h>
#include <Variance.h>

#define ACCEL_ALPHA 0.5 //0.05
#define OF_ALPHA 0.02
#define ORION_ALPHA 0.5 //0.02

#define IMU_VAR_THR 0.5

// #define IMU_VAR_THR 0.1

#define KAL_TASK_CORE 1
#define KAL_TASK_PRIORITY 1
#define KAL_TASK_STACK_SIZE 4096

#define PREDICT_TASK_FREQUENCY_HZ 50
#define UPDATE_TASK_FREQUENCY_HZ 10

#define KF_INIT_X 0.0f
#define KF_INIT_P 0.1f
#define KF_INIT_Q 0.001f
#define KF_INIT_R 0.01f

// #define IMU_VAR_THR 0.10f
// #define KF_INIT_P 0.2255923f
// #define KF_INIT_Q 0.0004803f
// #define KF_INIT_R 27.8762436f

// #define IMU_VAR_THR 0.05f
// #define KF_INIT_P 0.2029382f
// #define KF_INIT_Q 0.0005760f
// #define KF_INIT_R 25.0738049f

// opt slow
// #define IMU_VAR_THR 0.25f
// #define KF_INIT_P 0.2255923f
// #define KF_INIT_Q 0.0004803f
// #define KF_INIT_R 27.8762436f

// opt fast
// #define IMU_VAR_THR 0.25f
// #define KF_INIT_P 0.0415492f
// #define KF_INIT_Q 0.7152761f
// #define KF_INIT_R 2.4869432f

// opt faster
// #define IMU_VAR_THR 0.25f
// #define KF_INIT_P 0.0663339f
// #define KF_INIT_Q 0.6855177f
// #define KF_INIT_R 2.1589546f

// opt fastest
// #define IMU_VAR_THR 0.25f
// #define KF_INIT_P 0.1016924f
// #define KF_INIT_Q 0.6860596f
// #define KF_INIT_R 1.8777355f


#include <LiteLED.h>
static const crgb_t L_RED   = 0xff0000;
static const crgb_t L_YELLOW = 0xffff00;
static const crgb_t L_GREEN = 0x00ff00;
static const crgb_t L_BLUE  = 0x0000ff;
static const crgb_t L_CYAN  = 0x00ffff;


// Kalman filter class
class KalmanFilter {
   public:

    KalmanFilter();
    ~KalmanFilter();

    void begin();

    /*
    // Thread-safe state access
    void getState(float out_state[STATE_DIM]);
    void getCovariance(float out_cov[STATE_DIM * STATE_DIM]);
    */

   private:
    // FreeRTOS primitives
    SemaphoreHandle_t _mutex;
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

    Variance _imu_var;
    bool moving = false;

    // State
    struct State {
        dspm::Mat x;
        dspm::Mat P;
        dspm::Mat Q;
        dspm::Mat R;

        dspm::Mat Qd;
        dspm::Mat f;
        dspm::Mat A;
        dspm::Mat h;
        dspm::Mat C;
        dspm::Mat K;

        State() : x(6, 1), P(6,6), Q(3,3), Qd(6,6), R(3,3), f(6,1), A(6,6), h(3,1), C(3,6), K(3,6) {}
    } _kf_state;

    // Internal methods
    void _initialise();
    void _predict(float dt);
    void _update(float dt);

    // Task functions
    static void _predict_task(void* pvParams);
    static void _update_task(void* pvParams);

    // Utils
    void _printMat(const char* name, const dspm::Mat& mat, size_t precision = 3);
    void _rotateVectorByQuat(const float* quat, const float* vec, float* result);
    float _getVecMagnitude(const float* vec);

    LiteLED _led;
};