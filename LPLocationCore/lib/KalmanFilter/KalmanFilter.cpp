#include "KalmanFilter.h"

#include "config.h"

KalmanFilter::KalmanFilter() : 
    _pmw(), _imu(),
    _accel_filter(ACCEL_ALPHA, 3),
    _of_filter(OF_ALPHA, 2),
    _orion_filter(ORION_ALPHA, 4),
    _imu_var(),
    _led(LED_STRIP_WS2812, 0) {

    _mutex = xSemaphoreCreateMutex();
    _predict_task_h = NULL;
    _update_task_h = NULL;
}

KalmanFilter::~KalmanFilter() {
    vSemaphoreDelete(_mutex);
    vTaskDelete(_predict_task_h);
    vTaskDelete(_update_task_h);
}

void KalmanFilter::begin() {
    bool suc = true;

    // LED
    pinMode(16, OUTPUT);
    _led.begin(16, 1);
    _led.brightness(10);
    _led.setPixel(0, L_BLUE, 1);

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
        for (int i =0; i < 20; i++) {  // Stop the program if sensor initialization fails
            _led.brightness(0, 1);
            delay(100);
            _led.brightness(10, 1);
            delay(100);
        }
        ESP.restart(); // reset
    }

    // Start IMU sensor
    suc = _imu.begin(ICM20948_SDA_PIN, ICM20948_SCL_PIN, true);
    if (suc) {
        #ifdef DEBUG
        Serial.println("ICM20948 sensor initialised successfully!");
        #endif
    } else {
        Serial.println("ICM20948 sensor initialization failed!");
        for (int i =0; i < 20; i++) {  // Stop the program if sensor initialization fails
            _led.brightness(0, 1);
            delay(100);
            _led.brightness(10, 1);
            delay(100);
        }
        ESP.restart(); // reset
    }

    delay(2000);

    // Initialize the Kalman filter
    _initialise();

    // wait for IMU to stabalise
    _led.setPixel(0, L_CYAN, 1);
    for (int i = 0; i < 2; i++) {
        _led.brightness(0, 1);
        delay(100);
        _led.brightness(10, 1);
        delay(900);
    }

    // Start the predict task
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(_predict_task, "predict_task", KAL_TASK_STACK_SIZE, this, KAL_TASK_PRIORITY, &_predict_task_h, KAL_TASK_CORE);
    if (xReturned != pdPASS) {
        Serial.println("Kalman filter predict task creation failed!");
        for (int i =0; i < 20; i++) {  // Stop the program if sensor initialization fails
            _led.brightness(0, 1);
            delay(100);
            _led.brightness(10, 1);
            delay(100);
        }
        ESP.restart(); // reset
    }

    // Start the update task
    xReturned = xTaskCreatePinnedToCore(_update_task, "update_task", KAL_TASK_STACK_SIZE, this, KAL_TASK_PRIORITY, &_update_task_h, KAL_TASK_CORE);
    if (xReturned != pdPASS) {
        Serial.println("Kalman filter update task creation failed!");
        for (int i =0; i < 20; i++) {  // Stop the program if sensor initialization fails
            _led.brightness(0, 1);
            delay(100);
            _led.brightness(10, 1);
            delay(100);
        }
        ESP.restart(); // reset
    }
}

void KalmanFilter::_printMat(const char* name, const dspm::Mat& mat, size_t precision) {
    Serial.printf(" \"%s\": [", name);
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            Serial.printf("%.*f", precision, mat(i, j));
            if (!( j == mat.cols-1 && i == mat.rows-1 )) {
                Serial.print(", ");
            } else {
                Serial.print(" ");
            }
        }
    }
    Serial.printf("] ");
}

void KalmanFilter::_rotateVectorByQuat(const float* quat, const float* vec, float* result) {
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

float KalmanFilter::_getVecMagnitude(const float* vec) {
    return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

void KalmanFilter::_initialise() {
    // Initialise the state and covariance
    _kf_state.x = dspm::Mat::ones(6,1) * KF_INIT_X;
    _kf_state.P = dspm::Mat::eye(6) * KF_INIT_P;
    _kf_state.Q = dspm::Mat::eye(3) * KF_INIT_Q;
    _kf_state.R = dspm::Mat::eye(3) * KF_INIT_R;
    #ifdef DEBUG
    _printMat("Initial state x", _kf_state.x);
    _printMat("Initial covariance P", _kf_state.P);
    _printMat("Process noise covariance Q", _kf_state.Q);
    _printMat("Measurement noise covariance R", _kf_state.R);
    #endif
    _kf_state.K = dspm::Mat::ones(3,6) * 0.0f;

    // Calculate the expanded process noise covariance Qd
    float dt = 1.0f / PREDICT_TASK_FREQUENCY_HZ;
    dspm::Mat G(6,3);
    G(3,0) = G(4,1) = G(5,2) = dt;
    G(0,0) = G(1,1) = G(2,2) = 0.5 * dt * dt;
    _kf_state.Qd = G * _kf_state.Q * G.t();
    #ifdef DEBUG
    _printMat("State transition noise matrix G", G);
    _printMat("Expanded process noise covariance Qd", _kf_state.Qd, 6);
    #endif

    // Initialise Jacobian matrices
    _kf_state.A(0, 3) = _kf_state.A(1, 4) = _kf_state.A(2, 5) = 1.0;
    _kf_state.C(0, 0) = _kf_state.C(1, 1) = _kf_state.C(2, 2) = 1.0;
    #ifdef DEBUG
    _printMat("f Jacobian A", _kf_state.A, 0);
    _printMat("h Jacobian C", _kf_state.C, 0);
    #endif

    #ifdef DEBUG
    Serial.println("Kalman filter initialised!");
    #endif
}

void KalmanFilter::_predict_task(void* pvParameters) {
    KalmanFilter* kf = (KalmanFilter*) pvParameters;
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
        kf->_predict(dt);
    }
}

void KalmanFilter::_update_task(void* pvParameters) {
    KalmanFilter* kf = (KalmanFilter*) pvParameters;
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
        kf->_update(dt);
    }
}

void KalmanFilter::_predict(float dt) {
    // Serial.printf("Predicting with dt = %fs\n", dt);

    // Print _quat and _accel as JSON
    // Serial.print("{\"quat\": [");
    // for (int i = 0; i < 4; ++i) {
    //     Serial.print(_quat[i]);
    //     if (i < 3) Serial.print(", ");
    // }
    // Serial.print("], \"accel\": [");
    // for (int i = 0; i < 3; ++i) {
    //     Serial.print(_accel[i]);
    //     if (i < 2) Serial.print(", ");
    // }
    // Serial.println("]}");

    // 1. Rotate the acceleration vector to the global frame
    float u[3];
    _rotateVectorByQuat(_quat, _accel, u);

    //? ZUPT implementation, zero if not moving
    // float var = _imu_var.update(_getVecMagnitude(u));
    float var = _getVecMagnitude(u) + (_getVecMagnitude(_imu.data.gyro)/10);
    if (var < IMU_VAR_THR) {
        if (moving) {
            moving = false;
            _led.setPixel(0, L_GREEN, 1);
        }
    } else {
        if (!moving) {
            _led.setPixel(0, L_RED, 1);
            moving = true;
        }
    }

    if (!moving) {
        u[0] = u[1] = u[2] = 0;
        _kf_state.x(3,0) = _kf_state.x(4,0) = _kf_state.x(5,0) = 0;
    }

    // 2. Update the current state prediction
    // 2.1 Update f
    _kf_state.f(0,0) = _kf_state.x(3,0) + 0.5*u[0]*dt;
    _kf_state.f(1,0) = _kf_state.x(4,0) + 0.5*u[1]*dt;
    _kf_state.f(2,0) = _kf_state.x(5,0) + 0.5*u[2]*dt;
    _kf_state.f(3,0) = u[0];
    _kf_state.f(4,0) = u[1];
    _kf_state.f(5,0) = u[2];
    // 2.2 Update x
    xSemaphoreTake(_mutex, portMAX_DELAY);

    _kf_state.x = _kf_state.x + dt*_kf_state.f;

    // 3. Update the error covariance matrix
    // 3.1 Update A
    // 3.2 Update P
    _kf_state.P = _kf_state.P + dt*(_kf_state.A*_kf_state.P + _kf_state.P*_kf_state.A.t() + _kf_state.Qd);
    

    // Print the updated state and covariance
    // _printMat("Updated state x", _kf_state.x);
    // Print the updated state as JSON
    
    const size_t pres = 7;
    Serial.print("{\"state\": {\"x\": ");
    Serial.print(_kf_state.x(0, 0), pres);
    Serial.print(", \"y\": ");
    Serial.print(_kf_state.x(1, 0), pres);
    Serial.print(", \"z\": ");
    Serial.print(_kf_state.x(2, 0), pres);
    Serial.print(", \"vx\": ");
    Serial.print(_kf_state.x(3, 0), pres);
    Serial.print(", \"vy\": ");
    Serial.print(_kf_state.x(4, 0), pres);
    Serial.print(", \"vz\": ");
    Serial.print(_kf_state.x(5, 0), pres);
    Serial.print(", \"dt\": ");
    Serial.print(dt, pres);
    Serial.print("}, ");
    Serial.print("\"sensor_input\": {");
    Serial.print("\"quat\": {\"x\": ");
    Serial.print(_quat[1], pres);
    Serial.print(", \"y\": ");
    Serial.print(_quat[2], pres);
    Serial.print(", \"z\": ");
    Serial.print(_quat[3], pres);
    Serial.print(", \"w\": ");
    Serial.print(_quat[0], pres);
    Serial.print("}, \"accel\": {\"x\": ");
    Serial.print(_accel[0], pres);
    Serial.print(", \"y\": ");
    Serial.print(_accel[1], pres);
    Serial.print(", \"z\": ");
    Serial.print(_accel[2], pres);
    // Serial.print(", \"var\": ");
    // Serial.print(var, pres);
    //? dt times?
    Serial.printf("}},");
    _printMat("f", _kf_state.f, pres);
    Serial.print(",");
    _printMat("P", _kf_state.P, pres);
    Serial.printf(", \"micros\": %lu ", micros());
    Serial.print("}\n");

    xSemaphoreGive(_mutex);


    // LED overrun indicator
    // red if cycle is more than 10% off the target frequency
    // if (dt*0.9f > 1.0f/PREDICT_TASK_FREQUENCY_HZ) {
    //     _led.setPixel(0, L_YELLOW, 1);
    // }
}

void KalmanFilter::_update(float dt) {
    // Serial.printf("Updating with dt = %fs\n", dt);

    // Serial.print("{\"dx\": ");
    // Serial.print(_of[0], 7);
    // Serial.print(", \"dy\": ");
    // Serial.print(_of[1], 7);
    // Serial.println("}, ");

    // 1. Project OF delta to global 3D frame
    float y[3] = {_of[0], _of[1], 0};
    _rotateVectorByQuat(_quat, y, y);

    dspm::Mat y_mat(3, 1);
    y_mat(0, 0) = y[0];
    y_mat(1, 0) = y[1];
    y_mat(2, 0) = y[2];

    // Print y as JSON
    /*
    Serial.print("{\"y_\": {\"x\": ");
    Serial.print(y[0], 7);
    Serial.print(", \"y\": ");
    Serial.print(y[1], 7);
    Serial.print(", \"z\": ");
    Serial.print(y[2], 7);
    Serial.println("}}, ");
    */

    xSemaphoreTake(_mutex, portMAX_DELAY);
    
    // 2. Compute the Kalman gain
    _kf_state.K = _kf_state.P * _kf_state.C.t() * (_kf_state.C * _kf_state.P * _kf_state.C.t() + _kf_state.R).pinv();
    /*
    // Print the Kalman gain matrix K as JSON
    Serial.print("{\"K\": [");
    for (int i = 0; i < _kf_state.K.rows; ++i) {
        Serial.print("[");
        for (int j = 0; j < _kf_state.K.cols; ++j) {
            Serial.print(_kf_state.K(i, j), 7);
            if (j < _kf_state.K.cols - 1) Serial.print(", ");
        }
        Serial.print("]");
        if (i < _kf_state.K.rows - 1) Serial.print(", ");
    }
    Serial.println("]}, ");
    */

    // TODO: implement this with static variable to hold the previous state
    // 3. Update the current state estimate
    // 3.1 Calculate h
    static float prev_x[3] = {0.0f, 0.0f, 0.0f};
    _kf_state.h(0,0) = _kf_state.x(0,0) - prev_x[0];
    _kf_state.h(1,0) = _kf_state.x(1,0) - prev_x[1];
    _kf_state.h(2,0) = _kf_state.x(2,0) - prev_x[2];

    // 3.2 Update x
    dspm::Mat inno = y_mat - _kf_state.h;
    _kf_state.x = _kf_state.x + _kf_state.K * (inno);

    // 4. Update the error covariance matrix
    _kf_state.P = (dspm::Mat::eye(6) - _kf_state.K * _kf_state.C) * _kf_state.P;

    // 5. Update last x state
    prev_x[0] = _kf_state.x(0,0);
    prev_x[1] = _kf_state.x(1,0);
    prev_x[2] = _kf_state.x(2,0);

    // Print data as JSON
    const size_t pres = 7;
    Serial.print("{\"state\": {\"x\": ");
    Serial.print(_kf_state.x(0, 0), pres);
    Serial.print(", \"y\": ");
    Serial.print(_kf_state.x(1, 0), pres);
    Serial.print(", \"z\": ");
    Serial.print(_kf_state.x(2, 0), pres);
    Serial.print(", \"vx\": ");
    Serial.print(_kf_state.x(3, 0), pres);
    Serial.print(", \"vy\": ");
    Serial.print(_kf_state.x(4, 0), pres);
    Serial.print(", \"vz\": ");
    Serial.print(_kf_state.x(5, 0), pres);
    Serial.print(", \"dt\": ");
    Serial.print(dt, pres);
    Serial.print("}, ");
    Serial.print("\"sensor_input\": {");
    Serial.print("\"quat\": {\"x\": ");
    Serial.print(_quat[1], pres);
    Serial.print(", \"y\": ");
    Serial.print(_quat[2], pres);
    Serial.print(", \"z\": ");
    Serial.print(_quat[3], pres);
    Serial.print(", \"w\": ");
    Serial.print(_quat[0], pres);
    Serial.print("}, \"of\": {\"x\": ");
    Serial.print(y[0], pres);
    Serial.print(", \"y\": ");
    Serial.print(y[1], pres);
    Serial.print(", \"z\": ");
    Serial.print(y[2], pres);
    //? dt times?
    Serial.printf("}},");
    _printMat("y-h", inno, pres);
    Serial.print(",");
    _printMat("K", _kf_state.K, pres);
    Serial.print(",");
    _printMat("P", _kf_state.P, pres);
    Serial.printf(", \"micros\": %lu ", micros());
    Serial.print("}\n");
    
    xSemaphoreGive(_mutex);

    // LED overrun indicator
    // red if cycle is more than 10% off the target frequency
    // if (dt*0.9f > 1.0f/UPDATE_TASK_FREQUENCY_HZ) {
    //     _led.setPixel(0, L_YELLOW, 1);
    // }
}