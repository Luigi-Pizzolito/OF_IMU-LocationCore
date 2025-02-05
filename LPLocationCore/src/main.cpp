#include <Arduino.h>

// #include "config.h"

// #include <PMW3610_driver.h>
// #include <ICM20948_driver.h>

// PMW3610Driver pmw;
// ICM20948Driver imu;

#include <KalmanFilter.h>

KalmanFilter kf;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(2000);

    /*
    bool suc = true;

    Serial.println("Starting pmw communication...");
    // Initialize the sensor
    suc = pmw.begin(PMW3610_SCK_PIN, PMW3610_DIO_PIN, PMW3610_CS_PIN,
                    PMW3610_RST_PIN, PMW3610_IRQ_PIN);
    if (suc) {
        Serial.println("PMW3610 sensor initialized successfully!");
    } else {
        Serial.println("PMW3610 sensor initialization failed!");
        while (1);  // Stop the program if sensor initialization fails
    }

    // #ifdef PMW3610_ENABLE_FRAME_CAPTURE
    //     for (int i = 0; i < 5000 / 5; i++) {
    //         if (pmw.data.motion) {
    //             pmw.printData();
    //         }
    //         delay(5);
    //     }
    //     pmw.capture_frame();
    //     delay(5);
    //     pmw.print_frame_as_pgm();
    //     delay(5);
    // #endif

    suc = imu.begin(ICM20948_SDA_PIN, ICM20948_SCL_PIN);
    if (suc) {
        Serial.println("ICM20948 sensor initialised successfully!");
    } else {
        Serial.println("ICM20948 sensor initialization failed!");
        while (1); // Stop the program if sensor initialization fails
    }
    */

   kf.begin();
}

void loop() {
    /*
    // put your main code here, to run repeatedly:
    if (pmw.data.motion) {
        pmw.printData();
    }
    imu.printData();
    delay(100);
    */
}