// FILE: test/test_stepper/test_main.cpp
#include <Arduino.h>
#include "FastAccelStepper.h"

// #define STEPPER_SPEED 800 //25000 // uS/step
#define STEPPER_ACCELERATION 100  // 5000

#define STEPPER_ENABLE 48
#define STEPPER_ENABLE_ACTIVE_LOW false

#define STEPPER_L1_STEP 47
#define STEPPER_L1_DIR 21
#define STEPPER_L2_STEP 12
#define STEPPER_L2_DIR 11
#define STEPPER_R1_STEP 10
#define STEPPER_R1_DIR 9
#define STEPPER_R2_STEP 14
#define STEPPER_R2_DIR 13

FastAccelStepperEngine engine = FastAccelStepperEngine();
// FastAccelStepper *stepperL1;
FastAccelStepper *stepperL2;
// FastAccelStepper *stepperR1;
FastAccelStepper *stepperR2;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
    }
    delay(2000);

    engine.init();

    // stepperL1 = engine.stepperConnectToPin(STEPPER_L1_STEP);
    stepperL2 = engine.stepperConnectToPin(STEPPER_L2_STEP);
    // stepperR1 = engine.stepperConnectToPin(STEPPER_R1_STEP);
    stepperR2 = engine.stepperConnectToPin(STEPPER_R2_STEP);

    // if (stepperL1 && stepperL2 && stepperR1 && stepperR2) {
    // stepperL1->setDirectionPin(STEPPER_L1_DIR);
    // stepperL1->setEnablePin(STEPPER_ENABLE, STEPPER_ENABLE_ACTIVE_LOW);
    // stepperL1->setAutoEnable(true);

    stepperL2->setDirectionPin(STEPPER_L2_DIR);
    stepperL2->setEnablePin(STEPPER_ENABLE, STEPPER_ENABLE_ACTIVE_LOW);
    stepperL2->setAutoEnable(true);

    // stepperR1->setDirectionPin(STEPPER_R1_DIR);
    // stepperR1->setEnablePin(STEPPER_ENABLE, STEPPER_ENABLE_ACTIVE_LOW);
    // stepperR1->setAutoEnable(true);

    stepperR2->setDirectionPin(STEPPER_R2_DIR);
    stepperR2->setEnablePin(STEPPER_ENABLE, STEPPER_ENABLE_ACTIVE_LOW);
    stepperR2->setAutoEnable(true);

    // stepperL1->setSpeedInUs(STEPPER_SPEED);
    // stepperL1->setAcceleration(STEPPER_ACCELERATION);

    // stepperL2->setSpeedInUs(STEPPER_SPEED);
    stepperL2->setSpeedInHz(100);
    stepperL2->setAcceleration(STEPPER_ACCELERATION);

    // stepperR1->setSpeedInUs(STEPPER_SPEED);
    // stepperR1->setAcceleration(STEPPER_ACCELERATION);

    // stepperR2->setSpeedInUs(STEPPER_SPEED);
    stepperR2->setSpeedInHz(100);
    stepperR2->setAcceleration(STEPPER_ACCELERATION);

    Serial.println("Steppers initialized");
    // }
}

void loop() {
    // if (stepperL1 && stepperL2 && stepperR1 && stepperR2) {
    // stepperL1->runForward();
    stepperL2->runForward();
    // stepperR1->runForward();
    stepperR2->runForward();

    delay(5000);

    // stepperL1->stopMove();
    stepperL2->stopMove();
    // stepperR1->stopMove();
    stepperR2->stopMove();
    delay(500);

    // stepperL1->runBackward();
    stepperL2->runBackward();
    // stepperR1->runBackward();
    stepperR2->runBackward();

    delay(5000);

    // stepperL1->stopMove();
    stepperL2->stopMove();
    // stepperR1->stopMove();
    stepperR2->stopMove();
    delay(500);

    // stepperL1->move(1);
    // stepperL2->move(1);
    // stepperR1->move(1);
    // stepperR2->move(1);
    // delay(25);

    // stepperL1->move(1000);
    // stepperL2->move(1000);
    // stepperR1->move(1000);
    // stepperR2->move(1000);
    // delay(5000);

    // delay(500);

    // stepperL1->move(-1000);
    // stepperL2->move(-1000);
    // stepperR1->move(-1000);
    // stepperR2->move(-1000);
    // delay(5000);
    // }
}