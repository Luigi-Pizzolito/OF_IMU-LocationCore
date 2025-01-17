#include <Arduino.h>

#define STEPPER_L1_STEP 47
#define STEPPER_L1_DIR 21
#define STEPPER_L2_STEP 12
#define STEPPER_L2_DIR 11
#define STEPPER_R1_STEP 10
#define STEPPER_R1_DIR 9
#define STEPPER_R2_STEP 14
#define STEPPER_R2_DIR 13
#define STEPPER_ENABLE 48

void setup() {
  // Initialize all pins as outputs
  pinMode(STEPPER_L1_STEP, OUTPUT);
  pinMode(STEPPER_L1_DIR, OUTPUT);
  pinMode(STEPPER_L2_STEP, OUTPUT);
  pinMode(STEPPER_L2_DIR, OUTPUT);
  pinMode(STEPPER_R1_STEP, OUTPUT);
  pinMode(STEPPER_R1_DIR, OUTPUT);
  pinMode(STEPPER_R2_STEP, OUTPUT);
  pinMode(STEPPER_R2_DIR, OUTPUT);
  pinMode(STEPPER_ENABLE, OUTPUT);

  // Set all pins low before starting
  digitalWrite(STEPPER_L1_STEP, LOW);
  digitalWrite(STEPPER_L1_DIR, LOW);
  digitalWrite(STEPPER_L2_STEP, LOW);
  digitalWrite(STEPPER_L2_DIR, LOW);
  digitalWrite(STEPPER_R1_STEP, LOW);
  digitalWrite(STEPPER_R1_DIR, LOW);
  digitalWrite(STEPPER_R2_STEP, LOW);
  digitalWrite(STEPPER_R2_DIR, LOW);
  digitalWrite(STEPPER_ENABLE, LOW);

  // Set each pin high sequentially with 5ms delay
  delay(5);
  digitalWrite(STEPPER_L1_STEP, HIGH);
  delay(5);
  digitalWrite(STEPPER_L1_DIR, HIGH);
  delay(5);
  digitalWrite(STEPPER_L2_STEP, HIGH);
  delay(5);
  digitalWrite(STEPPER_L2_DIR, HIGH);
  delay(5);
  digitalWrite(STEPPER_R1_STEP, HIGH);
  delay(5);
  digitalWrite(STEPPER_R1_DIR, HIGH);
  delay(5);
  digitalWrite(STEPPER_R2_STEP, HIGH);
  delay(5);
  digitalWrite(STEPPER_R2_DIR, HIGH);
  delay(5);
  digitalWrite(STEPPER_ENABLE, HIGH);
}

void loop() {
  // Nothing to do here
}