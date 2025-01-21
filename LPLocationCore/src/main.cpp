#include <Arduino.h>

#include "config.h"

#include <PMW3610_driver.h>

PMW3610Driver pmw;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  // Serial.println("Hello World!");
  // pinMode(38, INPUT);
  pmw.linkInterrupt([]()
                    {
    Serial.println("Motion detected!");
    pmw.printData(); });

  Serial.println("Starting pmw communication...");
  if (
      pmw.begin(PMW3610_SCK_PIN, PMW3610_DIO_PIN, PMW3610_CS_PIN, PMW3610_RST_PIN, PMW3610_IRQ_PIN) // Initialize the sensor
  )
  {
    Serial.println("PMW3610 sensor initialized successfully!");
  }
  else
  {
    Serial.println("PMW3610 sensor initialization failed!");
    while (1)
      ; // Stop the program if sensor initialization fails
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Serial.println(digitalRead(38));
  if (pmw.data.motion)
  {
    pmw.printData();
  }
  delay(5);
}