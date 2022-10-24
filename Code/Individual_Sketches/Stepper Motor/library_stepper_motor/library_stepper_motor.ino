// Include the AccelStepper Library
// (search in the library manager "accelstepper")

// Library docs: http://www.airspayce.com/mikem/arduino/AccelStepper/

#include <AccelStepper.h>

// Define pin connections
const int DIR_PIN = 12; // Pin 11 was not changing the direction (probably damaged on the Arduino Leonardo)
const int STEP_PIN = 10;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, STEP_PIN, DIR_PIN);

void setup() {
  myStepper.setMaxSpeed(1000);
  myStepper.setSpeed(160);
}

void loop() {
  myStepper.runSpeed();
}
