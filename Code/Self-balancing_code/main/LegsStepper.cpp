#include "LegsStepper.h"

#include <Servo.h>
Servo servo;

// this is to account for the fact that the servos are opposite each other so 0 for one is position 180 for another
int topIsInPosition = 0;

LegsStepper::LegsStepper() {
  
}

void LegsStepper::initialize(int pinNumber, int topIsInPosition) {
  servo.attach(pinNumber);
  topIsInPosition = topIsInPosition;
}

void moveTo(int position) {
  servo.write(position + topIsInPosition);
}


void LegsStepper::moveToTop() {
  servo.write(0 + topIsInPosition);
}

void LegsStepper::moveToBottom() {
  servo.write(180 + topIsInPosition);
}
