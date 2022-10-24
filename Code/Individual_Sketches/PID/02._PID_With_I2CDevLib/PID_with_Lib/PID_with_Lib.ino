#include <Wire.h>

// From I2Cdev install zip the following 2 folders under Arduino and install them through the IDE
// https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;


/* MPU */
int16_t AcX, AcZ, GyX;

const int TILT_RANGE = 16000; // 0 - 16 000 (corresponds to approx. 0 - 90°)

/* MOTOR */
volatile int motorPower;

/* PID */
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;

/*
 * Fine tuning the PID constants
1. Set Ki and Kd to zero and gradually increase Kp so that the robot starts to oscillate about the zero position.

2. Increase Ki so that the response of the robot is faster when it is out of balance. 
Ki should be large enough so that the angle of inclination does not increase. 
The robot should come back to zero position if it is inclined.

3. Increase Kd so as to reduce the oscillations. The overshoots should also be reduced by now.

4. Repeat the above steps by fine tuning each parameter to achieve the best result.
*/
const float Kp = 0.5;
const float Ki = 0.05;
const float Kd = 0.01;
const float targetAngle = 0;

unsigned long prevTimePID;
unsigned long currentTimePID;
float elapsedTimeSecondsPID;

void setup() {  
  Serial.begin(9600);
  
  mpu.initialize();
  // todo raise the value when done developing
  mpu.CalibrateAccel(2);
  mpu.CalibrateGyro(2);

  prevTimePID = millis();
}



void loop() {
  AcX = mpu.getAccelerationX();
  AcZ = mpu.getAccelerationZ();  
  GyX = mpu.getRotationX();
  
  calculatePID();

  // todo remove
  delay(40);
  Serial.println(motorPower);

}

void calculatePID() {
  accAngle = atan2f(AcX, AcZ)*RAD_TO_DEG;
 

  currentTimePID = millis();
  elapsedTimeSecondsPID = (currentTimePID - prevTimePID) / 1000.0f;
  prevTimePID = currentTimePID;

  gyroAngle = (float) GyX * (elapsedTimeSecondsPID);


  // even the angle out by adding to the previous angle
  // the magic numbers are part of the equation for the complementary filter
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);

  error = currentAngle - targetAngle;
  errorSum = error;  
  errorSum = constrain(errorSum, -300, 300);

  // calculate output from P, I and D values
  motorPower = Kp*error + Ki*errorSum*elapsedTimeSecondsPID - Kd*(currentAngle-prevAngle)/elapsedTimeSecondsPID;
  motorPower = constrain(motorPower, -255, 255);
  
  prevAngle = currentAngle;
}
