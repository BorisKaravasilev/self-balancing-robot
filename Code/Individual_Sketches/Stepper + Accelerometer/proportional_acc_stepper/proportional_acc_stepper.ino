// Include the AccelStepper Library
// (search in the library manager "accelstepper")
// Library docs: http://www.airspayce.com/mikem/arduino/AccelStepper/

#include <Wire.h>
#include <AccelStepper.h>

// PIN CONNECTIONS
const int M1_DIR_PIN = 12; // Pin 11 was not changing the direction (probably damaged on the Arduino Leonardo)
const int M1_STEP_PIN = 10;

const int M2_DIR_PIN = 8;
const int M2_STEP_PIN = 9;

// CONSTANTS
const int MPU = 0x68; // I2C address of the MPU-6050
const int TILT_RANGE = 16000; // 0 - 16 000 (corresponds to approx. 0 - 90°)
const int MOTOR_MAX_SPEED = 1000;

// GLOBAL VARIABLES
int16_t tilt_y;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper stepper1(motorInterfaceType, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(motorInterfaceType, M2_STEP_PIN, M2_DIR_PIN);

void setup() {
  setupMPU();

  stepper1.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper2.setMaxSpeed(MOTOR_MAX_SPEED);
  
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);

  Serial.begin(9600);
}

void loop() {
  read_accelerometer();
//  Serial.print("Tilt Y:"); Serial.println(tilt_y); // Printing to the terminal slows down the loop (motor speed) a lot
  
  int motor_speed = map(tilt_y, -TILT_RANGE, TILT_RANGE, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  motor_speed = constrain(motor_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  
//  Serial.print("Motor speed:"); Serial.println(motor_speed);

  stepper1.setSpeed(-motor_speed);
  stepper2.setSpeed(motor_speed); // inverted speed because motors are facing opposite directions
  
  stepper1.runSpeed();
  stepper2.runSpeed();

//  delay(300);
}


void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // accessing the PWR_MGMT_1 register
  Wire.write(0);   // set to zero wakes up the MPU-6050 
                   // goes straight to sleep mode after being woken up thus the two step begin transmission in the loop
  Wire.endTransmission(true);
}

void read_accelerometer(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);  // request a total of 14 register bytes from the MPU

  tilt_y = Wire.read()<<8|Wire.read();
}
