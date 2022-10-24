#include <Wire.h>

/* MPU */
const int MPU = 0x68; // I2C address of the MPU-6050
//int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t tilt_y, AcY, AcZ, Tmp, GyX, GyY, GyZ;

const int TILT_RANGE = 16000; // 0 - 16 000 (corresponds to approx. 0 - 90°)

/* MOTOR */
volatile int motorPower, gyroRate;

/* PID */
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;

const int Kp = 40;
const int Ki = 40;
const int Kd = 0.05;
const float targetAngle = -2.5;

long prevTimePID;
long currentTimePID;
float elapsedTimeSecondsPID;

void setup() {
  setupMPU();
  
  Serial.begin(9600);

  initializePID();

  prevTimePID = millis();
}

void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // accessing the PWR_MGMT_1 register
  Wire.write(0);   // set to zero wakes up the MPU-6050 
                   // goes straight to sleep mode after being woken up thus the two step begin transmission in the loop
  Wire.endTransmission(true);
}

void initializePID() {

}


void loop() {
  read_accelerometer();

  

  calculatePID();

  Serial.println(motorPower);

}

// ISR (Interrupt Service Routine) is a function that will be called every 5 miliseconds
// TIMER1_COMPA_vect is a library constant that needs to be given
// ISR(TIMER1_COMPA_vect) {}

void calculatePID() {
  accAngle = atan2(AcY, AcZ)*RAD_TO_DEG;
  
  // convert it gyro around x to degrees per second and then multiply it with the loop time to obtain the change in angle
  gyroRate = map(GyX, -32768, 32767, -250, 250);

  currentTimePID = millis();
  elapsedTimeSecondsPID = (currentTimePID - prevTimePID) / 1000;
  prevTimePID = currentTimePID;
  
  gyroAngle = (float) gyroRate * (elapsedTimeSecondsPID);

  // even the angle out by adding to the previous angle
  // the magic numbers are part of the equation for the complementary filter
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);

  // calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*elapsedTimeSecondsPID - Kd*(currentAngle-prevAngle)/elapsedTimeSecondsPID;
  motorPower = constrain(motorPower, -255, 255);
  
  prevAngle = currentAngle;
}

void motor_speed_tmp() {
  int motor_speed = 0;
  bool motor_dir = 0;

  if (tilt_y > 0){
    motor_speed = map(tilt_y, 0, TILT_RANGE, 0, 255);
    motor_dir = 0;
  } else {
    motor_speed = map(tilt_y, 0, -TILT_RANGE, 0, 255);
    motor_dir = 1;
  }
  
  Serial.print("Motor direction: "); Serial.println(motor_dir);
  Serial.print("Motor speed: "); Serial.println(motor_speed);
}

void read_accelerometer() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);  // request a total of 14 register bytes from the MPU

  tilt_y = Wire.read()<<8|Wire.read();
  AcY = Wire.read()<<8|Wire.read();
  AcZ = Wire.read()<<8|Wire.read();
  Tmp = Wire.read()<<8|Wire.read(); // temperature to be discarded
  GyX = Wire.read()<<8|Wire.read();
  GyY = Wire.read()<<8|Wire.read();
  GyZ = Wire.read()<<8|Wire.read();
}
