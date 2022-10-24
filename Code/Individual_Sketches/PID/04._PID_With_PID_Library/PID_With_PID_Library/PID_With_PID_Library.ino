// https://playground.arduino.cc/Code/PIDLibrary/

// https://github.com/kurimawxx00/arduino-self-balancing-robot/blob/master/AmBOT_final_nano.ino
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"

double Kp = 50;
double Kd = 1.4;
double Ki = 60;



MPU6050 mpu;

volatile float accAngle;

int16_t AcX, AcZ;
double Setpoint, Input, Output;

PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    mpu.initialize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    // turn the PID on
    pid.SetMode(AUTOMATIC);
//    pid.SetSampleTime(10);
//    pid.SetOutputLimits(-1000, 1000); 

    Serial.begin(9600);
}

void loop() {
    AcX = mpu.getAccelerationX();
    AcZ = mpu.getAccelerationX();

    accAngle = atan2f(AcX, AcZ)*RAD_TO_DEG;

    Input = accAngle;

    pid.Compute();
    Serial.println(Output);
    delay(200);
}
