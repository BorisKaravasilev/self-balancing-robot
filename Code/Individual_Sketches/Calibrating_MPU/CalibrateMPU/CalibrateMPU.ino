#include "Wire.h"

// From I2Cdev install zip the following 2 folders under Arduino and install them through the IDE
// https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
int16_t rawAccX, rawAccY, rawAccZ;

void setup() {
  mpu.initialize();

  Serial.begin(9600);
  calibrateOffset();
  
  
}

void loop() {
  // read acceleration
  accX = mpu.getAccelerationX();  
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  

  Serial.print("accX: ");
  Serial.println(accX);

  Serial.print("accY: ");
  Serial.println(accY);

  Serial.print("accZ: ");
  Serial.println(accZ);

  Serial.println();

  // read gyroscope values
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();
  gyroZ = mpu.getRotationZ();

  
//  Serial.print("gyroX: ");
//  Serial.println(gyroX);
//
//  Serial.print("gyroY: ");
//  Serial.println(gyroY);
//
//  Serial.print("gyroZ: ");
//  Serial.println(gyroZ);

  delay(800);
}

// This is for development to save time
void hardcodedOffsets() {
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setZGyroOffset(0);
}

void calibrateOffset() {
  Serial.println("Calibrating the MPU");
  mpu.CalibrateAccel(8);
  mpu.CalibrateGyro(8);

  Serial.println("600 Readings");
  mpu.PrintActiveOffsets();
  Serial.println();
}
