#include <Wire.h>

const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // accessing the PWR_MGMT_1 register
  Wire.write(0);   // set to zero wakes up the MPU-6050 
                   // goes straight to sleep mode after being woken up thus the two step begin transmission in the loop
  Wire.endTransmission(true);
  
  Serial.begin(9600);
}

void setupMPU() {
  
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);  // request a total of 14 register bytes from the MPU
  
  AcX = Wire.read()<<8|Wire.read();    
  AcY = Wire.read()<<8|Wire.read();  
  AcZ = Wire.read()<<8|Wire.read();
  Tmp = Wire.read()<<8|Wire.read(); // temperature to be discarded
  GyX = Wire.read()<<8|Wire.read();  
  GyY = Wire.read()<<8|Wire.read();  
  GyZ = Wire.read()<<8|Wire.read();  

  print_values();
//  plot_acc_values();

  delay(600);
}

void print_values() {
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = "); Serial.println(AcZ); 
  
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);

//  Serial.print(" | TMP = "); Serial.println(Tmp/340.00+36.53);

  Serial.println(" ");
}

void plot_acc_values() {
  Serial.print("AcX:");
  Serial.print(AcX);

  Serial.print(",");
//
//  Serial.print("AcY:");
//  Serial.println(AcY);
//
//  Serial.print(",");
//  
//  Serial.print("AcZ:");
//  Serial.println(AcZ);
}
