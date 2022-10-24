// Standard libraries
#include <Wire.h>
#include <LiquidCrystal.h>
#include <Servo.h>

// Our files


/* Include the AccelStepper Library
(search in the library manager "accelstepper")
Library docs: http://www.airspayce.com/mikem/arduino/AccelStepper/ */
#include <AccelStepper.h>

/* From I2Cdev install zip the following 2 folders under Arduino and install them through the IDE
https://github.com/jrowberg/i2cdevlib

 1) Download the ZIP from GitHub
 2) Go to "i2cdevlib-master.zip > i2cdevlib-master > Arduino"
 3) Zip "I2Cdev" and "MPU6050" into individual zip archives
 4) Include them one by one through the Arduino IDE
      -->  "Sketch > Include Library > Add .ZIP Library..."
*/
#include "I2Cdev.h"
#include "MPU6050.h"


//**************************************************************
// GLOBAL VARIABLES
//**************************************************************

/*
 * Fine tuning the PID constants
1. Set Ki and Kd to zero and gradually increase Kp so that the robot starts to oscillate about the zero position.

2. Increase Ki so that the response of the robot is faster when it is out of balance. 
Ki should be large enough so that the angle of inclination does not increase. 
The robot should come back to zero position if it is inclined.

3. Increase Kd so as to reduce the oscillations. The overshoots should also be reduced by now.

4. Repeat the above steps by fine tuning each parameter to achieve the best result.
*/
float Kp = 150; //100;
float Ki = 5.1; // Influence of the sum of integrated errors in the final equation
float Kd = 0;

const float SPEED_OF_INTEGRATION = 460; // Multiplier of the integrated error
const float MAX_INTEGRATION_SUM = 70; // Floor and ceiling in order to make it easier to go back to zero sum
                                       // (to not be too far in the positive or negative numbers)


// Define how often code in inner loop should run
const int loopMax = 1000;
int loopCounter = loopMax;



//**************************************************************
// Stepper motor variables
//**************************************************************

// Define motor interface type
#define motorInterfaceType 1

const int MOTOR_MAX_SPEED = 1000;

// PIN CONNECTIONS
const int M1_DIR_PIN = 11; 
const int M1_STEP_PIN = 10;

const int M2_DIR_PIN = 12; // 8 to 13
const int M2_STEP_PIN = 13; // 9 to 13

// LCD Data Pins

const int LCD_d_4 = 2;
const int LCD_d_5 = 3;
const int LCD_d_6 = 4;
const int LCD_d_7 = 7;

const int LCD_Register_Sel = 5;
const int LCD_Enable = 6;

// Creates an instance
AccelStepper stepper1(motorInterfaceType, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(motorInterfaceType, M2_STEP_PIN, M2_DIR_PIN);


//**************************************************************
// LCD Screen
//**************************************************************

// Setup LCD pins to Arduino pins
LiquidCrystal lcd(LCD_Register_Sel, 
                  LCD_Enable, 
                  LCD_d_4, 
                  LCD_d_5, 
                  LCD_d_6, 
                  LCD_d_7);

// Ranges for PID tweak values
// Change these when testing
float KpMin = 50;
float KpMax = 200;

float KiMin = 0;
float KiMax = 120;

float KdMin = 0;
float KdMax = 0.01;

// Maximum analog read value for our arduino
const int PotMax = 1023;

const float KpFactor = (KpMax - KpMin) / PotMax;
const float KiFactor = (KiMax - KiMin) / PotMax;
const float KdFactor = (KdMax - KdMin) / PotMax;

// The readings from the potentiometers
int KpRead = 0;
int KiRead = 0;
int KdRead = 0;

// PID Tweaker pins
const int TWEAK_KP_PIN = A0;
const int TWEAK_KI_PIN = A1;
const int TWEAK_KD_PIN = A2;



//**************************************************************
// MPU6050 variables = Accelerometer + Gyroscope 
//**************************************************************
MPU6050 mpu;

/* MPU */
int16_t AcY, AcZ, GyX;


//**************************************************************
// PID variables
//**************************************************************

/* MOTOR */
volatile int motorPower, gyroRate;

/* PID */
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0, errorRate=0;

const float targetAngle = -12.22;
/* This is assuming that it is in a forward resting position on the support wheels. If leaning back then change the sign.
 *  Code for recalculating the target angle
 *    float sum = 0;
  float readings = 20000;
  for (int i = 0; i < readings; i++) {
      AcY = mpu.getAccelerationY();
      AcZ = mpu.getAccelerationZ(); 
      sum += atan2(AcY, AcZ)*RAD_TO_DEG;
  }
  Serial.println(sum / readings);
 */

unsigned long prevTimePID;
unsigned long currentTimePID;
float elapsedTimePIDSeconds;

//**************************************************************
// Servos - Legs
//**************************************************************

// One leg on each side share the same pins
const int SERVO_LEFT_LEG_PIN = 8; // todo change to the right pin
const int SERVO_RIGHT_LEG_PIN = 9; // todo change to the right pin
//LegsStepper rightLeg;
//LegsStepper leftLeg;
Servo rightLeg;
Servo leftLeg;

//**************************************************************
// SETUP
//**************************************************************
void setup() {
  Serial.begin(9600);
  
  // ******** Stepper Motor
  stepper1.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper2.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);

  // ******** Leg Servos
//  rightLeg.initialize(SERVO_RIGHT_LEG_PIN);
//  leftLeg.initialize(SERVO_LEFT_LEG_PIN);
  rightLeg.attach(SERVO_RIGHT_LEG_PIN);

  // ******** LCD - Welcome Screen
  // Setup the first (unchanging) line of the display
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Callibrating...");
  lcd.setCursor(0, 1);
  lcd.print("PS Loves You");

  // this is because turning on the power supply jolts the motors and this interfers with the mpu callibration. 
  delay(400);

  // ******** MPU
  mpu.initialize();
  // todo raise the value when done developing
  mpu.CalibrateAccel(8);
  mpu.CalibrateGyro(8);

  // ******** LCD - Info Screen
  lcd.setCursor(0, 0);
  lcd.print(" P     I    D  ");
  lcd.setCursor(0, 1);
  lcd.print("               ");

  // ******** PID
  prevTimePID = millis();
 
}

//**************************************************************
// LOOP
//**************************************************************
void loop() {
  AcY = mpu.getAccelerationY();
  AcZ = mpu.getAccelerationZ();  
  GyX = mpu.getRotationX();
  
  calculatePID();

  stepper1.setSpeed(motorPower);
  stepper2.setSpeed(-motorPower); // inverted speed because motors are facing opposite directions
  
  stepper1.runSpeed();
  stepper2.runSpeed();
 
  if (loopCounter < loopMax) {
      loopCounter++;
    } else {
      loopCounter = 0;
      updatePIDValuesFromTweaker();
      showPIDValuesOnScreen();
  }

  /* ESP CODE === NEW */
  // since this is not in a while loop constantly receiving it might miss the entire command
  if (Serial.available()) {
    handleClientCommands();
  }
}


void calculatePID() {
  currentTimePID = millis();
  elapsedTimePIDSeconds = (currentTimePID - prevTimePID) / 1000.0f;

  accAngle = atan2(AcY, AcZ)*RAD_TO_DEG;

  // The MPU6050 outputs values in a format in a range that needs to be converted
  gyroRate = map(GyX, -32768, 32767, -250, 250);
  gyroAngle = (float) gyroRate * (elapsedTimePIDSeconds);


  // even the angle out by adding to the previous angle
  // this is the formula for the complementary filter
  // Page 11: https://d1.amobbs.com/bbs_upload782111/files_44/ourdev_665531S2JZG6.pdf 
  currentAngle = 0.98 * (prevAngle + gyroAngle) + 0.02 * (accAngle);

  error = currentAngle - targetAngle;
  errorSum += SPEED_OF_INTEGRATION * error * elapsedTimePIDSeconds;

  errorSum = constrain(errorSum, -MAX_INTEGRATION_SUM, MAX_INTEGRATION_SUM);

  errorRate = (currentAngle-prevAngle)/elapsedTimePIDSeconds;
  
  // calculate output from P, I and D values
  motorPower = Kp*error + Ki*errorSum - Kd*errorRate;
  motorPower = constrain(motorPower, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  
  
  prevAngle = currentAngle;
  prevTimePID = currentTimePID;
}

void updatePIDValuesFromTweaker() {

  // Find the proportional, update the value and write to LCD
  KpRead = PotMax - analogRead(TWEAK_KP_PIN);
  Kp = KpRead * KpFactor + KpMin;


  // Find the integral, update the value and write to LCD
  KiRead = PotMax - analogRead(TWEAK_KI_PIN);
  Ki = KiRead * KiFactor + KiMin;


  // Find the derivative, update the value and write to LCD
  KdRead = PotMax - analogRead(TWEAK_KD_PIN);
  Kd = KdRead * KdFactor + KdMin;

}

void showPIDValuesOnScreen() {
  lcd.setCursor(0, 1);
  lcd.print(Kp);
  
  lcd.setCursor(6, 1);
  lcd.print(Ki);
  
  lcd.setCursor(12, 1);
  lcd.print(Kd);
}

void moveLegUp(Servo servo) {
  servo.write(180);
}

void moveLegDown(Servo servo) {
  servo.write(0);
}

/* ESP CODE === NEW */
String receivedCommand = "";
bool dataIn = false;

void handleClientCommands() {
  while (Serial.available()) {
		char c = Serial.read(); //read it

		if (c == '[') {
			// this is the start of the command string
			receivedCommand = "";
			dataIn = true;
		}
		// otherwise, we are still reading the command string:
		else if (dataIn && c != ']'){
			receivedCommand += c;
		}

		else if (dataIn && c == ']') {
			//finished receiving the command, process it

			if (receivedCommand == "DELAY"){
        delay(10000);
			}
		}
	}
}
