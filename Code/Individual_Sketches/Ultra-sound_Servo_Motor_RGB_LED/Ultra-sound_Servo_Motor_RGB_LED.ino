#include <Servo.h>

#include <FastLED.h>


Servo servo;

#define SERVO_PIN   6

int turnValue = 0;
bool isTurningRight = true;
int servoRotationIncrement = 4;


#define trigPin 2 //attach pin D3 Arduino to pin Trig of HC-SR04
#define echoPin 3 // attach pin D2 Arduino to pin Echo of HC-SR04


long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

#define LED_PIN     5
#define NUM_LEDS    1
#define BRIGHTNESS  32
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];



void setup() {
  servo.attach(SERVO_PIN);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  Serial.begin(9600);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(BRIGHTNESS);
  leds[0].r = 255; 
}

void loop() {
    if (isTurningRight) {
      turnValue += servoRotationIncrement;     
    } else {
     turnValue -= servoRotationIncrement;     
    }
    if (turnValue >= 180) {
      isTurningRight = false;
    }
    else if (turnValue <= 0) {
      isTurningRight = true;
    }    
    
//    servo.write(turnValue);                 
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    leds[0].g = 255 - 2 * distance; 
    leds[0].g = 255 - 2 * distance; 
    
    FastLED.show();
    FastLED.delay(100);
    
    delay(100);
}
