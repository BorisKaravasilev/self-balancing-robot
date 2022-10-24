const int DIR_PIN = 11;
const int STEP_PIN = 10;

const int STEP_DELAY = 20;

void setup() {
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);

    digitalWrite(DIR_PIN, 1);   // Set the direction
    digitalWrite(STEP_PIN, 0);  // Initialize step pin to 0
}

void loop() {
    // Generate step pulses (square wave X ms wide)
    digitalWrite(STEP_PIN, 1);
    delay(STEP_DELAY);
    digitalWrite(STEP_PIN, 0);
    delay(STEP_DELAY);
}