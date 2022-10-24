#define SERIAL_BAUD 115200

/*******************************************
	In this code we have used "[]" to surround our command codes
	As a bit of a proof of concept for how to use the XC4411 board
*********************************************/
const int led_pin = 13;

String receivedCommand = "";
bool dataIn = false;

void setup() {
	// put your setup code here, to run once:

	Serial.begin(SERIAL_BAUD); //same as ESP baud
	pinMode(led_pin, OUTPUT);
}

void loop() {
	// put your main code here, to run repeatedly:

	while (Serial.available()){
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
			Serial.print("XC4411 has been running for ");
			Serial.print(millis(), DEC);
			Serial.println(" milliseconds..");
			Serial.print("Received command was '");
			Serial.print(receivedCommand);
			Serial.print("' - action: ");

			if (receivedCommand == "LEDON"){
				Serial.println("TURN LED ON");
				digitalWrite(led_pin, HIGH);
			}
			else if (receivedCommand == "LEDOFF") {
				Serial.println("TURN LED OFF");
				digitalWrite(led_pin, LOW);
			}
			else if (receivedCommand == "LEDTOGGLE") {
				Serial.println("CHANGE LED");
				digitalWrite(led_pin, !digitalRead(led_pin));
			}
			Serial.println("---------------------------------------");
		}
	}
	delay(10);
}
