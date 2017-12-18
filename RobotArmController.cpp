#include "RobotArmController.h"

char inputBuffer[MAX_COMMAND_LENGTH];
uint8_t index = 0;
boolean receiving = false;

Joint joints[NUMBER_OF_JOINTS] = {
		Joint("Base", 4, 90),
		Joint("Shoulder", 5, 65),
		Joint("Elbow", 6, 65),
		Joint("Wrist", 7, 90),
		Joint("Rotate", 8, 90),
		Joint("Grip", 9, 120),
		Joint("Pan", A0, 90),
		Joint("Tilt", A1, 90)
};

void setup() {

	Serial.begin(SER_BAUD);

	pinMode(PROBLEM_LED, OUTPUT);
	pinMode(HEARTLED, OUTPUT);
	digitalWrite(PROBLEM_LED, LOW);
	digitalWrite(HEARTLED, LOW);

	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		joints[i].attach(joints[i].getPin());
		delay(50);
		joints[i].moveToImmediate(joints[i].getPosition());
		delay(450);
	}

}

void loop() {

	if (Serial.available()) {

		char c = Serial.read();

		if (c == START_OP) {
			index = 0;
			inputBuffer[index] = 0;
			receiving = true;
		}

		if (receiving) {
			inputBuffer[index] = c;
			inputBuffer[++index] = 0;

			if (c == END_OP) {
				receiving = false;
				parseCommand();
			}

		}

	}

	heartbeat();

}

void heartbeat() {
	static unsigned long pt = millis();
	unsigned long ct = millis();

	if (ct - pt >= 500) {
		digitalWrite(HEARTLED, !digitalRead(HEARTLED));
		pt = ct;
	}
}

void parseCommand() {

	int jointIndex = -1;

	if (inputBuffer[0] == '<') {
		char* delimiters = "<,>";
		for (char* p = strtok(inputBuffer, delimiters); p != NULL;
				p = strtok(NULL, delimiters)) {

			//  S sets the active servo
			if (p[0] == 'S') {
				jointIndex = atoi((const char*) (p + 1));
			}
			//  R for requests for data
			else if (p[0] == 'R') {
				for (uint8_t i = 0; i < NUMBER_OF_JOINTS; i++) {
					Serial.print("<");
					Serial.print(i);
					Serial.print(",");
					Serial.print(joints[i].getPosition());
					Serial.print(">");
				}
			} else if (p[0] == 'C') {
				Serial.print("<C,0>");
			}
			//  Raw numbers get written to the currently active servo
			else {
				int position = atoi((const char*) p);
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					joints[jointIndex].moveToImmediate(position);
					// jointIndex = -1;   // comment this line to allow run-on commands
				}
			}
		}

		// clear the command
		inputBuffer[0] = 0;
	}

}

