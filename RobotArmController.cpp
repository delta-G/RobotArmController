/*

RobotArmController  --  runs onArduino Nano and handles the Arm for my robot
     Copyright (C) 2017  David C.

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

     */

#include "RobotArmController.h"

//char inputBuffer[MAX_COMMAND_LENGTH];
uint8_t index = 0;
boolean receiving = false;


unsigned int heartbeatDelay = 250;

StreamParser parser(&Serial, START_OF_PACKET, END_OF_PACKET, parseCommand);

//  Joint (name, pin, starting pos, min us, min angle, max us, max angle)
Joint joints[NUMBER_OF_JOINTS] = {
		Joint("Base", 4, 1500, 544, 0, 2400, PI),
		Joint("Shoulder", 5, 1215, 544, 0, 2400, PI),
		Joint("Elbow", 6, 1215, 544, 0, 2400, PI),
		Joint("Wrist", 7, 1500, 544, 0, 2400, PI),
		Joint("Rotate", 8, 1500, 544, 0, 2400, PI),
		Joint("Grip", 9, 1781, 1680, 1.923, 2400, PI),
		Joint("Pan", A0, 1350),
		Joint("Tilt", A1, 1220)
};

Arm_Class arm(joints, NUMBER_OF_JOINTS);

//  declared in Defines.h for everyone to use.
boolean eepromGood(){
	byte flag = EEPROM.read(EEPROM_START_FLAG);
	return (flag == EEPROM_START_VALUE);
}


void setup() {

	// Servo Power Enable
	pinMode(SERVO_POWER_PIN, OUTPUT);
	digitalWrite(SERVO_POWER_PIN, LOW);  //powers arm OFF

	Serial.begin(ARM_BOARD_BAUD);
	pinMode(HEARTLED, OUTPUT);
	digitalWrite(HEARTLED, LOW);

	for (int i = 0; i < 5; i++){
		digitalWrite(HEARTLED, HIGH);
		delay(100);
		digitalWrite(HEARTLED, LOW);
		delay(100);
	}

	//TODO:  COMMENT THIS BACK OUT
	EEPROM.write(0, 0x47);
	EEPROM.write(1, 0xFF);
	//TODO:  COMMENT THIS BACK OUT

	arm.init();
	delay(250);

	heartbeatDelay = 500;

	if(!eepromGood()){
		heartbeatDelay = 100;
	}

}

void loop() {

	arm.run();
	parser.run();
	heartbeat();

}

void heartbeat() {
	static unsigned long pt = millis();
	unsigned long ct = millis();
	if (ct - pt >= heartbeatDelay) {
		digitalWrite(HEARTLED, !digitalRead(HEARTLED));
		pt = ct;
	}
}

void powerUpServos(){
	arm.detachAll();
	delay(10);
	digitalWrite(A3, HIGH);
	delay(10);
	arm.init();   // 2 seconds of blocking delay !!!!
}


void powerDownServos(){
	arm.detachAll();
	delay(10);
	digitalWrite(A3, LOW);
	delay(10);
}


void parseCommand(char* aCommand) {
	char inBuf[MAX_COMMAND_LENGTH];
	strncpy(inBuf, aCommand, MAX_COMMAND_LENGTH -1);
	inBuf[MAX_COMMAND_LENGTH -1] = 0;
	int jointIndex = -1;

	if (inBuf[0] == '<') {
		char* delimiters = "<,>";
		for (char* p = strtok(inBuf, delimiters); p != NULL;
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
//					Serial.print(",");
//					Serial.print(joints[i].isMoving());
					Serial.print(">");
				}
			} else if (p[0] == 'B') {
				Serial.print("<Booted / Connected>");
			} else if (p[0] == 'C') {
				Serial.print("<SavCal>");
				arm.saveCalibrations(EEPROM_CALIBRATION_START);
			} else if (p[0] == 'c') {
				Serial.print("<LodCal>");
				arm.loadCalibrations(EEPROM_CALIBRATION_START);
			} else if (p[0] == 'T') {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					int targ = atoi((const char*) (p + 2));
					joints[jointIndex].setTarget(targ);
				}
			} else if (p[0] == 's') {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					int spd = atoi((const char*) (p + 2));
					joints[jointIndex].setSpeed(spd);
				}
			} else if (p[0] == 'P') {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					int stickPos = atoi((const char*) (p + 1));
					joints[jointIndex].useStick(stickPos);
				}
			} else if (p[0] == 'X') {
				arm.stop();
			} else if (p[0] == 'x') {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					joints[jointIndex].stop();
				}
			} else if (p[0] == 'E') {
				powerUpServos();
			} else if (p[0] == 'e') {
				powerDownServos();
			}
			//  Raw numbers get written to the currently active servo
			else if (isDigit(p[0])){
				int position = atoi((const char*) p);
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					joints[jointIndex].moveToImmediate(position);
					// jointIndex = -1;   // comment this line to allow run-on commands
				}
			}
		}

		// clear the command
		inBuf[0] = 0;
	}

}


