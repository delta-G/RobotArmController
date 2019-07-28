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

boolean programmingEEPROM = false;


unsigned int heartbeatDelay = 250;

StreamParser parser(&Serial, START_OF_PACKET, END_OF_PACKET, parseCommand);

//  Joint (name, pin, starting pos, min us, min angle, max us, max angle)
Joint joints[NUMBER_OF_JOINTS] = {
		Joint("Base", 4, 1500, 544, 3.49066, 2400, -0.17453),
		Joint("Shoulder", 5, 1215, 544, -0.087266, 2400, 2.91470),
		Joint("Elbow", 6, 1215, 544, 2.96706, 2400, -0.13963),
		Joint("Wrist", 7, 1500, 605, -1.22173, 2400, 2.024582),
		Joint("Rotate", 8, 1500, 564, -0.34907, 2400, 3.316126),
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

//	//TODO:  COMMENT THIS BACK OUT
//	EEPROM.write(0, 0x47);
//	EEPROM.write(1, 0xFF);
//	//TODO:  COMMENT THIS BACK OUT

	arm.init();  // 2 seconds of blocking delay!!!
	delay(250);

	heartbeatDelay = 500;

	if(!eepromGood()){
		heartbeatDelay = 100;
		Serial.print(ARM_BAD_EEPROM);
	}

	Serial.print(ARM_INIT_COMPLETE);

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
	strncpy(inBuf, aCommand, MAX_COMMAND_LENGTH - 1);
	inBuf[MAX_COMMAND_LENGTH - 1] = 0;
	int jointIndex = -1;

	if (inBuf[0] == '<') {
		char* delimiters = "<,>";
		for (char* p = strtok(inBuf, delimiters); p != NULL;
				p = strtok(NULL, delimiters)) {

			switch (p[0]) {
			//  S sets the active servo
			case 'S': {
				jointIndex = atoi((const char*) (p + 1));
				break;
			}
				//  R for requests for data
			case 'R': {
				for (uint8_t i = 0; i < NUMBER_OF_JOINTS; i++) {
					Serial.print("<");
					Serial.print(i);
					Serial.print(",");
					Serial.print(joints[i].getPosition());
//					Serial.print(",");
//					Serial.print(joints[i].isMoving());
					Serial.print(">");
				}
				break;
			}
			case 'B': {
				Serial.print(ARM_CONNECT_RESPONSE);
				break;
			}
			case 'C': {
				Serial.print("<SavCal>");
				arm.saveCalibrations();
				break;
			}
			case 'c': {
				Serial.print("<LodCal>");
				arm.loadCalibrations();
				break;
			}
			case 'T': {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					int targ = atoi((const char*) (p + 1));
					joints[jointIndex].setTarget(targ);
				}
				break;
			}
			case 'L': {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					int targ = atoi((const char*) (p+1));
					float flargRad = (float)targ * PI / 180;
					joints[jointIndex].setTargetAngle(flargRad);
				}
				break;
			}
			case 's': {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					int spd = atoi((const char*) (p + 1));
					joints[jointIndex].setSpeed(spd);
				}
				break;
			}
			case 'J': {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					int stickPos = atoi((const char*) (p + 1));
					joints[jointIndex].useStick(stickPos);
				}
				break;
			}
			case 'F': {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					int stickPos = atoi((const char*) (p + 1));
					joints[jointIndex].followTheStick(stickPos);
				}
				break;
			}
			case 'A': {
				arm.attachAll();
				break;
			}
			case 'D': {
				arm.detachAll();
				break;
			}
			case 'X': {
				arm.stop();
				break;
			}
			case 'x': {
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					joints[jointIndex].stop();
				}
				break;
			}
			case 'E': {
				powerUpServos();
				break;
			}
			case 'e': {
				powerDownServos();
				break;
			}
			case 'P': {

				// program EEPROM by Serial.
				parser.setCallback(programEEPROM);
				programmingEEPROM= true;
				while(programmingEEPROM){
					parser.run();
				}
				parser.setCallback(parseCommand);

				break;
			}
				//  Raw numbers get written to the currently active servo
			case '0' ... '9': {
				int position = atoi((const char*) p);
				if (jointIndex >= 0 && jointIndex < NUMBER_OF_JOINTS) {
					joints[jointIndex].moveToImmediate(position);
					// jointIndex = -1;   // comment this line to allow run-on commands
				}
				break;
			}

			default : {
				Serial.print("<BADC,");
				Serial.print(*p);
				Serial.print(">");
			}

			} //  End of Switch
		}

		// clear the command
		inBuf[0] = 0;
	}

}



void programEEPROM(char* aCommand) {

	char inBuf[MAX_COMMAND_LENGTH];
	strncpy(inBuf, aCommand, MAX_COMMAND_LENGTH - 1);
	inBuf[MAX_COMMAND_LENGTH - 1] = 0;
	int address = -1;

	if (inBuf[0] == '<') {
		char* delimiters = "<,>";
		for (char* p = strtok(inBuf, delimiters); p != NULL;
				p = strtok(NULL, delimiters)) {

			switch (p[0]) {

			case 'L': {
				address = atoi(p + 1);
				break;
			}

			case 'Q': {
				programmingEEPROM = false;
				break;
			}

			case 'R': {
				// reset everything
				// TODO:  Implement this!
				break;
			}
			case 'H':
				// get off the H if there is one:
				// then fall through to process hex digits
				p++; // @suppress("No break at end of case")

			// then all hex digits go straight to EEPROM
			case 'A' ... 'F':
			case 'a' ... 'f':
			case '0' ... '9':
			 {
				if (address >= 0) {
					// as long as we have two more hex digits
					while ((isxdigit(*p)) && (isxdigit(*(p+1)))) {
						char c[3];
						c[0] = *p;
						c[1] = *(p + 1);
						c[2] = 0;
						byte b = strtoul(c, NULL, 16) & 0xFF;
						writeToEEPROM(address, b);
						address += 1;
						p += 2;
					}
				}
				break;
			}
			default :
				break;
			} // end switch

		}
	}

}
