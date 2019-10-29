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


DriveModeEnum currentDriveMode;

XboxHandler xbox;

StreamParser parser(&Serial, START_OF_PACKET, END_OF_PACKET, parseCommand);
extern CommandParser cp;

//  Joint (name, pin, starting pos, length, min us, min angle, max us, max angle)
Joint joints[NUMBER_OF_JOINTS] = {
		Joint(4, 1500, 37, 544, -0.34907, 2400, 3.31613),
		Joint(5, 1270, 105, 544, -0.087266, 2400, 2.91470),
		Joint(6, 2251, 98, 544, 2.96706, 2400, -0.13963),
		Joint(7, 1374, 158, 32, 605, -1.16937, 2400, 2.12930),
		Joint(8, 1500, 0, 564, -0.34907, 2400, 3.316126),
		Joint(9, 2250, 0, 1850, 1.923, 2400, PI),
		Joint(A0, 1420, 0, 600, -1.57, 2350, PI),
		Joint(A1, 1220, 0, 600, 0.87, 1470, -0.35)
};

Arm_Class arm(joints, NUMBER_OF_JOINTS - 2);
GimbalClass gimbal(&joints[NUMBER_OF_JOINTS - 2], &joints[NUMBER_OF_JOINTS - 1]);

//  declared in Defines.h for everyone to use.
boolean eepromGood(){
	byte flag = EEPROM.read(EEPROM_START_FLAG_ADDRESS);
	return (flag == EEPROM_START_FLAG_VALUE);
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
	gimbal.init();

	heartbeatDelay = 500;

	if(!eepromGood()){
		heartbeatDelay = 100;
		Serial.print(ARM_BAD_EEPROM);
	}

	initControllerFunctions(&arm, &xbox);

	for(int i=0; i<5; i++){
		joints[i].setSpeed(200);
	}
	joints[6].setSpeed(300);
	joints[7].setSpeed(300);

	parser.setRawCallback(rawDataCallback);

	Serial.print(ARM_INIT_COMPLETE);

}

void loop() {

	arm.run();             // things the arm does by itself
	gimbal.run();          // things the gimbal does
	parser.run();          // handle any serial data
	heartbeat();
	mainControllerLoop();  // xbox controller stuff

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
	gimbal.detach();
	delay(10);
	digitalWrite(SERVO_POWER_PIN, HIGH);
	delay(10);
	arm.init();   // 2 seconds of blocking delay !!!!
	gimbal.init();
}


void powerDownServos(){
	arm.detachAll();
	gimbal.detach();
	delay(10);
	digitalWrite(SERVO_POWER_PIN, LOW);
	delay(10);
}

void rawDataCallback(char* p){
	xboxCommandRaw(p);
}

void parseCommand(char *aCommand) {
	char inBuf[MAX_COMMAND_LENGTH];
	strncpy(inBuf, aCommand, MAX_COMMAND_LENGTH - 1);
	inBuf[MAX_COMMAND_LENGTH - 1] = 0;

	if (inBuf[0] == '<') {

		//  To program EEPROM by Serial, the P has to be the first thing in the command string
		//  This won't ever come through the robot since it can only forward messages starting
		//  with A or S.  This is only for plugging something into the arm to program it.
		//  If we want to do this over the air we will have to write more later.
		if (inBuf[1] == 'P') {
			// program EEPROM by Serial.
			parser.setCallback(programEEPROM);
			programmingEEPROM = true;
			while (programmingEEPROM) {
				parser.run();
			}
			parser.setCallback(parseCommand);
		}

		char *delimiters = "<,>";
		for (char *p = strtok(inBuf, delimiters); p != NULL;
				p = strtok(NULL, delimiters)) {
			cp.parseCommandString(p);
		}
		// clear the command
		inBuf[0] = 0;
	}
}


void programEEPROM(char* aCommand) {

	char inBuf[COM_PARSER_MAX_COMMAND_LENGTH];
	strncpy(inBuf, aCommand, COM_PARSER_MAX_COMMAND_LENGTH - 1);
	inBuf[COM_PARSER_MAX_COMMAND_LENGTH - 1] = 0;
	int address = -1;

	if (inBuf[0] == '<') {
		char* delimiters = "<,>";
		for (char* p = strtok(inBuf, delimiters); p != NULL;
				p = strtok(NULL, delimiters)) {

			switch (p[0]) {

			case 'G': {
				while(address < 1024){
					Serial.print(EEPROM.read(address), HEX);
					address++;
				}
				break;
			}

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
				if (address >= 0 && address < 1024) {
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
