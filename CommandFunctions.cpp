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

#include "CommandFunctions.h"

/*  These functions (unlike RMB) expect the start and end stripped off
 *  the parser for these uses strtok to cycle through a big string of commands
 */

extern Arm_Class arm;
extern XboxHandler xbox;

extern DriveModeEnum currentDriveMode;

int jointIndex = -1;

//  P is reserved for programming the EEPROM
//  the callback that is calling us is checking for it
Command commands[] = {
		{ 'X', xboxCommand },
		{ 'A', startCommands },
		{ 'S', setJointIndex },
		{ 'R', requestFromArm },
		{ 'B', bootResponse },
		{ 'T', setTarget },
		{ 'L', setAngle },
		{ 's', setSpeed },
		{ 'J', useStick },
		{ 'F', followStick },
		{ 'C', controlCodes },
		{ '#', moveToPosition }
};


CommandParser cp(&commands[0], NUM_ELEMENTS(commands), false);

void xboxCommand(char* p){
	xbox.handleIncomingASCII(p+1);
}

void startCommands(char* p){
	// RMB gets an A as a signal to send to arm
	// This is that A
	// set up for a new string of commands
	jointIndex = -1;
}

void setJointIndex(char* p){
	jointIndex = atoi((const char*) (p + 1));
}

void requestFromArm(char *p) {
	switch (p[1]) {
	case 'G': {
			char gitbuf[9];
			strncpy(gitbuf, GIT_HASH, 8);
			gitbuf[8] = 0;
			Serial.print("<RMBGIT-");
			Serial.print(gitbuf);
			Serial.print(">");
			break;
		}

	case 'p': {
		uint8_t rawBuf[21];
		rawBuf[0] = '<';
		rawBuf[1] = 0x12;
		rawBuf[2] = 21;
		rawBuf[3] = 'p';
		for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

			rawBuf[(2 * i) + 4] = (byte)((arm.getJoint(i)->getPosition()) >> 8) & 0xFF;
			rawBuf[(2 * i) + 5] = (byte)(arm.getJoint(i)->getPosition()) & 0xFF;

		}
		rawBuf[20] = '>';
		for(int i=0; i<21; i++){
			Serial.write(rawBuf[i]);
		}
		break;
	}
	case 't':{
		uint8_t rawBuf[21];
		rawBuf[0] = '<';
		rawBuf[1] = 0x12;
		rawBuf[2] = 21;
		rawBuf[3] = 't';
		for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

			rawBuf[(2 * i) + 4] = (byte)((arm.getJoint(i)->getTarget()) >> 8) & 0xFF;
			rawBuf[(2 * i) + 5] = (byte)(arm.getJoint(i)->getTarget()) & 0xFF;

		}
		rawBuf[20] = '>';
		for(int i=0; i<21; i++){
			Serial.write(rawBuf[i]);
		}
		break;
	}
	case 's':{
		uint8_t rawBuf[21];
		rawBuf[0] = '<';
		rawBuf[1] = 0x12;
		rawBuf[2] = 21;
		rawBuf[3] = 's';
		for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

			rawBuf[(2 * i) + 4] = (byte)((arm.getJoint(i)->getSpeed()) >> 8) & 0xFF;
			rawBuf[(2 * i) + 5] = (byte)(arm.getJoint(i)->getSpeed()) & 0xFF;

		}
		rawBuf[20] = '>';
		for(int i=0; i<21; i++){
			Serial.write(rawBuf[i]);
		}
		break;
	}
	}

}

void bootResponse(char* p){
	Serial.print(ARM_CONNECT_RESPONSE);
}

void controlCodes(char* p){

	switch(p[1]){


	case 'M':    // set mode (D, A, M = Drive, Arm, Mine)
		if (p[2] == 'D') {
			currentDriveMode = DRIVE;
			Serial.print("<DRIVE_MODE>");
		} else if (p[2] == 'A') {
			currentDriveMode = ARM;
			Serial.print("<ARM_MODE>");
		} else if (p[2] == 'M') {
			currentDriveMode = MINE;
			Serial.print("<MINE_MODE>");
		}
		break;

	case 'S':
		arm.saveCalibrations();
		break;
	case 'L':
		arm.loadCalibrations();
		break;
	case 'C':{
		//save position
		int a = atoi(p+2);
		arm.savePosition(a);
		break;
	}
	case 'V':{
		//load position
		int a = atoi(p+2);
		arm.gotoPosition(a);
		break;
	}
	case 'A':
		arm.attachAll();
		break;
	case 'D':
		arm.detachAll();
		break;
	case 'X':
		arm.stop();
		break;
	case 'P':
		//power up
		arm.detachAll();
		delay(10);
		digitalWrite((byte)SERVO_POWER_PIN, HIGH);
		delay(10);
		arm.init();   // 2 seconds of blocking delay !!!!
		break;
	case 'p':
		// power down
		arm.detachAll();
		delay(10);
		digitalWrite((byte)SERVO_POWER_PIN, LOW);
		delay(10);
		break;
	}

}


void setTarget(char *p) {
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		int targ = atoi((const char*) (p + 1));
		arm.getJoint(jointIndex)->setTarget(targ);
	}
}

void setAngle(char *p) {
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		int targ = atoi((const char*) (p + 1));
		float flargRad = (float)targ * PI / 180;
		arm.getJoint(jointIndex)->setTargetAngle(flargRad);
	}
}

void setSpeed(char *p) {
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		int spd = atoi((const char*) (p + 1));
		arm.getJoint(jointIndex)->setSpeed(spd);
	}
}

void useStick(char *p) {
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		int stickPos = atoi((const char*) (p + 1));
		arm.getJoint(jointIndex)->useStick(stickPos);
	}
}

void followStick(char *p) {
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		int stickPos = atoi((const char*) (p + 1));
		arm.getJoint(jointIndex)->followTheStick(stickPos);
	}
}

void moveToPosition(char *p) {
	int position = atoi((const char*) p);
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		arm.getJoint(jointIndex)->moveToImmediate(position);
		// jointIndex = -1;   // comment this line to allow run-on commands
	}
}


void sendRawArmData(){

	uint8_t rawBuf[19];
	rawBuf[0] = '<';
	rawBuf[1] = 'p';
	for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

		rawBuf[(2*i)+2] = ((arm.getJoint(i)->getPosition()) >> 8) & 0xFF;
		rawBuf[(2*i)+3] = (arm.getJoint(i)->getPosition())  & 0xFF;

	}
	rawBuf[18] = '>';

	for(int i = 0; i < 19; i++){
		Serial.write(rawBuf[i]);
	}

}




