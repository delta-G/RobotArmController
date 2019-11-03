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
extern GimbalClass gimbal;
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
		{ 'a', advanceJoint },
		{ 'J', useStick },
		{ 'F', followStick },
		{ 'C', controlCodes },
		{ '#', moveToPosition }
};


CommandParser cp(&commands[0], NUM_ELEMENTS(commands), false);

void xboxCommand(char* p){
	xbox.handleIncomingASCII(p+1);
}

void xboxCommandRaw(char* p) {
	if(p[1] == 0x14 && p[2] == 16){
		// It's a real raw xbox command from Base
		uint8_t temp[14];
		memcpy(temp, p+1, 14);
		temp[1] = 0x0D;  // xboxHandler Expects this
		xbox.handleIncoming(temp);
	}
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
			Serial.print("<ARMGIT-");
			Serial.print(gitbuf);
			Serial.print(">");
			break;
		}

	case 'p': {
		uint8_t rawBuf[22];
		rawBuf[0] = '<';
		rawBuf[1] = 0x12;
		rawBuf[2] = 22;
		rawBuf[3] = arm.getStatusByte();
		rawBuf[4] = 'p';
		for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

			rawBuf[(2 * i) + 5] = (byte)((arm.getJoint(i)->getPosition()) >> 8) & 0xFF;
			rawBuf[(2 * i) + 6] = (byte)(arm.getJoint(i)->getPosition()) & 0xFF;

		}
		uint16_t pan = gimbal.getPanJoint()->getPosition();
		uint16_t tilt = gimbal.getTiltJoint()->getPosition();
		rawBuf[17] = pan >> 8;
		rawBuf[18] = pan & 0xFF;
		rawBuf[19] = tilt >> 8;
		rawBuf[20] = tilt & 0xFF;

		rawBuf[21] = '>';
		for(int i=0; i<22; i++){
			Serial.write(rawBuf[i]);
		}
		break;
	}
	case 't':{
		uint8_t rawBuf[22];
		rawBuf[0] = '<';
		rawBuf[1] = 0x12;
		rawBuf[2] = 22;
		rawBuf[3] = arm.getStatusByte();
		rawBuf[4] = 't';
		for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

			rawBuf[(2 * i) + 5] = (byte) ((arm.getJoint(i)->getTarget()) >> 8)
					& 0xFF;
			rawBuf[(2 * i) + 6] = (byte) (arm.getJoint(i)->getTarget()) & 0xFF;

		}
		uint16_t pan = gimbal.getPanJoint()->getTarget();
		uint16_t tilt = gimbal.getTiltJoint()->getTarget();
		rawBuf[17] = pan >> 8;
		rawBuf[18] = pan & 0xFF;
		rawBuf[19] = tilt >> 8;
		rawBuf[20] = tilt & 0xFF;
		rawBuf[21] = '>';
		for (int i=0; i<22; i++){
			Serial.write(rawBuf[i]);
		}
		break;
	}
	case 's':{
		uint8_t rawBuf[22];
		rawBuf[0] = '<';
		rawBuf[1] = 0x12;
		rawBuf[2] = 22;
		rawBuf[3] = arm.getStatusByte();
		rawBuf[4] = 's';
		for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

			rawBuf[(2 * i) + 5] = (byte) ((arm.getJoint(i)->getSpeed()) >> 8)
					& 0xFF;
			rawBuf[(2 * i) + 6] = (byte) (arm.getJoint(i)->getSpeed()) & 0xFF;

		}
		uint16_t pan = gimbal.getPanJoint()->getSpeed();
		uint16_t tilt = gimbal.getTiltJoint()->getSpeed();
		rawBuf[17] = pan >> 8;
		rawBuf[18] = pan & 0xFF;
		rawBuf[19] = tilt >> 8;
		rawBuf[20] = tilt & 0xFF;
		rawBuf[21] = '>';
		for (int i = 0; i < 22; i++) {
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
		gimbal.saveCalibrations(
				EEPROM_CALIBRATION_START + EEPROM_GIMBAL_CALIBRATION_OFFSET);
		break;
	case 'L':{
		arm.loadCalibrations();
		byte flags = EEPROM.read(EEPROM_FLAG_BYTE);
		flags = ~flags;  // flags are set as 0 since cleared EEPROM is 0xFF
		if (flags & FLAG_CALIBRATIONS_SAVED) {
			gimbal.loadCalibrations(EEPROM_CALIBRATION_START + EEPROM_GIMBAL_CALIBRATION_OFFSET);
		}
		break;
	}
	case 'C': {
		//save position
		int a = atoi(p + 2);
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
		gimbal.detach();
		delay(10);
		arm.setServoPower(true);
		delay(10);
		arm.init();   // 2 seconds of blocking delay !!!!
		gimbal.init();
		break;
	case 'p':
		// power down
		arm.detachAll();
		gimbal.detach();
		delay(10);
		arm.setServoPower(false);
		delay(10);
		break;
	}

}


void setTarget(char *p) {
	int targ = atoi((const char*) (p + 1));
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		arm.getJoint(jointIndex)->setTarget(targ);
	}
	else if(jointIndex == arm.getNumJoints()){
		gimbal.getPanJoint()->setTarget(targ);
	}
	else if(jointIndex == arm.getNumJoints() +1){
		gimbal.getTiltJoint()->setTarget(targ);
	}
}

void setAngle(char *p) {
	int targ = atoi((const char*) (p + 1));
	float flargRad = (float)targ * PI / 180;
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		arm.getJoint(jointIndex)->setTargetAngle(flargRad);
	}
	else if(jointIndex == arm.getNumJoints()){
		gimbal.getPanJoint()->setTargetAngle(flargRad);
	}
	else if(jointIndex == arm.getNumJoints() +1){
		gimbal.getTiltJoint()->setTargetAngle(flargRad);
	}
}

void setSpeed(char *p) {
	int spd = atoi((const char*) (p + 1));
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		arm.getJoint(jointIndex)->setSpeed(spd);
	}
	else if(jointIndex == arm.getNumJoints()){
		gimbal.getPanJoint()->setSpeed(spd);
	}
	else if(jointIndex == arm.getNumJoints() +1){
		gimbal.getTiltJoint()->setSpeed(spd);
	}
}

void advanceJoint(char* p) {
	int mult = atoi((const char*) (p + 1));
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		arm.getJoint(jointIndex)->advance(mult);
	}
	else if(jointIndex == arm.getNumJoints()){
		gimbal.getPanJoint()->advance(mult);
	}
	else if(jointIndex == arm.getNumJoints() +1){
		gimbal.getTiltJoint()->advance(mult);
	}
}

void useStick(char *p) {
	int stickPos = atoi((const char*) (p + 1));
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		arm.getJoint(jointIndex)->useStick(stickPos);
	}
	else if(jointIndex == arm.getNumJoints()){
		gimbal.getPanJoint()->useStick(stickPos);
	}
	else if(jointIndex == arm.getNumJoints() +1){
		gimbal.getTiltJoint()->useStick(stickPos);
	}
}

void followStick(char *p) {
	int stickPos = atoi((const char*) (p + 1));
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		arm.getJoint(jointIndex)->followTheStick(stickPos);
	}
	else if(jointIndex == arm.getNumJoints()){
		gimbal.getPanJoint()->followTheStick(stickPos);
	}
	else if(jointIndex == arm.getNumJoints() +1){
		gimbal.getTiltJoint()->followTheStick(stickPos);
	}
}

void moveToPosition(char *p) {
	int position = atoi((const char*) p);
	if (jointIndex >= 0 && jointIndex < arm.getNumJoints()) {
		arm.getJoint(jointIndex)->moveToImmediate(position);
		// jointIndex = -1;   // comment this line to allow run-on commands
	}
	else if(jointIndex == arm.getNumJoints()){
		gimbal.getPanJoint()->moveToImmediate(position);
	}
	else if(jointIndex == arm.getNumJoints() +1){
		gimbal.getTiltJoint()->moveToImmediate(position);
	}
}



