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
		{ 'G', gimbalCommand },
		{ '#', moveToPosition }
};


CommandParser cp(&commands[0], NUM_ELEMENTS(commands), false);

void gimbalCommand(char *p) {
	switch (p[1]) {
	case 'C':
		if (p[2] == 'T') {
			int targ = atoi((const char*) (p + 3));
			gimbal.setCenter(gimbal.getCenterPan(), targ);
		} else if (p[2] == 'P') {
			int targ = atoi((const char*) (p + 3));
			gimbal.setCenter(targ, gimbal.getCenterTilt());
		} else {
			gimbal.setCenter();
		}
		break;
	case 'c':
		gimbal.gotoCenter();
		break;
	default:
		break;

	}
}

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


enum reportTypeEnum {SPEED,	TARGET,	POSITION};

void requestFromArm(char *p) {
	static uint8_t lastPositionReport[22] = "";
	static uint8_t lastTargetReport[22] = "";
	static uint8_t lastSpeedReport[22] = "";

	static reportTypeEnum lastReport = POSITION;

	boolean fallingThrough = false;

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

	case 'R': {
		fallingThrough = true;
	}
	/* no break */
	case 's': {
		//  Speed won't run if it or target was last to prevent it and target from just taking turns
		//  Speed shouldn't be changing a lot.
		if (!fallingThrough || (fallingThrough && (lastReport == POSITION))) {
			uint8_t rawBuf[22];
			rawBuf[0] = '<';
			rawBuf[1] = 0x12;
			rawBuf[2] = 22;
			rawBuf[3] = arm.getStatusByte();
			rawBuf[4] = 's';
			for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

				rawBuf[(2 * i) + 5] = (byte)((arm.getJoint(i)->getSpeed()) >> 8)
						& 0xFF;
				rawBuf[(2 * i) + 6] = (byte)(arm.getJoint(i)->getSpeed())
						& 0xFF;

			}
			uint16_t pan = gimbal.getPanJoint()->getSpeed();
			uint16_t tilt = gimbal.getTiltJoint()->getSpeed();
			rawBuf[17] = pan >> 8;
			rawBuf[18] = pan & 0xFF;
			rawBuf[19] = tilt >> 8;
			rawBuf[20] = tilt & 0xFF;
			rawBuf[21] = '>';
			if (!fallingThrough || (memcmp(lastSpeedReport, rawBuf, 22))) {
				memcpy(lastSpeedReport, rawBuf, 22);
				for (int i = 0; i < 22; i++) {
					Serial.write(rawBuf[i]);
				}
				fallingThrough = false;
				lastReport = SPEED;
			}
		}
		if (!fallingThrough) {
			break;
		}
	}
		/* no break */

	case 't': {
		// Target will run if it wasn't the last one.
		if (!fallingThrough || (fallingThrough && (lastReport != TARGET))) {
			uint8_t rawBuf[22];
			rawBuf[0] = '<';
			rawBuf[1] = 0x12;
			rawBuf[2] = 22;
			rawBuf[3] = arm.getStatusByte();
			rawBuf[4] = 't';
			for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

				rawBuf[(2 * i) + 5] = (byte)(
						(arm.getJoint(i)->getTarget()) >> 8) & 0xFF;
				rawBuf[(2 * i) + 6] = (byte)(arm.getJoint(i)->getTarget())
						& 0xFF;

			}
			uint16_t pan = gimbal.getPanJoint()->getTarget();
			uint16_t tilt = gimbal.getTiltJoint()->getTarget();
			rawBuf[17] = pan >> 8;
			rawBuf[18] = pan & 0xFF;
			rawBuf[19] = tilt >> 8;
			rawBuf[20] = tilt & 0xFF;
			rawBuf[21] = '>';
			if (!fallingThrough || (memcmp(lastTargetReport, rawBuf, 22))) {
				memcpy(lastTargetReport, rawBuf, 22);
				for (int i = 0; i < 22; i++) {
					Serial.write(rawBuf[i]);
				}
				fallingThrough = false;
				lastReport = TARGET;
			}
		}
		if (!fallingThrough) {
			break;
		}
	}
		/* no break */

	case 'p': {
		uint8_t rawBuf[22];
		rawBuf[0] = '<';
		rawBuf[1] = 0x12;
		rawBuf[2] = 22;
		rawBuf[3] = arm.getStatusByte();
		rawBuf[4] = 'p';
		for (uint8_t i = 0; i < arm.getNumJoints(); i++) {

			rawBuf[(2 * i) + 5] = (byte) ((arm.getJoint(i)->getPosition()) >> 8)
					& 0xFF;
			rawBuf[(2 * i) + 6] = (byte) (arm.getJoint(i)->getPosition())
					& 0xFF;

		}
		uint16_t pan = gimbal.getPanJoint()->getPosition();
		uint16_t tilt = gimbal.getTiltJoint()->getPosition();
		rawBuf[17] = pan >> 8;
		rawBuf[18] = pan & 0xFF;
		rawBuf[19] = tilt >> 8;
		rawBuf[20] = tilt & 0xFF;

		rawBuf[21] = '>';
		if (!fallingThrough || (memcmp(lastPositionReport, rawBuf, 22))) {
			memcpy(lastPositionReport, rawBuf, 22);
			for (int i = 0; i < 22; i++) {
				Serial.write(rawBuf[i]);
			}
			fallingThrough = false;
			lastReport = POSITION;
		}
		// Selection was R and we fell through to here with no data
		if (fallingThrough) {
			Serial.print(ARM_NO_NEW_DATA);
			// So we don't get stuck because only targets or speeds are changing
			lastReport = POSITION;  // The only one that can go twice in a row.
		}
		break;
	}

	case 'c': {
		uint8_t numBytes = 76;
		uint8_t rawBuf[numBytes];
		rawBuf[0] = '<';
		rawBuf[1] = 0x12;
		rawBuf[2] = numBytes;
		for (uint8_t i = 0; i < arm.getNumJoints(); i++) {
			ServoCalibrationStruct cs = arm.getJoint(i)->getCalibrationStruct();
			memcpy(rawBuf + (12 * i) + 3, &(cs.minimumAngle), 4);
			memcpy(rawBuf + (12 * i) + 7, &(cs.maximumAngle), 4);
			memcpy(rawBuf + (12 * i) + 11, &(cs.minimumMicros), 2);
			memcpy(rawBuf + (12 * i) + 13, &(cs.maximumMicros), 2);
		}
		rawBuf[75] = '>';
		for (uint8_t i = 0; i < numBytes; i++) {
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
//			Serial.print("<DRIVE_MODE>");
		} else if (p[2] == 'A') {
			currentDriveMode = ARM;
//			Serial.print("<ARM_MODE>");
		} else if (p[2] == 'M') {
			currentDriveMode = MINE;
//			Serial.print("<MINE_MODE>");
		} else if (p[2] == 'T') {
			currentDriveMode = AUTO;
//			Serial.print("<MINE_MODE>");
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
	case 'c': {
		//save position
		int a = atoi(p + 2);
		arm.saveStates(a);
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
	case 'Q':
		// Park Arm
		arm.setCallback(parkArm);
		arm.gotoPosition(EEPROM_POSITION_SITTING);
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
		gimbal.getPanJoint()->followTheStick(stickPos, gimbal.getCenterPan());
	}
	else if(jointIndex == arm.getNumJoints() +1){
		gimbal.getTiltJoint()->followTheStick(stickPos, gimbal.getCenterTilt());
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


boolean parkArm(){

	///  Must start from sitting scorpion position
	//  Call that first then set this as movementDoneCallback
//	Serial.print("<PARKING>");
	if(arm.getJoint(ELBOW)->getPosition() == 2400){
//		Serial.print("<PARKING1>");
		if(!(gimbal.getTiltJoint()->getPosition() == 700)){
//			Serial.print("<PARKING1-1>");
			gimbal.getTiltJoint()->setTarget(700, 500);
			return false;
		}
		if (!(gimbal.getPanJoint()->getPosition() == 1420)) {
			//			Serial.print("<PARKING1-1>");
			gimbal.getPanJoint()->setTarget(1420, 500);
			return false;
		}
		if (!(arm.getJoint(ROTATE)->getPosition() == 700)) {
//			Serial.print("<PARKING1-2>");
			arm.getJoint(ROTATE)->setTarget(700,500);
			return false;
		}
		if(!(arm.getJoint(BASE)->getPosition() == 1340)){
//			Serial.print("<PARKING1-3>");
			arm.getJoint(BASE)->setTarget(1340, 300);
			arm.getJoint(WRIST)->setTarget(1200, 350);
			return false;
		}
		if(!(arm.getJoint(WRIST)->getPosition() == 1200)){
//			Serial.print("<PARKING1-3a>");
			arm.getJoint(WRIST)->setTarget(1200, 350);
			return false;
		}
//		Serial.print("<PARKING1-END>");
		arm.getJoint(ELBOW)->setTarget(1800, 200);
		arm.getJoint(SHOULDER)->setTarget(1600, 200);
		return false;
	}
	if(arm.getJoint(ELBOW)->getPosition() == 1800) {
//		Serial.print("<PARKING2>");
		arm.getJoint(ELBOW)->setTarget(2000, 100);
		arm.getJoint(SHOULDER)->setTarget(1350, 100);
		return false;
	}
	if(arm.getJoint(SHOULDER)->getPosition() == 1350){
//		Serial.print("<PARKING3>");
		arm.getJoint(SHOULDER)->setTarget(1250, 50);
		return false;
	}
	if (arm.getJoint(SHOULDER)->getPosition() == 1250) {
		//		Serial.print("<PARKING4>");
		arm.getJoint(SHOULDER)->setTarget(1150, 50);
		return false;
	}
	if(arm.getJoint(SHOULDER)->getPosition() == 1150){
//		Serial.print("<PARKING - DETACH>");
		arm.detachAll();
		gimbal.detach();
		delay(10);
		arm.setServoPower(false);
		delay(10);
		return true;
	}
	Serial.print("<PARK FAILED>");
	arm.gotoPosition(EEPROM_POSITION_SITTING);
	return false;
}

