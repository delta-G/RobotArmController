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

#include "ArmClass.h"

#define NUMBER_OF_JOINTS 8


enum State_Enum {READY, MOVING} state;

void Arm_Class::run() {

	State_Enum newState = state;

	switch (state) {
	case READY:
		for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
			if (joints[i].isMoving()) {
				newState = MOVING;
			}
		}
		break;
	case MOVING:
		boolean done = true;
		for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
			if (joints[i].isMoving()) {
				done = false;
			}
		}
		if (done) {
			newState = READY;
		}
		break;

	}  // end switch

	if(newState != state){
		if(newState == READY){
			Serial.print("<ARM_READY>");
		}
	}

	state = newState;

	for (int i = 0; i < numJoints; i++) {
		joints[i].run();
	}
}




Arm_Class::Arm_Class(){

	joints = 0;
	numJoints = NUMBER_OF_JOINTS;

}

Arm_Class::Arm_Class(Joint* aJoints, int aNum){
	joints = aJoints;
	numJoints = aNum;
}


void Arm_Class::addJoint(int i, Joint j){
	if((i >=0)  && (i < numJoints)){
		joints[i] = j;
	}
}

//  This has 2 seconds of delay in it for those who are counting.
void Arm_Class::init(){
//	loadAll(EEPROM_INITIAL_STATES);
	for(int i = 0; i < numJoints; i++){
		joints[i].init();
		delay(250);
	}
}

void Arm_Class::attachAll() {
	for (int i = 0; i < numJoints; i++) {
		joints[i].attach(joints[i].getPin());
		delay(250);
	}
}

void Arm_Class::detachAll() {
	for (int i = 0; i < numJoints; i++) {
		joints[i].detach();
		delay(250);
	}
}

void Arm_Class::stop() {
	for (int i = 0; i < numJoints; i++) {
			joints[i].stop();
		}
}

boolean Arm_Class::isMoving(){

	for(int i = 0; i< numJoints; i++){
		if(joints[i].isMoving()){
			return true;
		}
	}
	return false;
}



int Arm_Class::savePosition(int aAddress){
	int add = (aAddress >= 32)? aAddress : EEPROM_POSITION(aAddress);
	int offset = 0;
	for(int i = 0; i < numJoints; i++){
		int p = joints[i].getPosition();
		offset += writeToEEPROM(add + offset, p );
	}
	return offset;
}


//  Sends Robot to a position with the speed values it already has.
int Arm_Class::gotoPosition(int aAddress){


	int add = (aAddress >= 32)? aAddress : EEPROM_POSITION(aAddress);
	int offset = 0;
	int t;

	for(int i = 0; i < numJoints; i++){
		offset += readFromEEPROM(add + offset, t);
		joints[i].setTarget(t);
	}
	return offset;
}


//  Sets both the target and the position and speed.
//  It is the responsibility of the caller to make
//  sure that the arm gets to this position before
//  another is fed.

int Arm_Class::loadMovement(int aAddress) {
	int a = 0;
	int t;
	int s;

	for (int i = 0; i < numJoints; i++) {
		a += readFromEEPROM(aAddress + a, t);
		a += readFromEEPROM(aAddress + a, s);

		joints[i].setTarget(t);
		joints[i].setSpeed(s);
	}
	return a;
}

int Arm_Class::loadAll(int aAdress){
	int x = loadStates(aAdress);
	loadCalibrations();
	return x;
}

int Arm_Class::saveAll(int aAdress){
	int x = saveStates(aAdress);
	saveCalibrations();
	return x;
}

int Arm_Class::saveStates(int aAddress){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].saveState(aAddress + add);
	}
	return add;
}

int Arm_Class::loadStates(int aAddress){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].recallState(aAddress + add);
	}
	return add;
}

int Arm_Class::saveCalibrations(){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].saveCalibration(EEPROM_CALIBRATION_START + add);
	}
	byte flags = EEPROM.read(EEPROM_FLAG_BYTE);
	flags &= ~FLAG_CALIBRATIONS_SAVED;  // flags are set as 0 since cleared EEPROM is 0xFF
	EEPROM.write(1, flags);
	return add;
}

int Arm_Class::loadCalibrations() {

	int add = 0;
	byte flags = EEPROM.read(EEPROM_FLAG_BYTE);
	flags = ~flags;  // flags are set as 0 since cleared EEPROM is 0xFF
	if (flags & FLAG_CALIBRATIONS_SAVED) {
		for (int i = 0; i < numJoints; i++) {
			add += joints[i].loadCalibration(EEPROM_CALIBRATION_START + add);
		}
	}

	return add;
}




XYandAngle Arm_Class::findEndEffector(){

	// Location of base for this xperiment is the
	// pivot of the base servo.

	// This xperiment is just the 2D plane of the arm, so
	// we can locate the pivot point of the shoulder by just
	// adding to our Y variable (Z direction in 3D)

	XYpoint basePoint = {0,0};

	XYpoint shoulderPoint = {basePoint.x , basePoint.y + 37};

	XYandAngle elbowPoint = joints[1].findEndXY(shoulderPoint, 1.5708);

	XYandAngle wristPoint = joints[2].findEndXY(elbowPoint);

	XYandAngle gripperTip = joints[3].findEndXY(wristPoint);


	Serial.print("<");

	Serial.print("EB ");
	elbowPoint.printOut();

	Serial.print(" WR ");
	wristPoint.printOut();

	Serial.print(" GT ");
	gripperTip.printOut();

	Serial.print(">");

	return gripperTip;
}



