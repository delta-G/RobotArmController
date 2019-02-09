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

void Arm_Class::init(){
	for(int i = 0; i < numJoints; i++){
		joints[i].init();
		joints[i].recallState((i*6)+EEPROM_INITIAL_STATES);
		joints[i].run();
	}
}


void Arm_Class::run(){
	for(int i = 0; i < numJoints; i++){
			joints[i].run();
		}
}



int Arm_Class::savePosition(int aAddress){
	int add = 0;
	for(int i = 0; i < numJoints; i++){
		int p = joints[i].getPosition();
		add += writeToEEPROM(aAddress + add, p );
	}
	return add;
}

int Arm_Class::gotoPosition(int aAddress){

	int a = 0;
	int t;

	for(int i = 0; i < numJoints; i++){
		a += readFromEEPROM(aAddress + a, t);
		joints[i].setTarget(t);
	}
	return a;
}

int Arm_Class::saveAllStates(int aAddress){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].saveState(aAddress + add);

	}
	return add;
}

int Arm_Class::getAllStates(int aAddress){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].recallState(aAddress + add);

	}
	return add;
}

int Arm_Class::saveCalibrations(int aAddress){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].saveCalibration(aAddress + add);

	}
	return add;
}

int Arm_Class::loadCalibrations(int aAddress){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].loadCalibration(aAddress + add);

	}
	return add;
}



