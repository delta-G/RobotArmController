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
	if((i >=0)  && (i < NUMBER_OF_JOINTS)){
		joints[i] = j;
	}
}

void Arm_Class::init(){
	for(int i = 0; i < NUMBER_OF_JOINTS; i++){
		joints[i].init();
	}
}


void Arm_Class::run(){
	for(int i = 0; i < NUMBER_OF_JOINTS; i++){
			joints[i].run();
		}
}
