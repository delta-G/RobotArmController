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

#include "ControllerFunctions.h"

XboxHandler* xbox_ptr;

Arm_Class* arm_ptr;

extern DriveModeEnum currentDriveMode;

boolean invertWrist = true;
boolean invertElbow = false;
boolean invertShoulder = false;


void initControllerFunctions(Arm_Class* aArm, XboxHandler* aXbox){
	arm_ptr = aArm;
	xbox_ptr = aXbox;
}


void mainControllerLoop() {

	if (xbox_ptr->newDataAvailable()) {
		if(currentDriveMode == ARM){
			rawMode();
		}
	}
}

void rawMode() {

	static boolean followTrigMode = false;

	int16_t rotateVal = xbox_ptr->getHatValue(RightHatX);
	int16_t wristVal = xbox_ptr->getHatValue(RightHatY);
	int16_t shoulderVal = xbox_ptr->getHatValue(LeftHatX);
	int16_t elbowVal = xbox_ptr->getHatValue(LeftHatY);

	//  Don't mess with the D-pad, Robot is using it to drive
	arm_ptr->getJoint(ROTATE)->useStick(rotateVal);
	arm_ptr->getJoint(WRIST)->useStick(invertWrist? -wristVal : wristVal);
	arm_ptr->getJoint(SHOULDER)->useStick(invertShoulder? -shoulderVal : shoulderVal);
	arm_ptr->getJoint(ELBOW)->useStick(invertElbow? -elbowVal : elbowVal);

	if(xbox_ptr->isPressed(L1)){
		arm_ptr->getJoint(BASE)->advance(200);
	}
	else if (xbox_ptr->isPressed(R1)){
		arm_ptr->getJoint(BASE)->advance(-200);
	}
	if (xbox_ptr->isClicked(R3)){
		invertElbow = !invertElbow;
		invertWrist = !invertWrist;
	}
	if (xbox_ptr->isClicked(L3)){
		invertShoulder = !invertShoulder;
	}

	if (xbox_ptr->isClicked(X)){
		followTrigMode = !followTrigMode;
		arm_ptr->getJoint(GRIP)->stop();
	}

	int16_t gripVal = 0;

	if (followTrigMode) {
		gripVal = (xbox_ptr->getTriggerValue(R2) - 128) * 255;
		arm_ptr->getJoint(GRIP)->followTheStick(gripVal);
	} else {
		gripVal = (xbox_ptr->getTriggerValue(R2) - xbox_ptr->getTriggerValue(L2))* 128;
		arm_ptr->getJoint(GRIP)->useStick(gripVal);
	}


}

