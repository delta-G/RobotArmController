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

#ifndef ARMCLASS_H_
#define ARMCLASS_H_

#include "Arduino.h"
#include "Defines.h"
#include <Joint.h>
#include <Gimbal.h>

#include "SpacePoint.h"

#include <EepromFuncs.h>


struct ArmPositionStruct {

	SpacePoint basePoint;
	SpacePoint shoulderPoint;
	SpacePoint elbowPoint;
	SpacePoint wristPoint;
	SpacePoint gripperTip;

};

struct JointSpaceStruct {

	float baseAngle;
	float shoulderAngle;
	float elbowAngle;
	float wristAngle;

};

enum State_Enum {READY, MOVING};

class Arm_Class {

private:

	Joint* joints;
	int numJoints;

	boolean servoPower;



public:

	State_Enum state;

	Arm_Class();
	Arm_Class(Joint*, int);

	int getNumJoints();
	Joint* getJoint(unsigned int);

	void addJoint(int, Joint);
	void init();
	void detachAll();
	void attachAll();
	void setServoPower(boolean);
	void run();
	void stop();
	boolean isMoving();

	boolean (*movementDoneCallback)();

//	boolean *(movementDoneCallback)();
	void setCallback(boolean (*aCallback)());

	int savePosition(int);
//	int readPosition(int);

	int saveAll(int);
	int loadAll(int);
	int saveStates(int);
	int loadStates(int);
	int saveCalibrations();
	int loadCalibrations();

	int gotoPosition(int);
	int loadMovement(int);

	XYandAngle findEndEffector();

	ArmPositionStruct currentPositions;
	boolean positionValid;
	void invalidatePosition();
	void findPosition();

	uint8_t getStatusByte();

	ArmPositionStruct jointSpaceToPosition(JointSpaceStruct);


	JointSpaceStruct gripperPositionToJointSpace(SpacePoint, float);

};







#endif /* ARMCLASS_H_ */
