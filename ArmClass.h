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
#include "Joint.h"
#include <EepromFuncs.h>

#define EEPROM_START 24
#define EEPROM_INITIAL_STATES 48
#define EEPROM_POSITION_STANDING 128
#define EEPROM_POSITION_SITTING 144


class Arm_Class {

private:

	Joint* joints;
	int numJoints;



public:

	Arm_Class();
	Arm_Class(Joint*, int);
	void addJoint(int, Joint);
	void init();
	void run();
	void stop();
	boolean isMoving();

	int savePosition(int);
//	int readPosition(int);

	int saveAll(int);
	int loadAll(int);
	int saveStates(int);
	int loadStates(int);
	int saveCalibrations(int);
	int loadCalibrations(int);

	int gotoPosition(int);
	int loadMovement(int);

};





#endif /* ARMCLASS_H_ */
