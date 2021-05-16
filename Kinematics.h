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

#ifndef KINEMATICS_H_
#define KINEMATICS_H_



#include "Arduino.h"
#include "ArmClass.h"

struct PositionStruct{

	int16_t x;
	int16_t y;
	int16_t z;
	float phi;

	PositionStruct(int16_t aX, int16_t aY, int16_t aZ, float aPhi):x(aX), y(aY), z(aZ), phi(aPhi){}

	boolean compare(PositionStruct other, uint16_t tolerance);
};

boolean runInverse(float *aReturnArray, int16_t aX, int16_t aY, int16_t aZ, float aPhi, uint16_t aTolerance);
PositionStruct forwardK1(float baseAngle, float shoulderAngle, float elbowAngle, float wristAngle);
void inverseK1(float *aReturnArray, int16_t aX, int16_t aY, int16_t aZ, float aPhi);


#endif /* KINEMATICS_H_ */
