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

#ifndef JOINT_H_
#define JOINT_H_

#include "Arduino.h"
#include <Servo.h>
#include <EepromFuncs.h>
#include "Defines.h"

#include "ServoCalibration.h"
#include "SpacePoint.h"

XYpoint solveTriangle (float aAngle, uint16_t aLength);

class Joint : public Servo {

private:

	uint8_t pin;
	uint16_t position;
	uint16_t target;
	uint16_t speed;   /// us of change in pulse width per second

	boolean moving;

	uint16_t length;  // should be integer mm.
	uint16_t offset;  // mm in vertical offset

	uint32_t lastStickUpdate;
	uint32_t lastRunTime;

	ServoCalibrationStruct calibration;

	uint16_t max_refresh_rate;

	char* name;

public:


	Joint(char* name, uint8_t aPin, uint16_t aPos);
	Joint(char* name, uint8_t aPin, uint16_t aPos, uint16_t aLength, uint16_t aMinMicros, float aMinAngle, uint16_t aMaxMicros, float aMaxAngle);
	Joint(char* name, uint8_t aPin, uint16_t aPos, uint16_t aLength, uint16_t aOffset, uint16_t aMinMicros, float aMinAngle, uint16_t aMaxMicros, float aMaxAngle);

	void init();

	void moveToImmediate(uint16_t);
	void moveToImmediateAngle(float);

	uint8_t getPin();
	uint16_t getPosition();
	char* getName();

	uint16_t getLength();
	void setLength(uint16_t);

	boolean isMoving();

	uint16_t setTarget(uint16_t);
	uint16_t setTarget(uint16_t, uint16_t);
	uint16_t getTarget();
	void setSpeed(uint16_t);
	uint16_t getSpeed();

	float getAngle();
	float setTargetAngle(float);
	float setTargetAngle(float, uint16_t);


	void stop();

	boolean run();

	int saveCalibration(int);
	int loadCalibration(int);
	int saveState(int);
	int recallState(int);

	void followTheStick(int);
	void useStick(int);

	XYandAngle findEndXY(XYpoint aPivot, float aAngle);
	XYandAngle findEndXY(XYandAngle aPivot);
	XYandAngle findEndXY(XYandAngle aPivot, float aPosition);

};





#endif /* JOINT_H_ */
