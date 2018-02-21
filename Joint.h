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


class Joint : public Servo {

private:

	uint8_t pin;
	uint16_t position;
	uint16_t target;
	uint16_t speed;
	uint16_t minPos;
	uint16_t maxPos;

	uint16_t max_refresh_rate;

	char* name;

public:


	Joint(char* name, uint8_t aPin, uint16_t aPos);

	void init();

	void moveToImmediate(uint16_t aPos);

	uint8_t getPin();
	uint16_t getPosition();
	char* getName();

	boolean onTarget();

	void setTarget(uint16_t);
	void setTarget(uint16_t, uint16_t);
	void setSpeed(uint16_t);

	void stop();

	boolean run();

	void saveState(int);
	void recallState(int);

	void followTheStick(int);
	void useStick(int);


};





#endif /* JOINT_H_ */
