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

#ifndef _RobotArmController_H_
#define _RobotArmController_H_
#include "Arduino.h"

#include <Servo.h>

#include "Joint.h"

#define SER_BAUD 115200
#define PROBLEM_LED A0
#define HEARTLED 13

#define MAX_COMMAND_LENGTH 20
#define START_OP '<'
#define END_OP '>'
#define SEPERATOR ','

enum JointsE {
	BASE, SHOULDER, ELBOW, WRIST, ROTATE, GRIP, PAN, TILT, NUMBER_OF_JOINTS
};

void setup();
void loop();
void heartbeat();

void parseCommand();

#endif /* _RobotArmController_H_ */
