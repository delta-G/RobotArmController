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

#ifndef COMMANDFUNCTIONS_H_
#define COMMANDFUNCTIONS_H_

#include <Arduino.h>
#include <CommandParser.h>
#include <XboxHandler.h>

#include "Defines.h"
#include "ArmClass.h"

void startCommands(char* p);
void setJointIndex(char* p);
void requestFromArm(char *p);
void bootResponse(char* p);
void setTarget(char* p);
void setAngle(char* p);
void setSpeed(char* p);
void useStick(char *p);
void followStick(char *p);
void controlCodes(char* p);
void moveToPosition(char* p);

void xboxCommand(char* p);

void sendRawArmData();



#endif /* COMMANDFUNCTIONS_H_ */
