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


/*
 *
 *
 * Let's move the 0,0,0 point to the floor under the bot so the bot is
 * entirely in positive space.
 *
 *
 */

#ifndef NOGOZONES_H_
#define NOGOZONES_H_

#include "Arduino.h"


#define MAIN_DECK_HEIGHT 115
#define MAIN_DECK_FRONT 127
#define MAIN_DECK_BACK -127
#define MAIN_DECK_LEFT -140
#define MAIN_DECK_RIGHT 140


#define TURRET_HEIGHT 115
#define TURRET_FRONT 127
#define TURRET_BACK -127
#define TURRET_LEFT -140
#define TURRET_RIGHT 140


// return true if the xyz is outside of the no-go zone.
boolean goCheckMainDeck(int x, int y, int z);

boolean goCheckTurret(int x, int y, int z);


boolean goCheck(int x, int y, int z);

#endif /* NOGOZONES_H_ */
