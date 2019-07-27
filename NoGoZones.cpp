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


#include "NoGoZones.h"


// return true if the xyz is outside of the no-go zone.
boolean goCheckMainDeck(int x, int y, int z){

	//check against main deck:
	//  if above the deck it doesn't matter
	if(z < MAIN_DECK_HEIGHT){

		if(y < MAIN_DECK_FRONT && y > MAIN_DECK_BACK){
			if(x < MAIN_DECK_RIGHT && x > MAIN_DECK_LEFT){
				return false;
			}
		}
	}

	return true;

}

boolean goCheckTurret(int x, int y, int z){

	//check against main deck:
	//  if above the deck it doesn't matter
	if(z < TURRET_HEIGHT){

		if(y < TURRET_FRONT && y > TURRET_BACK){
			if(x < TURRET_RIGHT && x > TURRET_LEFT){
				return false;
			}
		}
	}

	return true;
}


boolean goCheck(int x, int y, int z){

	if(z < TURRET_HEIGHT){

			if(y < MAIN_DECK_FRONT && y > MAIN_DECK_BACK){
				if(x < MAIN_DECK_RIGHT && x > MAIN_DECK_LEFT){
					return false;
				}
			}
		}

		return true;

}
