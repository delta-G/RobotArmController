/*
 * Joint.cpp
 *
 *  Created on: Jun 23, 2016
 *      Author: david
 */

#include "Joint.h"

Joint::Joint(char* aName, uint8_t aPin, uint16_t aPos) {
	name = aName;
	pin = aPin;
	position = aPos;
	write(aPos);
}

void Joint::moveToImmediate(uint16_t aPos) {
	write(aPos);
	position = aPos;
}

uint8_t Joint::getPin(){
	return pin;
}

uint16_t Joint::getPosition(){
	return position;
}

char* Joint::getName(){
	return name;
}
