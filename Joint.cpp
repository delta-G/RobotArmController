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

#include "Joint.h"

Joint::Joint(char* aName, uint8_t aPin, uint16_t aPos) {
	name = aName;
	pin = aPin;
	position = aPos;
	target = aPos;
	speed = 5;
	minPos=544;
	maxPos=2400;
//	write(aPos);   Shouldn't write anything before we have hardware ready  this is probably why it jerks on startup

	max_refresh_rate = 100;
}

void Joint::init(){
	attach(pin);
}

void Joint::moveToImmediate(uint16_t aPos) {
	if(aPos<minPos) aPos = minPos;
	if(aPos>maxPos) aPos = maxPos;
	write(aPos);
	position = aPos;
	target = position;
}

uint8_t Joint::getPin(){
	return pin;
}

uint16_t Joint::getPosition(){
	return position;
}

boolean Joint::onTarget(){
	return position == target;
}


char* Joint::getName(){
	return name;
}

void Joint::setTarget(uint16_t aTarget){
	if(aTarget < minPos) aTarget = minPos;
	if(aTarget > maxPos) aTarget = maxPos;
	target = aTarget;
}

void Joint::setTarget(uint16_t aTarget, uint16_t aSpeed){
	if(aTarget < minPos) aTarget = minPos;
	if(aTarget > maxPos) aTarget = maxPos;
	target = aTarget;
	setSpeed(aSpeed);
}

void Joint::setSpeed(uint16_t aSpeed){
	speed = aSpeed;
}

void Joint::stop() {
	target = position;
}

boolean Joint::run() {

	static unsigned long prev = millis();
	unsigned long cur = millis();
	unsigned long deltaTime = cur - prev;
	prev = cur;

	if (position != target) {

		//  Speed in us pulse time per ms of real time
		//  What would be the max?
		unsigned long deltaPulse = deltaTime * speed;
		uint16_t deltaTarget = position - target;
		if(abs(deltaTarget) < deltaPulse) deltaPulse = abs(deltaTarget);

		if(target < position){
			position -= deltaPulse;
		}
		else {
			position += deltaPulse;
		}
		if(position < minPos) position = minPos;
		if(position > maxPos) position = maxPos;
	}
	write(position);
	return (position == target);
}


//  Saves 5 ints for a total of 10 bytes.
void Joint::saveState(int aAddress){

	int add = aAddress;
	add += writeToEEPROM(add, position);
	add += writeToEEPROM(add, target);
	add += writeToEEPROM(add, speed);
	add += writeToEEPROM(add, minPos);
	add += writeToEEPROM(add, maxPos);

}


void Joint::recallState(int aAddress){

	int add = aAddress;
	add += readFromEEPROM(add, position);
	add += readFromEEPROM(add, target);
	add += readFromEEPROM(add, speed);
	add += readFromEEPROM(add, minPos);
	add += readFromEEPROM(add, maxPos);

}




void Joint::followTheStick(int aReading){

	int res = map(aReading, -32768, 32767, minPos, maxPos);
	setTarget(res);

}


void Joint::useStick(int aReading){


	//  **TODO
	/*
	 *
	 * We need for this to affect the speed variable and we need min and max
	 * speeds.  It's OK if they are calculated in function but we need to
	 * think about how we're going to do that.
	 *
	 * We could even use the method from DiscoBot where we use the
	 * known time dif but I'd rather not rely on that when we have
	 * unknowns about transmission speeds.
	 *
	 *
	 */



}
