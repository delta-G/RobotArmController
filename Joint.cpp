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
	speed = 100;

	length = 100;
	offset = 0;

	moving = false;
	calibration.calibrate(544, 0.0, 2400, 180.0);
	lastStickUpdate = millis();
//	write(aPos);   Shouldn't write anything before we have hardware ready  this is probably why it jerks on startup

	max_refresh_rate = 100;
}

Joint::Joint(char* aName, uint8_t aPin, uint16_t aPos, uint16_t aLength, uint16_t aMinMicros, float aMinAngle, uint16_t aMaxMicros, float aMaxAngle) {
	name = aName;
	pin = aPin;
	position = aPos;
	target = aPos;
	length = aLength;
	offset = 0;
	speed = 100;
	moving = false;
	lastStickUpdate = millis();
	calibration.calibrate(aMinMicros, aMinAngle, aMaxMicros, aMaxAngle);
//	write(aPos);   Shouldn't write anything before we have hardware ready  this is probably why it jerks on startup

	max_refresh_rate = 100;
}

Joint::Joint(char* aName, uint8_t aPin, uint16_t aPos, uint16_t aLength, uint16_t aOffset, uint16_t aMinMicros, float aMinAngle, uint16_t aMaxMicros, float aMaxAngle) {
	name = aName;
	pin = aPin;
	position = aPos;
	target = aPos;
	length = aLength;
	offset = aOffset;
	speed = 100;
	moving = false;
	lastStickUpdate = millis();
	calibration.calibrate(aMinMicros, aMinAngle, aMaxMicros, aMaxAngle);
//	write(aPos);   Shouldn't write anything before we have hardware ready  this is probably why it jerks on startup

	max_refresh_rate = 100;
}

void Joint::init(){
	attach(pin);
	moveToImmediate(position);
}

boolean Joint::isMoving(){
	return (position != target);
}

void Joint::moveToImmediate(uint16_t aPos) {
	aPos = calibration.constrainMicros(aPos);
	write(aPos);
	position = aPos;
	target = position;
	Serial.print("<MTI,");
	Serial.print(aPos);
	Serial.print(">");
}

void Joint::moveToImmediateAngle(float aAng) {
	aAng = calibration.constrainAngle(aAng);
	position = calibration.angleToMicros(aAng);
	write(position);
	target = position;
}

uint8_t Joint::getPin(){
	return pin;
}

uint16_t Joint::getPosition(){
	return position;
}

float Joint::getAngle() {
	return calibration.microsToAngle(getPosition());
}

char* Joint::getName(){
	return name;
}

uint16_t Joint::getLength(){
	return length;
}

void Joint::setLength(uint16_t aLength){
	length = aLength;
}

uint16_t Joint::setTarget(uint16_t aTarget){
	aTarget = calibration.constrainMicros(aTarget);
	target = aTarget;
	return target;
}

float Joint::setTargetAngle(float aAngle){
	aAngle = calibration.constrainAngle(aAngle);
	target = calibration.angleToMicros(aAngle);
	return calibration.microsToAngle(target); // just in case setting it changes things.
}

uint16_t Joint::setTarget(uint16_t aTarget, uint16_t aSpeed){
	aTarget = calibration.constrainMicros(aTarget);
	target = aTarget;
	setSpeed(aSpeed);
	return target;
}

float Joint::setTargetAngle(float aAngle, uint16_t aSpeed){
	aAngle = calibration.constrainAngle(aAngle);
	target = calibration.angleToMicros(aAngle);
	setSpeed(aSpeed);
	return calibration.microsToAngle(target); // just in case setting it changes things.
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
//	prev = cur;     //////////////////////////   This can't go here, it will never run...

	if (position != target) {

		if(!moving){
			prev = cur; // reset our timer for the new move
			moving = true;
			return false;  // bail out until next run
		}

		//  Speed in us pulse time per ms of real time
		//  What would be the max?
		unsigned long deltaPulse = deltaTime * speed / 1000;
//		uint16_t deltaTarget = position - target;
//		if(abs(deltaTarget) < deltaPulse) deltaPulse = abs(deltaTarget);

		if (deltaPulse > 0) {
			if (target < position) {
				position -= deltaPulse;
				if(position < target){
					position = target;
				}
			} else {
				position += deltaPulse;
				if(position > target){
					position = target;
				}
			}
			position = calibration.constrainMicros(position);
			prev = cur;
			if(position == target){
				moving = false;
			}
		}

	}
	write(position);

	return (position == target);
}

int Joint::saveCalibration(int aAddress){
	return calibration.saveCalibration(aAddress);
}

int Joint::loadCalibration(int aAddress){
	return calibration.readCalibration(aAddress);
}


//  Saves 3 ints for a total of 6 bytes.
int Joint::saveState(int aAddress){

	int add = 0;
	add += writeToEEPROM(aAddress + add, position);
	add += writeToEEPROM(aAddress + add, target);
	add += writeToEEPROM(aAddress + add, speed);

	return add;
}


int Joint::recallState(int aAddress){

	int add = 0;
	add += readFromEEPROM(aAddress + add, position);
	add += readFromEEPROM(aAddress + add, target);
	add += readFromEEPROM(aAddress + add, speed);

	return add;

}




void Joint::followTheStick(int aReading){

	int res = map(aReading, -32768, 32767, calibration.minimumMicros, calibration.maximumMicros);
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
	 * SOOOOOOO
	 * Lets scale the input between 0 and 1 for this and then multiply that
	 * times our speed variable and set the speed by that.  We just need to
	 * think of a reasonable time in the future to set the target value
	 * or do we set it with move to immediates and hope it's fast enough?
	 * We could just track the last update time and go from there based on
	 * the speed we come up with.  Yeah, let's try that first.
	 *
	 *
	 * We could even use the method from DiscoBot where we use the
	 * known time dif but I'd rather not rely on that when we have
	 * unknowns about transmission speeds.
	 *
	 *
	 */





	unsigned long cm = millis();

	//The delta time should be last time since a
	//move or a zero reading from the stick.

	if(aReading == 0){
		lastStickUpdate = cm;
		return;
	}

	float timeScale = (cm - lastStickUpdate) / 1000.0;  // speed is in microsecond steps per second
	float speedRatio = (float)aReading / 32767.0;
	int16_t step = speed * speedRatio * timeScale;

	if((step >= 1)||(step <= -1)){
		moveToImmediate(position + step);
		lastStickUpdate = cm;
	}
}







/*********




For the purpose of kinematics, let's construct a line drawing
starting at the base servo and going up through the middle of
all the arm pieces.  If we want to fill things out later we can.
But for now all arm pieces are modeled as a line.

For the purposes of this kinematics we will set our coordinate frame
such that 0,0,0 is the hub of the base servo.  Positive Y will be
towards the front of the vehicle.  Positive X will be to the right.
And positive Z will be up.  Angles will be measured so 90 degrees
(pi/2) is straight up and down.  Angles less than 90 will be towards
the front of the vehicle if the base is at it's mid-position with the
arm facing forwards.





**************/


//  Let's start with some utility funcs.

// Start by treating the arm as a 2D flat object and calculate the distance from base (X) and height (Y)
// then you can combine that with data from base servo later and calculate actual xyz.

//  Find end of hypotenuse from angle at 0,0
XYpoint solveTriangle (float aAngle, uint16_t aLength){
	XYpoint retval = {0,0};
	retval.x = cos(aAngle) * aLength;
	retval.y = sin(aAngle) * aLength;
	return retval;
}


XYandAngle Joint::findEndXY(XYpoint aPivot, float aAngle){

	float endAngle = aAngle + calibration.microsToAngle(position) - 1.5708;

	XYpoint solution = solveTriangle(endAngle, length);

	XYandAngle retval;

	retval.x = solution.x + aPivot.x;
	retval.y = solution.y + aPivot.y;
	retval.approachAngle = endAngle;

	return retval;
}


XYandAngle Joint::findEndXY(XYandAngle aPivot){

	float endAngle = aPivot.approachAngle + calibration.microsToAngle(position) - 1.5708;

	XYpoint solution = solveTriangle(endAngle, length);

	if(offset != 0){
		XYpoint offsetSolution = solveTriangle(endAngle + 90, offset);
		solution.x += offsetSolution.x;
		solution.y += offsetSolution.y;
	}

	XYandAngle retval;

	retval.x = solution.x + aPivot.x;
	retval.y = solution.y + aPivot.y;
	retval.approachAngle = endAngle;

	return retval;
}




XYandAngle Joint::findEndXY(XYandAngle aPivot, float aPosition){

	float endAngle = aPivot.approachAngle + calibration.constrainAngle(aPosition) - 1.5708;

		XYpoint solution = solveTriangle(endAngle, length);

		if(offset != 0){
			XYpoint offsetSolution = solveTriangle(endAngle + 90, offset);
			solution.x += offsetSolution.x;
			solution.y += offsetSolution.y;
		}

		XYandAngle retval;

		retval.x = solution.x + aPivot.x;
		retval.y = solution.y + aPivot.y;
		retval.approachAngle = endAngle;

		return retval;

}
