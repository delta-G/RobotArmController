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


#include"ServoCalibration.h"

uint16_t ServoCalibrationStruct::angleToMicros(float angle){
	uint16_t retval = 0;

	// constrain the angle
	angle = constrainAngle(angle);

	float ratio = (angle - minimumAngle) / (maximumAngle - minimumAngle);

	retval =  (ratio * (maximumMicros - minimumMicros)) + minimumMicros;

	return retval;
}

float ServoCalibrationStruct:: microsToAngle(uint16_t aMicros){

	float retval = 0.0;

	aMicros = constrainMicros(aMicros);

	float ratio = (float)(aMicros - minimumMicros) / (float)(maximumMicros - minimumMicros);
	retval = (ratio * (maximumAngle - minimumAngle)) + minimumAngle;

	return retval;
}

void ServoCalibrationStruct::calibrate(uint16_t aMinMicros, float aMinAngle, uint16_t aMaxMicros, float aMaxAngle){

	minimumMicros = aMinMicros;
	minimumAngle = aMinAngle;
	maximumMicros = aMaxMicros;
	maximumAngle = aMaxAngle;

}

uint16_t ServoCalibrationStruct::constrainMicros(uint16_t aMicros){

	uint16_t retval = aMicros;
	if(retval < minimumMicros){
		retval = minimumMicros;
	}
	else if(retval > maximumMicros){
		retval = maximumMicros;
	}
	return retval;
}

float ServoCalibrationStruct::constrainAngle(float aAngle) {
	float retval = aAngle;
	if (retval < minimumAngle) {
		retval = minimumAngle;
	} else if (retval > maximumAngle) {
		retval = maximumAngle;
	}
	return retval;
}


void ServoCalibrationStruct::saveCalibration(int address) {

	int add = address;
	add += writeToEEPROM(add, minimumMicros);
	add += writeToEEPROM(add, minimumAngle);
	add += writeToEEPROM(add, maximumMicros);
	add += writeToEEPROM(add, maximumAngle);

}

void ServoCalibrationStruct::readCalibration(int address) {

	int add = address;
	add += readFromEEPROM(add, minimumMicros);
	add += readFromEEPROM(add, minimumAngle);
	add += readFromEEPROM(add, maximumMicros);
	add += readFromEEPROM(add, maximumAngle);

}

