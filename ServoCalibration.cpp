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

	Serial.print("<ATM,");
	Serial.print(angle,3);
	Serial.print(",");


	// constrain the angle
	angle = constrainAngle(angle);

	float ratio = (angle - minimumAngle) / (maximumAngle - minimumAngle);

	retval =  (ratio * (maximumMicros - minimumMicros)) + minimumMicros;

	Serial.print(ratio,3);
	Serial.print(",");
	Serial.print(retval);
	Serial.print(">");

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

	if (minimumAngle < maximumAngle) {
		if (retval < minimumAngle) {
			retval = minimumAngle;
		} else if (retval > maximumAngle) {
			retval = maximumAngle;
		}
	} else if (minimumAngle > maximumAngle) {
		if (retval > minimumAngle) {
			retval = minimumAngle;
		} else if (retval < maximumAngle) {
			retval = maximumAngle;
		}
	} else {  // minimum and maximum are equal???
		retval = minimumAngle;
	}
	return retval;
}


int ServoCalibrationStruct::saveCalibration(int address) {

	int add = 0;
	add += writeToEEPROM(address + add, minimumMicros);
	add += writeToEEPROM(address + add, minimumAngle);
	add += writeToEEPROM(address + add, maximumMicros);
	add += writeToEEPROM(address + add, maximumAngle);
	return add;

}

int ServoCalibrationStruct::readCalibration(int address) {

	int add = 0;
	add += readFromEEPROM(address + add, minimumMicros);
	add += readFromEEPROM(address + add, minimumAngle);
	add += readFromEEPROM(address + add, maximumMicros);
	add += readFromEEPROM(address + add, maximumAngle);
	return add;

}

