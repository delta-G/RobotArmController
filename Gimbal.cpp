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

#include "Gimbal.h"



GimbalClass::GimbalClass(){
	panJoint = NULL;
	tiltJoint = NULL;
}

GimbalClass::GimbalClass(Joint* aPanJoint, Joint* aTiltJoint){
	panJoint = aPanJoint;
	tiltJoint = aTiltJoint;
}

Joint* GimbalClass::getPanJoint(){
	return panJoint;
}

Joint* GimbalClass::getTiltJoint(){
	return tiltJoint;
}

void GimbalClass::init(){
	panJoint->init();
	delay(25);
	tiltJoint->init();
}

void GimbalClass::detach(){
	panJoint->detach();
	tiltJoint->detach();
}

void GimbalClass::run(){
	panJoint->run();
	tiltJoint->run();
}


void GimbalClass::stop(){
	panJoint->stop();
	tiltJoint->stop();
}


float GimbalClass::getPanAngle(){
	return panJoint->getAngle();
}

float GimbalClass::getTiltAngle(){
	return tiltJoint->getAngle();
}

int GimbalClass::saveCalibrations(){

	int add = 0;
	add += panJoint->saveCalibration(EEPROM_CALIBRATION_START + EEPROM_GIMBAL_CALIBRATION_OFFSET + add);
	add += tiltJoint->saveCalibration(EEPROM_CALIBRATION_START + EEPROM_GIMBAL_CALIBRATION_OFFSET + add);
	return add;
}

int GimbalClass::loadCalibrations() {

	int add = 0;
	byte flags = EEPROM.read(EEPROM_FLAG_BYTE);
	flags = ~flags;  // flags are set as 0 since cleared EEPROM is 0xFF
	if (flags & FLAG_CALIBRATIONS_SAVED) {
		add += panJoint->loadCalibration(EEPROM_CALIBRATION_START + EEPROM_GIMBAL_CALIBRATION_OFFSET + add);
		add += tiltJoint->loadCalibration(EEPROM_CALIBRATION_START + EEPROM_GIMBAL_CALIBRATION_OFFSET + add);
	}

	//  Cool side effect, this returns 0 if the flag isn't set.
	return add;
}



