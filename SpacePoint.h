/*
 * SpacePoint.h
 *
 *  Created on: Feb 23, 2019
 *      Author: david
 */

#ifndef SPACEPOINT_H_
#define SPACEPOINT_H_

#include "Arduino.h"

struct SpacePoint {

	int16_t x;
	int16_t y;
	int16_t z;

	void printOut(){
			Serial.print(x);
			Serial.print(" ");
			Serial.print(y);
			Serial.print(" ");
			Serial.print(z);
		}

};

struct XYpoint {
	int16_t x;
	int16_t y;
};

struct XYandAngle {
	int16_t x;
	int16_t y;
	float approachAngle;  // approach angle

	void printOut(){
		Serial.print(x);
		Serial.print(" ");
		Serial.print(y);
		Serial.print(" ");
		Serial.print(approachAngle);
	}
};

#endif /* SPACEPOINT_H_ */
