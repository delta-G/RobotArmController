/*
 * Joint.h
 *
 *  Created on: Jun 23, 2016
 *      Author: david
 */

#ifndef JOINT_H_
#define JOINT_H_

#include <Arduino.h>
#include <Servo.h>



class Joint : public Servo {

private:

	uint8_t pin;
	uint16_t position;

	char* name;

public:


	Joint(char* name, uint8_t aPin, uint16_t aPos);

	void moveToImmediate(uint16_t aPos);

	uint8_t getPin();
	uint16_t getPosition();
	char* getName();
};





#endif /* JOINT_H_ */
