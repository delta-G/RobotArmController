#ifndef _RobotArmController_H_
#define _RobotArmController_H_
#include "Arduino.h"

#include <Servo.h>

#include "Joint.h"

#define SER_BAUD 115200
#define PROBLEM_LED A0
#define HEARTLED 13

#define MAX_COMMAND_LENGTH 20
#define START_OP '<'
#define END_OP '>'
#define SEPERATOR ','

enum JointsE {
	BASE, SHOULDER, ELBOW, WRIST, ROTATE, GRIP, PAN, TILT, NUMBER_OF_JOINTS
};

void setup();
void loop();
void heartbeat();

void parseCommand();

#endif /* _RobotArmController_H_ */
