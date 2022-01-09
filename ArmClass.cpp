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

#include "ArmClass.h"

//#define NUMBER_OF_JOINTS 8
extern GimbalClass gimbal;

void Arm_Class::run() {

	for (int i = 0; i < numJoints; i++) {
		joints[i].run();
	}

	State_Enum newState = state;

	switch (state) {
	case READY:
		if(isMoving()){
			newState = MOVING;
		}
		if(gimbal.isMoving()){
			newState = MOVING;
		}
		break;
	case MOVING:
		if(!(isMoving() || gimbal.isMoving())){
			newState = READY;
			if (movementDoneCallback != NULL) {
				if (movementDoneCallback()) {
					// callbacks should return true when they are done being called
					// each time state goes back to ready
					movementDoneCallback = NULL;
				}
			}
		}
		break;
	}  // end switch

	state = newState;

}

void Arm_Class::setCallback(boolean (*aCallback)()){
	movementDoneCallback = aCallback;
}


Arm_Class::Arm_Class(){

	joints = 0;
	numJoints = 8;
	positionValid = false;
	servoPower = false;
	movementDoneCallback = NULL;
	state = READY;

}

Arm_Class::Arm_Class(Joint* aJoints, int aNum){
	joints = aJoints;
	numJoints = aNum;
	positionValid = false;
	servoPower = false;
	movementDoneCallback = NULL;
	state = READY;
}

uint8_t Arm_Class::getStatusByte(){
	uint8_t retval = 0;
	if(servoPower){
		retval |= 0x01;
	}
	if(isMoving()){
		retval |= 0x02;
	}
	if(gimbal.isMoving()){
		retval |= 0x04;
	}
	return retval;
}

int Arm_Class::getNumJoints(){
	return numJoints;
}

void Arm_Class::setServoPower(boolean aBoo){
	servoPower = aBoo;
	digitalWrite((byte)SERVO_POWER_PIN, aBoo);
}

Joint* Arm_Class::getJoint(unsigned int aIndex){
	if(aIndex >= numJoints){
		return &joints[numJoints - 1];
	}
	return &joints[aIndex];
}

void Arm_Class::addJoint(int i, Joint j){
	if((i >=0)  && (i < numJoints)){
		joints[i] = j;
	}
}

//  This has 2 seconds of delay in it for those who are counting.
void Arm_Class::init(){
//	loadAll(EEPROM_INITIAL_STATES);
	for(int i = 0; i < numJoints; i++){
		joints[i].init();
		delay(250);
	}
}

void Arm_Class::attachAll() {
	for (int i = 0; i < numJoints; i++) {
		joints[i].attach(joints[i].getPin());
		delay(250);
	}
}

void Arm_Class::detachAll() {
	for (int i = 0; i < numJoints; i++) {
		joints[i].detach();
		delay(250);
	}
}

void Arm_Class::stop() {
	for (int i = 0; i < numJoints; i++) {
			joints[i].stop();
		}
}

boolean Arm_Class::isMoving(){

	for(int i = 0; i< numJoints; i++){
		if(joints[i].isMoving()){
			return true;
		}
	}
	return false;
}



int Arm_Class::savePosition(int aAddress){
	int add = (aAddress >= 32)? aAddress : EEPROM_POSITION(aAddress);
	int offset = 0;
	for(int i = 0; i < numJoints; i++){
		int p = joints[i].getPosition();
		offset += writeToEEPROM(add + offset, p );
	}
	return offset;
}


//  Sends Robot to a position with the speed values it already has.
int Arm_Class::gotoPosition(int aAddress){

	int add = (aAddress >= 32)? aAddress : EEPROM_POSITION(aAddress);
	int offset = 0;

	for(int i = 0; i < numJoints; i++){
		int t;
		offset += readFromEEPROM(add + offset, t);
		joints[i].setTarget(t);
	}
	state = MOVING;
	return offset;
}


//  Sets both the target and the position and speed.
//  It is the responsibility of the caller to make
//  sure that the arm gets to this position before
//  another is fed.

int Arm_Class::loadMovement(int aAddress) {
	int a = 0;

	for (int i = 0; i < numJoints; i++) {
		int t;
		int s;
		a += readFromEEPROM(aAddress + a, t);
		a += readFromEEPROM(aAddress + a, s);

		joints[i].setTarget(t);
		joints[i].setSpeed(s);
	}
	return a;
}

int Arm_Class::loadAll(int aAdress){
	int x = loadStates(aAdress);
	loadCalibrations();
	return x;
}

int Arm_Class::saveAll(int aAdress){
	int x = saveStates(aAdress);
	saveCalibrations();
	return x;
}

int Arm_Class::saveStates(int aAddress){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].saveState(aAddress + add);
	}
	return add;
}

int Arm_Class::loadStates(int aAddress){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].recallState(aAddress + add);
	}
	return add;
}

int Arm_Class::saveCalibrations(){

	int add = 0;
	for(int i = 0; i < numJoints; i++){
		add += joints[i].saveCalibration(EEPROM_CALIBRATION_START + add);
	}
	byte flags = EEPROM.read(EEPROM_FLAG_BYTE);
	flags &= ~FLAG_CALIBRATIONS_SAVED;  // flags are set as 0 since cleared EEPROM is 0xFF
	EEPROM.write(EEPROM_FLAG_BYTE, flags);
	return add;
}

int Arm_Class::loadCalibrations() {

	int add = 0;
	byte flags = EEPROM.read(EEPROM_FLAG_BYTE);
	flags = ~flags;  // flags are set as 0 since cleared EEPROM is 0xFF
	if (flags & FLAG_CALIBRATIONS_SAVED) {
		for (int i = 0; i < numJoints; i++) {
			add += joints[i].loadCalibration(EEPROM_CALIBRATION_START + add);
		}
	}

	//  Cool side effect, this returns 0 if the flag isn't set.
	return add;
}


//
//
//XYandAngle Arm_Class::findEndEffector(){
//
//	// Location of base for this xperiment is the
//	// pivot of the base servo.
//
//	// This xperiment is just the 2D plane of the arm, so
//	// we can locate the pivot point of the shoulder by just
//	// adding to our Y variable (Z direction in 3D) X - Z plane
//
//	XYpoint basePoint = {0,0};
//
//	XYpoint shoulderPoint = {basePoint.x , basePoint.y + 37};
//
//	XYandAngle elbowPoint = joints[1].findEndXY(shoulderPoint, 1.5708);
//
//	XYandAngle wristPoint = joints[2].findEndXY(elbowPoint);
//
//	XYandAngle gripperTip = joints[3].findEndXY(wristPoint);
//
//	SpacePoint tip3D = {0,0,0};
//
//	tip3D.z = gripperTip.y; //// In the 2D model we had Y for height
//	///  x in the 2D model is the radius in this calc.
//	///   TODO:  Fix the Z vs Y issue
//
//	float baseAngle = joints[0].getAngle();
//	tip3D.x = gripperTip.x * cos(baseAngle);
//	tip3D.y = gripperTip.x * sin(baseAngle);
//
//
//	Serial.print("<");
//
//	Serial.print("EB ");
//	elbowPoint.printOut();
//
//	Serial.print(" WR ");
//	wristPoint.printOut();
//
//	Serial.print(" GT ");
//	gripperTip.printOut();
//
//	Serial.print(" 3D ");
//	tip3D.printOut();
//
//	Serial.print(">");
//
//	return gripperTip;
//}
//
//
//SpacePoint xyAngleToSpacePoint(XYandAngle xy, float baseAngle ){
//
//	SpacePoint retval;
//
//	retval.z = xy.y;  // y in our xy plane is z in the 3d TODO:  Fix that
//	retval.x = xy.x * cos(baseAngle);
//	retval.y = xy.x * sin(baseAngle);
//
//	return retval;
//
//}
//
//
//void Arm_Class::findPosition(){
//
//	XYpoint basePoint = {0,0};
//
//	XYpoint shoulderPoint = {basePoint.x , basePoint.y + 37};
//
//	XYandAngle elbowPoint = joints[1].findEndXY(shoulderPoint, 1.5708);
//
//	XYandAngle wristPoint = joints[2].findEndXY(elbowPoint);
//
//	XYandAngle gripperTip = joints[3].findEndXY(wristPoint);
//
//	currentPositions.basePoint = { 0, 0, 0};
//
//	currentPositions.shoulderPoint = currentPositions.basePoint;
//	currentPositions.shoulderPoint.z += 37;
//
//	float baseAngle = joints[0].getAngle();
//	currentPositions.elbowPoint = xyAngleToSpacePoint(elbowPoint, baseAngle);
//	currentPositions.wristPoint = xyAngleToSpacePoint(wristPoint, baseAngle);
//	currentPositions.gripperTip = xyAngleToSpacePoint(gripperTip, baseAngle);
//
//	positionValid = true;
//
//}
//
//ArmPositionStruct Arm_Class::jointSpaceToPosition(JointSpaceStruct aPos) {
//
//	ArmPositionStruct retval;
//
//	//  First two never change...
//
//	XYpoint basePoint = {0,0};
//
//	XYandAngle shoulderPoint = {basePoint.x , basePoint.y + 37, 1.5708};
//
//	// These we calculate with the static function
//	XYandAngle elbowPoint = joints[1].findEndXY(shoulderPoint, aPos.shoulderAngle);
//
//	XYandAngle wristPoint = joints[2].findEndXY(elbowPoint, aPos.elbowAngle);
//
//	XYandAngle gripperTip = joints[3].findEndXY(wristPoint, aPos.wristAngle);
//
//	retval.basePoint = { 0, 0, 0};
//
//	retval.shoulderPoint = retval.basePoint;
//	retval.shoulderPoint.z += 37;
//
//	float baseAngle = joints[0].getAngle();
//	retval.elbowPoint = xyAngleToSpacePoint(elbowPoint, baseAngle);
//	retval.wristPoint = xyAngleToSpacePoint(wristPoint, baseAngle);
//	retval.gripperTip = xyAngleToSpacePoint(gripperTip, baseAngle);
//
//	return retval;
//
//}
//
//
///*
// *
// * Now inverse Kinematics
// * This is a herder nut to crack
// *
// * Let's start by saying where we want the tip.
// *
// * We can do something similar to how we solved the problem in
// * the forward case.  Let's devolve it into a 2D problem
// *
// * We know the angle that the desired point makes with the centerline
// * of the bot.  So we know the angle that the base servo has to go to.
// * It's the same conversion from polar to cartesian that we had on
// * the forward kinematics.  Then we calculate a radial distance and
// * turn it into a 2D problem for the rest of the servos.
// *
// * So we place the gripper tip in our 2D plane.
// * Then we can calculate an arc about that point that represents
// * all the places that the wrist servo could possibly be.
// * From there we can choose the point on that arc closest to
// * where the wrist is now.
// *
// * Then from there we do something similar with the elbow.
// * From the wrist position we calculate an arc of places the
// * elbow could be.  We caculate an arc of places the elbow could be
// * from the shoulder's point of view.  Where those two intersect is
// * where the elbow needs to go.  If there are two intersections then
// * we take the one that is closest to where the elbow is now OR we
// * take the one that involves the least movement of servos.
// *
// *
// * We need LOTS AND LOTS of safety built in.  If it tries to go to a position
// * that it can't do then we need safeties.
// *
// *
// *
// * It may be simpler than I think.  If we know the position of the tip and
// * the end angle of the claw, then we know exactly the position of the wrist.
// * From there it is spherical geometry.  The distance from the shoulder servo
// * gives the elbow angle and then the point on that sphere determines the shoulder
// * and the base angle.
// *
// * The trick is what if the position can't be reached with that given claw angle.
// * How do we then determine what angle to use?  We would have to do like above and
// * generate an arc and find intersections.  But the wrist position creates a pretty
// * good 3D field.  Maybe we need to just go ahead and define it.
// *
// * It can't be hard.  Most of it is as simple as defining spheres or sections of spheres.
// * There's a bubble where your max reach is.  Another bubble around the base of the arm.
// * A slightly more intricate set of bubbles around the tank base.  It looks like it can all
// * be modeled with spheres.  Time to bone up on 3D geometry.
// *
// */
//
//
///*
// *  First is the simple case.  We give the tip position and angle of the gripper.
// *  This means that we can directly calculate wrist position.
// *
// *  From wrist position there are only two solutions to elbow and shoulder
// *  One is the "elbow-up" position and the other is "elbow-down"
// *
// *  Because our robot has the big head on it, I think we only want to consider
// *  elbow up possibilities.  Maybe there are cases and we can explore that later.
// */
//
////  Find end of hypotenuse from angle at 0,0
////XYpoint solveTriangle (float aAngle, uint16_t aLength){
////	XYpoint retval = {0,0};
////	retval.x = cos(aAngle) * aLength;
////	retval.y = sin(aAngle) * aLength;
////	return retval;
////}
//
//JointSpaceStruct Arm_Class::gripperPositionToJointSpace(SpacePoint xyz, float wristAngle) {
//
//	//  Convert our 3D point to the 2D plane by taking out the base angle
//
//	XYpoint tip;
//	// x is distance in x-y plane of 3D
//	tip.x = sqrt((xyz.x * xyz.x) + (xyz.y * xyz.y));
//	// y in the 2D is z in the 3D
//	tip.y = xyz.z;
//
//	XYpoint wrist = solveTriangle((wristAngle - PI), joints[3].getLength());
//
//	//  The distance from the shoulder in 2D space determines the elbow angle.
//	//  Nothing can be done about that.
//	//  We're only calculating Elbow UP here
//
//	// This is NOT a right triangle.
//	//  We have 3 known sides and want one angle.
//
//	// Law of cosines  a and b are adjacent sides and c is opposite
//
//	// cos(C) = (a^2 + b^2 - c^2) / 2ab
//
//	// shoulder is fixed
//	XYpoint shoulder = {0 , 37};
//
//	// c is our distance from shoulder
//	float c = sqrt(((wrist.x - shoulder.x)*(wrist.x - shoulder.x)) + ((wrist.y - shoulder.y)*(wrist.y - shoulder.y)));
//
//	float a = joints[1].getLength();
//	float b = joints[2].getLength();
//
//	float cosC = ((a*a) + (b*b) - (c*c)) / (2 * a * b);
//
//	float elbowAngle = acos(cosC);
//
//	//  If the elbow makes an angle of elbowAngle, then the elbow servo makes an angle of
//	//  90 - (180 - elbowAngle)  // to radians of course.
//	//  90 - 180 + elbowAngle
//	// -90 + elbowAngle
//	//  elbowAngle - 90
//
//	float elbowServoAngle = elbowAngle - (PI/2);
//
//	float cosB = ((a*a) + (c*c) - (b*b)) / (2 * a * c);
//
//	float angleB = acos(cosB);
//
//	int d = wrist.y - shoulder.y;
//
//	float angleD = atan((float)d/c);
//
//	float shoulderServoAngle = angleB + angleD;
//
//	float wristServoAngle = wristAngle - shoulderServoAngle - elbowServoAngle + PI;
//
//	// That solves distance from the base pivot and the z axis in 3D as an X-Y problem
//	//  Now we need to get the angle for the base.
//
//	float baseAngle = atan((float) xyz.y / xyz.x);
//
//	Serial.print("<JSPC-");
//	Serial.print("tipxy");
//	Serial.print(tip.x);
//	Serial.print(",");
//	Serial.print(tip.y);
//	Serial.print(" - ");
//
//	Serial.print("<JSPC-");
//	Serial.print("wristxy");
//	Serial.print(wrist.x);
//	Serial.print(",");
//	Serial.print(wrist.y);
//	Serial.print(" - ");
//
//	Serial.print("c=");
//	Serial.print(c);
//	Serial.print(",");
//
//	Serial.print("a=");
//	Serial.print(a);
//	Serial.print(",");
//
//	Serial.print("b=");
//	Serial.print(b);
//	Serial.print(",");
//
//	Serial.print("cosC=");
//	Serial.print(cosC);
//	Serial.print(",");
//
//	Serial.print("ESA=");
//	Serial.print(elbowServoAngle);
//	Serial.print(",");
//
//	Serial.print("cosB=");
//	Serial.print(cosB);
//	Serial.print(",");
//
//	Serial.print("angB=");
//	Serial.print(angleB);
//	Serial.print(",");
//
//	Serial.print("angD=");
//	Serial.print(angleD);
//	Serial.print(",");
//
//	Serial.print("shAng=");
//	Serial.print(shoulderServoAngle);
//	Serial.print(",");
//
//	Serial.print("wrAng=");
//	Serial.print(wristServoAngle);
//	Serial.print(",");
//
//	Serial.print("bAng=");
//	Serial.print(baseAngle);
//	Serial.print(">");
//
//	JointSpaceStruct retval = {baseAngle, shoulderServoAngle, elbowServoAngle, wristServoAngle};
//
//	return retval;
//
//}
