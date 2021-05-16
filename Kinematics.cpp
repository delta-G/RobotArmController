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


#include "Kinematics.h"

extern Arm_Class arm;

//aReturnArray is an array of four floats to get back the angles of the first four servos
boolean runInverse(float *aReturnArray, int16_t aX, int16_t aY, int16_t aZ, float aPhi, uint16_t aTolerance){
	inverseK1(aReturnArray, aX, aY, aZ, aPhi);
	PositionStruct returned=forwardK1(aReturnArray[0], aReturnArray[1], aReturnArray[2], aReturnArray[3]);
	PositionStruct requested(aX, aY, aZ, aPhi);
	return requested.compare(returned, aTolerance);
}


// aReturnArray should be a pointer to an array of four float.
PositionStruct forwardK1(float baseAngle, float shoulderAngle, float elbowAngle, float wristAngle){
	int16_t a1 = arm.getJoint(0)->getLength();
	int16_t a2 = arm.getJoint(1)->getLength();
	int16_t a3 = arm.getJoint(2)->getLength();
	int16_t a4 = arm.getJoint(3)->getLength();
	int16_t a5 = arm.getJoint(3)->getOffset();

	float s1 = sin(baseAngle);
	float s2 = sin(shoulderAngle);
	float s3 = sin(elbowAngle);
	float s4 = sin(wristAngle);

	float c1 = cos(baseAngle);
	float c2 = cos(shoulderAngle);
	float c3 = cos(elbowAngle);
	float c4 = cos(wristAngle);

	float s2s3 = s2*s3;
	float c2c3 = c2*c3;
	float s2c3 = s2*c3;
	float c2s3 = c2*s3;

	float x4 = (a4*c4) - (a5*s4);
	float y4 = (a4*s4) - (a5*c4);

	float xCoord = s1
			* (((s2s3 - c2c3) * x4) + ((s2c3 + c2s3) * y4)
					+ (a3 * (s2c3 + c2s3)) + (a2 * c2));
	float yCoord = (0 - c1)
			* (((s2s3 - c2c3) * x4) + ((s2c3 + c2s3) * y4)
					+ (a3 * (s2c3 + c2s3)) + (a2 * c2));
	float zCoord = ((0 - c2s3 - s2c3) * x4) + ((s2s3 - c2c3) * y4)
			+ (a3 * (s2s3 - c2c3)) + (a2 * s2) + a1;

	float zproj = (c4 * (0 - c2s3 - s2c3)) + (s4 * (s2s3 - c2c3));
	float phi = asin(zproj);

	PositionStruct retval(xCoord, yCoord, zCoord, phi);
	return retval;

}



/*
 *
 * For our arm in it's current configuration it takes two things to fully
 * describe where the end effector is.  You need the xyz coordinates of
 * some part (we will use the tip of the closed gripper) and you also need
 * the angle from the Z axis to tell you how it is tilted.
 *
 * We will start by using those two things to calculate the position of the
 * wrist servo.  Then we will use inverse kinematics to find the angles needed
 * on shoulder and elbow servo to get an "elbow-up" solution.  Once we have
 * that solution, we can use the rotation of the forearm to calculate the
 * rotation needed for the end effector.
 *
 * Then we will check that solution through our already solved forward kinematics.
 *
 */



//TODO:
//  This fails in the case where the phi angle is less than -1.57 or greater than 1.57
//  because the z projection can't distinguish those angles from angles in the first or fourth quadrant
//  So we need to come up with a plan to handle that situation.
void inverseK1(float *aReturnArray, int16_t aX, int16_t aY, int16_t aZ, float aPhi) {
	int16_t a1 = arm.getJoint(0)->getLength();
	int16_t a2 = arm.getJoint(1)->getLength();
	int16_t a3 = arm.getJoint(2)->getLength();
	int16_t a4 = arm.getJoint(3)->getLength();
	int16_t a5 = arm.getJoint(3)->getOffset();
	/*
	 * Step 1 Calculate the base Angle (the easy part)
	 */
	float baseTheta = 0.0;
	if (aX == 0) {
		if (aY > 0) {
			baseTheta = -PI / 2;
		} else if (aY > 0) {
			baseTheta = PI / 2;
		} else {
			baseTheta = 0; // x and y both 0 (standing straight up this needs to be handled special to preserve the old base angle
		}
	} else {
		baseTheta = atan((float) aY / aX);
	}

	/*
	 * Step 2 Calculate the wrist position
	 * From here down we will work in the 2D
	 * r-z plane
	 */
//	float rdisp = (cos(aPhi)*a4) - (sin(aPhi)*a5);
//	float zdisp = (sin(aPhi)*a4) + (cos(aPhi)*a5);
//
//	float rtip = sqrt(((float)aX*aX)+((float)aY*aY));
//	float rwrist = rtip - rdisp;
	float rwrist = (sqrt(((float)aX*aX)+((float)aY*aY))) - ((cos(aPhi)*a4) - (sin(aPhi)*a5));
//	float zwrist = aZ - zdisp;
	float zwrist = aZ - ((sin(aPhi)*a4) + (cos(aPhi)*a5));

	/*
	 * Step 3 Calculate distance from wrist to base in r-z plane
	 */

	float dwr = sqrt((rwrist*rwrist)+((zwrist-a1)*(zwrist-a1)));

//	Serial.println();
////	Serial.print(F("rdisp :"));
////	Serial.println(rdisp);
////	Serial.print(F("zdisp :"));
////	Serial.println(zdisp);
////	Serial.print(F("rtip :"));
////	Serial.println(rtip);
//	Serial.print(F("rwrist :"));
//	Serial.println(rwrist);
//	Serial.print(F("zwrist :"));
//	Serial.println(zwrist);
//	Serial.print(F("dwr :"));
//	Serial.println(dwr);

	/*
	 * Step 4  Calculate elbow angle
	 */

	float elbowTheta = acos((((float)a2*a2) + ((float)a3*a3) - (dwr*dwr)) / (2.0*a2*a3));

	/*
	 * step 5  Set shoulder to get the right z
	 */

	// We need to add two angles.  Imagine a triangle between shoulder, elbow, and wrist.
	// We need the angle from 0 up to the bottom of that plus the angle of the shoulder corner.

	float t1 = atan((zwrist-a1)/rwrist);
	if(rwrist == 0){
		t1 = PI/2;
	}
	float t2 = acos((((float)a2*a2) + (dwr*dwr) - ((float)a3*a3)) / (2.0*a2*dwr));
	float shoulderTheta = t1 + t2;

//	Serial.println();
//	Serial.print(F("t1 :"));
//	Serial.println(t1);
//	Serial.print(F("t2 :"));
//	Serial.println(t2);
//	Serial.print(F("shoulderTheta :"));
//	Serial.println(shoulderTheta);

//	float shoulderTheta = (atan((zwrist-a1)/rwrist))+(acos((((float)a2*a2) + (dwr*dwr) - ((float)a3*a3)) / (2.0*a2*dwr)));

	/*
	 * Step 6  Set wrist to get the gripper angle
	 */

//	float forearmAngle = shoulderTheta + elbowTheta - PI;
//	float wristTheta = aPhi - (shoulderTheta + elbowTheta - PI) + PI;
	float wristTheta = aPhi - (shoulderTheta + elbowTheta - PI) + PI;

	//  We're done but some of these need to be rotated in their frames
	//  to translate into the angles for the servos.

	aReturnArray[0] = baseTheta + (PI/2);
	aReturnArray[1] = shoulderTheta;
	aReturnArray[2] = elbowTheta - (PI/2);
	aReturnArray[3] = wristTheta - (PI/2);

}


boolean PositionStruct::compare(PositionStruct other, uint16_t tolerance){
	boolean retval = true;

	if(abs(x-other.x) > tolerance){
		retval = false;
	}
	if(abs(y-other.y) > tolerance){
		retval = false;
	}
	if(abs(z-other.z) > tolerance){
		retval = false;
	}
	return retval;
}











