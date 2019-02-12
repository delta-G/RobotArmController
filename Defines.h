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


#ifndef DEFINES_H_
#define DEFINES_H_


#define SERVO_POWER_PIN A3
#define HEARTLED 13


#define FLAG_CALIBRATIONS_SAVED 0x10


#define EEPROM_START_FLAG 0
#define EEPROM_START_VALUE 0x47
#define EEPROM_FLAG_BYTE 1

#define EEPROM_CALIBRATION_START 24  // 12 bytes per calibration * 8 joints = 96 bytes of calibration!
// 96 + 24 puts the initial states starting at 120
#define EEPROM_INITIAL_STATES (EEPROM_CALIBRATION_START + (NUMBER_OF_JOINTS * sizeof(ServoCalibrationStruct)));
// initial states takes 6 bytes per servo times 8 servos or 48 bytes.
// So this ends at 168

//  Each position takes 2 bytes per servo times 8 servos for 16 bytes.
#define EEPROM_POSITION_STANDING 256
#define EEPROM_POSITION_SITTING 288

#define EEPROM_POSITION(x) (512 + (x * 16))  // This leaves room for 512 / 16 == 32 positions to save


boolean eepromGood();



#define MAX_COMMAND_LENGTH 64



#endif /* DEFINES_H_ */
