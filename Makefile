
#
ARDUINO_DIR = /home/david/.arduino15
ARDMK_DIR = /usr/share/arduino
AVR_TOOLS_DIR = /usr/

ARDUINO_VAR_PATH=/home/david/.arduino15/packages/arduino/hardware/avr/1.8.6/variants
BOARDS_TXT=/home/david/.arduino15/packages/arduino/hardware/avr/1.8.6/boards.txt
ARDUINO_CORE_PATH=/home/david/.arduino15/packages/arduino/hardware/avr/1.8.6/cores/arduino
BOOTLOADER_PARENT=/home/david/.arduino15/packages/arduino/hardware/avr/1.8.6/bootloaders

ARDUINO_PLATFORM_LIB_PATH = /home/david/.arduino15/packages/arduino/hardware/avr/1.8.6/libraries
#
USER_LIB_PATH = /home/david/Arduino/libraries
#
#
TARGET = RobotArmController.cpp
#
 BOARD_TAG    = nano  # for mega use mega2560
 MCU	=	atmega328p

 AVRDUDE_OPTS 	   = -v
 ARDUINO_PORT = /dev/ttyACM0  # change this to the port used by your board

ARDUINO_LIBS = RobotSharedDefines
ARDUINO_LIBS += Joint
ARDUINO_LIBS += Servo
ARDUINO_LIBS += EepromFuncs
ARDUINO_LIBS += EEPROM
ARDUINO_LIBS += ServoCalibration
ARDUINO_LIBS += Gimbal
ARDUINO_LIBS += CommandParser
ARDUINO_LIBS += XboxHandler
ARDUINO_LIBS += StreamParser


CPPFLAGS += -DARDUINO_ARCH_AVR
CPPFLAGS += -D__AVR_ATmega328P__

#
  PRE_BUILD_HOOK = ../preCompile.sh
#
include /usr/share/arduino/Arduino.mk

