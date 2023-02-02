/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif

// BrainRos region in main.h
// regquied for serial port communications
// users to add #include main.h in their .c, .cpp files to use the serial port
//
// to use the smart port number PORTNUMBER on V5 Brain, first
// vexGenericSerialEnable(PORTNUMBER -1,0);  // this code is done
// initialize() in main.cpp
//
// the next step is to set the desired baurd rate
// vexGenericSerialBaudrate( PORTNUMBER - 1, 921600 );
extern "C" int32_t vexGenericSerialReceive( uint32_t index, uint8_t *buffer, int32_t length );
extern "C" int32_t vexGenericSerialTransmit( uint32_t index, uint8_t *buffer, int32_t length );
extern "C" void vexGenericSerialEnable(  uint32_t index, uint32_t nu );
extern "C" void vexGenericSerialBaudrate(  uint32_t index, uint32_t rate );

// Port to use for serial data
#define SERIALPORT 20
#define DEBUG_BAUD_RATE 921600

// different conversions, needed
#define TICKS_PER_INCH 71.6197243913529 //4pi inches / 900 ticks, 4 in omnis
#define RADIAN_PER_DEG 0.017453292519943295 // pi/180

// physical constants
#define WHEEL_TO_WHEEL_LENGTH 14.5 //inches
#define BOT_MAXSPEED 42.0 // inch/s measured using odometry


#define BUTTON_TIME_LOCK 350
#define REVERSE_TURN_ANGLE 10.0


#define SINGLE_CHANNEL_DRIVE 1
#define TANK_DRIVE 0


// ptr has to be declared as uint8_t array
#define UPRINT(_ptr) ({ \
	vexGenericSerialTransmit(SERIALPORT-1, _ptr,strlen((char *)_ptr)); \
})

void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_