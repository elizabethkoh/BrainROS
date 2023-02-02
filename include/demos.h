
#ifndef _DEMOS_H_
#define _DEMOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> // needed for the types declariations

enum class MoveStates {BotStop = 5, Compute = 0, BotRotate = 6,
	BotContantSpeed = 2, BotAcc = 1, BotTurn = 3, BotStopping = 4,BotSense =7};

void moveTestNoCamAndOpSensor(double ticksMax, int speedVel, int speedValSens,
  double sense_dist, bool rev, int stopVal, int stopValCount, uint32_t sensorSelect);
void moveTestNoCamAssist(double ticksMax, int speedVel, bool rev);
void rotateYawOdom(double yawDegree, int32_t velocity, double L, int timeOut);
void testAutonDriveCorner();

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif //_DEMOS_H_
