#include "demos.h"
#include "main.h"

//extern
extern pros::Motor BASELEFTF;	//green 18:1
extern pros::Motor BASELEFTB;	//green 18:1
extern pros::Motor BASERIGHTF;	//green 18:1, reversed
extern pros::Motor BASERIGHTB;	//green 18:1, reversed

extern pros::Optical optical_sensor;
extern pros::Optical optical_front_sensor;

//globals
MoveStates g_moveState = MoveStates::Compute;


void moveTestNoCamAssist(double ticksMax, int speedVel, bool rev){
	/* use stateMachine for profile run to demo BrainRos unique
	capabilities
	states are :
	Compute: calc dist in tick, wheel ticks to start slowing bot down etc.

	BotAcc: energize motor acclerate, fix delay of 0.2s to reach
	constant speed

	BotContantSpeed: speed is constant, check for wheel ticks to begin braking

	BotStopping: when wheel tick reaches slow down ticks, speed of the motor is
	decrease at a contant rate of 20 per 10ms interval.
	transition to BotStop is when the abs(speedVel) < 20

	BotStop: motor.move_velocity(0). In this version, we have included the
	extra count down to ensure motor speed is actually zero before while loop exit.
	*/

	uint8_t buffer[128];
	pros::Vision *cameraPtr;
	memset(&buffer[0],0,sizeof(buffer)); // init all elements to zero


	std::sprintf((char *)&buffer[0],"running moveTestNoCamAssist\r\n");
	UPRINT(buffer);

	int reverse;
	int timeOutInterval = 3500;
	int timeOut = pros::millis() + timeOutInterval;
	int lspeed = 0,rspeed = 0;
	int lspeedSlow = 0, rspeedSlow = 0;
	int stopCount = 30;

	if(rev == true){
		reverse = -1;
		rspeed = speedVel;
		lspeed = -speedVel;
	}
	else {
		reverse = 1;
		rspeed = -speedVel;
		lspeed = speedVel;
	}
	double braking_dist = 6.0;// inch
	double goalPosiL = BASELEFTF.get_position() + (ticksMax * reverse);
	double goalPosiR = BASERIGHTF.get_position() - (ticksMax * reverse);
	double goalPosiSlowL = goalPosiL - (braking_dist*TICKS_PER_INCH * reverse);
	double goalPosiSlowR = goalPosiR + (braking_dist*TICKS_PER_INCH * reverse);


	double moveTicksL = ticksMax * reverse;
	double moveTicksR = ticksMax * -reverse;
	double moveTicksSlowL =0;
	double moveTicksSlowR = 0;
	double posL= 0;
	double posR = 0; // current position
	uint32_t now = 0;

	std::sprintf((char *)&buffer[0],"L: %f, R:%f\r\n", moveTicksL, moveTicksR);
	UPRINT(buffer);



	BASELEFTF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	BASERIGHTF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	BASELEFTB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	BASERIGHTB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	pros::vision_object_s_t visionObjs;


	double yawDegCam; // yaw

	int accCount = 0;

	int index = 0;
	double deltaSpeed = 0.;
	int deltaSpeedInt =0;
	int turnCount = 0;
	const double turnDuration = .1; // 0.1 sec => max Turn count of 0.1/10e-3 =

	bool stopFlag = false;
	bool brakeFlag = false;
	bool runFlag = false;
	bool turnTest = false;
	g_moveState = MoveStates::Compute;

	while (!runFlag){
		now = pros::millis();
		// check if run should completed
		posL = BASELEFTF.get_position();
		posR = BASERIGHTF.get_position();
		if (now >  timeOut){
			std::sprintf((char *)&buffer[0],"timeOut \r\n");
			UPRINT(buffer);
			stopFlag = true;
			g_moveState = MoveStates::BotStop;
		}

		switch (g_moveState){

			case MoveStates::Compute:

				// update all dist computation
				moveTicksL = ticksMax * reverse;
				moveTicksR = ticksMax * -reverse;
				// stop dist computation
				goalPosiL = BASELEFTF.get_position() + (ticksMax * reverse);
				goalPosiR = BASERIGHTF.get_position() - (ticksMax * reverse);
				// braking dist computation
				goalPosiSlowL = goalPosiL - braking_dist*TICKS_PER_INCH* reverse;
				goalPosiSlowR = goalPosiR + braking_dist*TICKS_PER_INCH* reverse;


				// slow down computations
				if (rev == true){ // lspeed < 0, rspeed >0
					lspeedSlow = lspeed + 20;
					rspeedSlow = rspeed - 20;
				}else{ // forwrd lspeed>0, rspeed <0
					lspeedSlow = lspeed - 20;
					rspeedSlow = rspeed + 20;
				}
				brakeFlag = false;
				g_moveState = MoveStates::BotAcc;
				accCount = 0;
				std::sprintf((char *)&buffer[0],"Compute | tt: %d, L: %f, R:%f, yaw: %2.3e\r\n",
				index, moveTicksL, moveTicksR, yawDegCam);
				UPRINT(buffer);
				break;

			case MoveStates::BotAcc:
				std::sprintf((char *)&buffer[0],"\tPosL: %.2f, goalPosL: %.2f, posR: %.2f, goalPosR: %.2f, goalPosSlowR: %.2f\r\n",
					posL, goalPosiL, posR, goalPosiR, goalPosiSlowR);
				UPRINT(buffer);
				BASELEFTB.move_velocity(lspeed);
				BASERIGHTB.move_velocity(rspeed);
				BASELEFTF.move_velocity(lspeed);
				BASERIGHTF.move_velocity(rspeed);
				// acceleration to max speed takes about 20*10ms = 200ms
				if (accCount >= 20)
					g_moveState = MoveStates::BotContantSpeed;
				accCount++;
				break;
			case MoveStates::BotContantSpeed:
				std::sprintf((char *)&buffer[0],"\tPosL: %.2f, goalPosL: %.2f, posR: %.2f, goalPosR: %.2f, goalPosSlowR: %.2f\r\n",
					posL, goalPosiL, posR, goalPosiR, goalPosiSlowR);
				UPRINT(buffer);
				if (rev){ // lspeed < 0, rspeed > 0
					// logical not of condition that should continue
					stopFlag = !((posL > goalPosiL) && (posR< goalPosiR) &&	(now < timeOut));
					brakeFlag  = (posR > goalPosiSlowR);
				}else{ //lspeed > 0 , rspeed < 0
					stopFlag = !((posL < goalPosiL) && (posR > goalPosiR) && (now < timeOut));
					brakeFlag = (posL > goalPosiSlowL);
				}

				if (stopFlag)
					g_moveState = MoveStates::BotStop;
				if (brakeFlag){
					g_moveState = MoveStates::BotStopping;
				}
				break;
			case MoveStates::BotStopping:
				if (rev){
					BASELEFTB.move_velocity(lspeedSlow);
					BASERIGHTB.move_velocity(rspeedSlow);
					BASELEFTF.move_velocity(lspeedSlow);
					BASERIGHTF.move_velocity(rspeedSlow);

					std::sprintf((char *)&buffer[0],"\tBotStopping | tt: %d, L: %.2f, R: %.2f, rev\r\n",
					index, posL, posR);
					UPRINT(buffer);
					// modify velSlowLeft and velSlowRight
					if (lspeedSlow <= -20)
						lspeedSlow +=20;
					if (rspeedSlow >= 20)
						rspeedSlow -=20;
					if (rspeedSlow < 20){
						g_moveState = MoveStates::BotStop;
					}
				}else{ // forward lspeed > 0, rspeed < 0
					BASELEFTB.move_velocity(lspeedSlow); // neg if rev is true
					BASERIGHTB.move_velocity(rspeedSlow);
					BASELEFTF.move_velocity(lspeedSlow);
					BASERIGHTF.move_velocity(rspeedSlow);

					std::sprintf((char *)&buffer[0],"\tBotStopping | tt: %d, L: %.2f, R: %.2f, fwd\r\n",
					index, posL, posR);
					UPRINT(buffer);
					// modify velSlowLeft and velSlowRight
					if (lspeedSlow >= 20)
						lspeedSlow -=20;
					if (rspeedSlow <= -20)
						rspeedSlow +=20;
					if (lspeedSlow < 20){
						g_moveState = MoveStates::BotStop;
					}
				}
				break;
			case MoveStates::BotStop:
				std::sprintf((char *)&buffer[0],"\tBotStop | tt: %d, L: %.2f, R: %.2f\r\n",index, posL, posR);
				UPRINT(buffer);

				BASELEFTB.move(0);
				BASERIGHTB.move(0);
				BASELEFTF.move(0);	//needs while loop after, doesn't block prog execution
				BASERIGHTF.move(0);

				// code to ensure the bot speed is zero before while loop exit
				if (stopCount ==0){
					runFlag = true;
				}
				stopCount--;
				break;
			default:
				std::sprintf((char *)&buffer[0],"unknown Move State");
				UPRINT(buffer);
				runFlag = true;
			}

			pros::delay(10);
	}// while loop
}


void rotateYawOdom(double yawDegree, int32_t velocity, double L, int timeOut){
	// yawDegree is positive for counter-clockwise rotation
 // commented out all UPRINT(buffer) to remove debug output

	int count = 0;
	double startPosi;
	int timeOutCount = timeOut / 10;

	double angDelta_rad = yawDegree*RADIAN_PER_DEG;
	double mticks = L*angDelta_rad*TICKS_PER_INCH/2.0;

	uint8_t buffer[128];
	memset(&buffer[0],0,sizeof(buffer)); // init all elements to zero


	startPosi = BASELEFTB.get_position();
	double goalPosi = startPosi - mticks;
//	std::sprintf((char *)&buffer[0], "start: %.3f, goalPosi: %.3f, posi: %.3f\r\n",startPosi,goalPosi, BASELEFTB.get_position());
//	UPRINT(buffer);

	BASERIGHTF.move_relative(-mticks, velocity);
	BASELEFTF.move_relative(-mticks, velocity);
	BASERIGHTB.move_relative(-mticks, velocity);
	BASELEFTB.move_relative(-mticks, velocity);
	double currPos =  BASELEFTB.get_position();



	if(yawDegree > 0.0){ //turning ccw
		/*std::sprintf((char *)&buffer[0], "+yaw: %.3f, goalPosi: %.3f, posi: %.3f\r\n",
		yawDegree, goalPosi, currPos);
		UPRINT(buffer);*/
		while(currPos >= goalPosi && count < timeOutCount){
			/*std::sprintf((char *)&buffer[0], "+yaw goal: %.3f, posi: %.3f\r\n",
			goalPosi, currPos );
			UPRINT(buffer);*/
			pros::delay(10);
			currPos = BASELEFTB.get_position();
			count++;

		}
	}
	if(yawDegree < 0.0){ //turning cw
		/*std::sprintf((char *)&buffer[0], "-yaw: %.3f, goalPosi: %.3f, posi: %.3f\r\n",
		yawDegree, goalPosi, currPos);
		UPRINT(buffer);*/
		while((BASELEFTB.get_position() <= goalPosi) && (count < timeOutCount)){
			/*std::sprintf((char *)&buffer[0], "\t-yaw goal: %.3f, posi: %.3f\r\n",
			goalPosi, currPos );
			UPRINT(buffer);*/
			pros::delay(10);
			currPos = BASELEFTB.get_position();
			count++;
		}
	}
	// check for time out and print out if that is the cases
	if (count>= timeOutCount){
		std::sprintf((char *)&buffer[0], "\trotateYawOdom timeOut: yaw goal: %.3f, posi: %.3f\r\n",
		goalPosi, currPos );
		UPRINT(buffer);
	}
	pros::delay(5);
}

void testAutonDriveCorner(){
  //print buffer
	uint8_t buffer[128];
	memset(&buffer[0],0,sizeof(buffer)); // init all elements to zero

	uint32_t time = pros::millis();
	std::sprintf((char *)&buffer[0],"Time: %d\n\r", time);
	UPRINT(buffer);

	//drive forward
	double ticks = 48 * TICKS_PER_INCH;
	moveTestNoCamAssist(ticks, 200, true);
	pros::delay(100);

  //turn 90 degress to the left
	rotateYawOdom(90.0, 90, WHEEL_TO_WHEEL_LENGTH, 2000);
	pros::delay(100);

	//drive forward
	moveTestNoCamAssist(ticks, 200, false);

}
