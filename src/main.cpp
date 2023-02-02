#include "main.h"
#include "demos.h"

//port declarations
#define BASE_LEFT_FRONT_PORT 6
#define BASE_LEFT_BACK_PORT 8
#define BASE_RIGHT_FRONT_PORT 5
#define BASE_RIGHT_BACK_PORT 7

#define OPTICAL_PORT 17
#define OPTICAL_FRONT_PORT 16

//motor declarations
pros::Motor BASELEFTF(BASE_LEFT_FRONT_PORT);	//green 18:1
pros::Motor BASELEFTB(BASE_LEFT_BACK_PORT);	//green 18:1
pros::Motor BASERIGHTF(BASE_RIGHT_FRONT_PORT);	//green 18:1, reversed
pros::Motor BASERIGHTB(BASE_RIGHT_BACK_PORT);	//green 18:1, reversed

pros::Optical optical_sensor(OPTICAL_PORT);
pros::Optical optical_front_sensor(OPTICAL_FRONT_PORT);

volatile uint32_t btnUp_CTime = pros::millis();
volatile uint32_t btnUp_GTime = pros::millis();
volatile uint32_t btnX_CTime = pros::millis();
volatile uint32_t btnX_GTime = pros::millis();

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	// high speed print
	vexGenericSerialEnable( SERIALPORT - 1, 0 );// Start serial on desired port
	vexGenericSerialBaudrate( SERIALPORT - 1, DEBUG_BAUD_RATE ); // Set BAUD rate
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	testAutonDriveCorner();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	uint8_t buffer[128];
	memset(&buffer[0],0,sizeof(buffer)); // init all elements to zero
	uint32_t count = 0;

	double throttleRatio = 0;
	double steeringRatio = 0;
	double radian2angle = 180/(atan(1)*4.0); // denominator is pi
	double angle2radian = 1/radian2angle;
	double tick2turn = 0;
	bool runflag = false;


	int right = master.get_analog(ANALOG_RIGHT_Y);
	int left = master.get_analog(ANALOG_LEFT_Y);

	pros::Motor *lf = &BASELEFTF;
	pros::Motor *lb = &BASELEFTB;
	pros::Motor *rf = &BASERIGHTF;
	pros::Motor *rb = &BASERIGHTB;

	bool printflag = false;
	memset(&buffer[0],0,128);// init to zero
	memset(&buffer[0],8,32); // send zero odom
	UPRINT(buffer);

	while (true) {

#if TANK_DRIVE == 1
		right = master.get_analog(ANALOG_RIGHT_Y);
		left = master.get_analog(ANALOG_LEFT_Y);

		rb->move(-right);
		lb->move(left);
		pros::delay(10);

#endif

#if SINGLE_CHANNEL_DRIVE == 1// single channel drive

		double rightSpeedRatio = 0;
		double rightSpeed =0;
		double leftSpeedRatio =0;
		double leftSpeed = 0;

		double right_y = (double)master.get_analog(ANALOG_RIGHT_Y);
		double right_x = (double)master.get_analog(ANALOG_RIGHT_X);

		if (( right_y < 0) && (right_y >=-10.0)){
			right_y = -right_y;
		}else if (( right_y < -10.) && (right_y >=-15.0)){
			right_y = 0.0;
		}
		int right_y_ind = (int)right_y + 127;
		int right_x_ind = (int)right_x + 127;

		double angle =  atan2(right_y,right_x)*radian2angle;

		double radius = sqrt(pow(right_y,2)+ pow(right_x,2));

		if (radius >127.0 ){
			radius = 127.0;
		}
		// forward right turn, first quadrant
		if (right_y >=0 && right_x >=0){
			left = (int)radius; //
			if (angle < REVERSE_TURN_ANGLE){
					right = -(int)left;
			}
			else{
				throttleRatio =radius/127.0;

				steeringRatio = (90.0-angle)/90.0;

				rightSpeedRatio = throttleRatio*(1-steeringRatio);
				rightSpeed = rightSpeedRatio*127.0;
				right = (int)rightSpeed;
			}
		}
		// forward left turn, second quadrant
		if (right_y >=0 && right_x < 0){
			right = (int)radius; //
			if(angle > 180.0 - REVERSE_TURN_ANGLE){
				left = -(int)right;
			}
			else{
				throttleRatio =radius/127.0;
				// atan computes to larger than 90degrees for 2nd quadrant

				steeringRatio = (angle-90.0)/90.0; // steering ratio is positive

				leftSpeedRatio = throttleRatio*(1-steeringRatio);
				leftSpeed = leftSpeedRatio*127.0;
				left = (int)leftSpeed;
			}
		}
		// backward left turn, 3rd quadrant
		if ((right_y < 0 ) && (right_x <= 0)){
			right = -(int)radius; //
			if(angle < -180 + REVERSE_TURN_ANGLE){
				left = -(int)right;
			}
			else{
				throttleRatio =radius/127.0;
				// atan computes to lesser than -90degrees for 3rd quadrant

				steeringRatio = (-90.0- angle)/90.0; // steering ratio is positive

				leftSpeedRatio = throttleRatio*(1-steeringRatio);
				leftSpeed = leftSpeedRatio*127.0;
				left = -(int)leftSpeed;
			}
		}
		// backward right turn, 4th quadrant
		if (right_y < 0 && right_x >=0){
			left = -(int)radius; //
			if(angle > -REVERSE_TURN_ANGLE){
				right = -(int)left;
			}
			else{
				throttleRatio =radius/127.0;

				// angle is -90.0 < angle < 0
				steeringRatio = (90.0 + angle)/90.0;

				rightSpeedRatio = throttleRatio*(1-steeringRatio);
				rightSpeed = rightSpeedRatio*127.0;
				right = -(int)rightSpeed;
			}
		}
	pros::delay(10);
#endif
	count++;
	rf->move(-right);
	lf->move(left);
	rb->move(-right);
	lb->move(left);

	// button processing
	double ticksRight = rf->get_position();
	double ticksLeft = lf->get_position();
	uint64_t timeStamp = pros::micros();

		if(master.get_digital(DIGITAL_UP)){
			btnUp_CTime = pros::millis();
			if(btnUp_CTime >= btnUp_GTime){
				testAutonDriveCorner();

				btnUp_GTime = pros::millis() + BUTTON_TIME_LOCK;
			}
		}

		if(master.get_digital(DIGITAL_X)){ // testing, can be free up
			btnX_CTime = pros::millis();
			if(btnX_CTime >= btnX_GTime){
				printflag = ~printflag;
				//std::sprintf((char *)&buffer[0], "\tpos: %.3f, %.3f\r\n",ticksLeft,ticksRight);
				//std::sprintf((char *)&buffer[0], "%s","\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
				memset(&buffer[0],0,128);
				memset(&buffer[0],8,32);
				UPRINT(buffer);
				count++;
				btnX_GTime = pros::millis() + BUTTON_TIME_LOCK;
			}
		}// if

		if (printflag) { //(printflag)
			//std::sprintf((char *)&buffer[0], "\t%llu, pos: %.3f, %.3f\r\n",timeStamp, ticksLeft,ticksRight);
			std::sprintf((char *)&buffer[0], "\tpos: %.3f, %.3f\r\n",ticksLeft,ticksRight);
			//std::sprintf((char *)&buffer[0], "0123");
			UPRINT(buffer);
		}

	}//while
}
