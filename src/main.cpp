#include "lemlib/api.hpp"
#include "main.hpp"
#include "robot-config.hpp"

const int MAXVOLTAGE = 12000;
#define _s *1000
float errorcircleradius = 10.30776406;
std::shared_ptr<pros::Task> auton_task;

unsigned int auton_start_time = 0;

void checkPosition()
{
	while ((chassis_ptr->getPose().y + errorcircleradius) < 72)
	{
		pros::delay(10);
	}
	printf("KILINNG============================================================\n");
	printf("KILINNG============================================================\n");
	printf("KILINNG============================================================\n");
	printf("KILINNG============================================================\n");
	printf("KILINNG============================================================\n");
	printf("KILINNG============================================================\n");
	auton_task->remove();
	while (pros::competition::is_autonomous())
	{
		motorInit();
		rightDrive.brake();
		leftDrive.brake();
		pros::delay(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// Odom Code

// lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// void screen() {
//     // loop forever
//     while (true) {
//        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
//        pros::lcd::print(0, "x: %f", pose.x); // print the x position
//         pros::lcd::print(1, "y: %f", pose.y); // print the y position
//         pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
//         pros::delay(10);
//     }
// }

void odomLogger()
{
	lemlib::Pose lastpose(0, 0, 0);
	unsigned long long counter = 0;
	while (true)
	{
		// print either when delta > .3 inches or > 1 deg or every 1sec
		if (chassis_ptr->getPose().distance(lastpose) > .2 || chassis_ptr->getPose().theta - lastpose.theta > 1 || !(counter % 5))
		{
			std::cout << pros::millis() / 1000. <<  "s " << chassis_ptr->getPose().x << "in " << chassis_ptr->getPose().y << "in  " << chassis_ptr->getPose().theta << "deg imu: " << imu_ptr2->get_rotation() << "deg" << std::endl;
		}
		counter++;
		lastpose = chassis_ptr->getPose();
		pros::delay(200);
	}
}

void initialize()
{
	printf("hi\n");
	// printf("hi1\n");
	// printf("hi2\n");

	//  pros::lcd::initialize();
	//          pros::lcd::print(1, "x: %f", 69.420); // print the x position
	// 		// pros::lcd::print(3, "y: %s", "Why am I here, just to suffer?");

	// pros::ADIEncoder enc('A', 'B', true); // ports A and B, reversed
	// pros::Rotation rot(1, false); // port 1, not reversed

	// pros::Imu inertial_sensor(6); //replacd that 2 with actual port IMU is connected to

	// lemlib::TrackingWheel tracking_wheel(&leftDrive, 3.25, 5.5, RPM);

	lemlib::Drivetrain drivetrain{
		&leftDrive,	 // left drivetrain motors
		&rightDrive, // right drivetrain motors
		10.75,		 // change to actual track width
		3.25,		 // wheel diameter
		360,		 // wheel rpm
		300000,		 // chasepower default
	};

	lemlib::ControllerSettings lateralController{
		7.5, // kP
		0,	 // ki
		0,	 // kD
		0,	 // windup range is what??
		0.6, // smallErrorRange
		100, // smallErrorTimeout
		2,	 // largeErrorRange
		500, // largeErrorTimeout
		0	 // slew rate
	};

	// turning PID
	lemlib::ControllerSettings angularController{
		2.1,  // kP
		0,	  // ki
		25.6, // kD
		0,	  // winuprange is what
		1,	  // smallErrorRange
		100,  // smallErrorTimeout
		5,	  // largeErrorRange
		500,  // largeErrorTimeout
		5	  // slew rate
	};

	encoder_ptr = std::make_shared<pros::ADIEncoder>('H', 'G', true);
	trackingwheel_ptr = std::make_shared<lemlib::TrackingWheel>(encoder_ptr.get(), 3.25, 0, 1);
	imu_ptr2 = std::make_shared<pros::Imu>(18);

	lemlib::infoSink()->setLowestLevel(lemlib::Level::WARN);
	lemlib::OdomSensors sensors{
		trackingwheel_ptr.get(),
		nullptr,
		nullptr,
		nullptr,
		imu_ptr2.get()};
	printf("lemlib calibrating\n");
	chassis_ptr = std::make_shared<lemlib::Chassis>(drivetrain, lateralController, angularController, sensors);

	chassis_ptr->calibrate(true);
	printf("lemlib calibrating done\n");
	// imu_ptr->reset(  false);
	// imu_ptr2->reset( true);

	pros::Task odomLog(odomLogger);
	//	pros::Task screenTask(screen);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	// while(true){
	//	master.set_text(0, 1, "Ready to Start");
	//	master.set_text(1, 1, "Battery Level: " + std::to_string(pros::battery::get_capacity()));
	//	pros::delay(50);
	// }
}

void movetobar()
{
	chassis_ptr->turnToHeading(-105, 1.25 _s, false);
	chassis_ptr->moveToPose(100, 16, -99, 3 _s, {}, false);
	chassis_ptr->moveToPose(64, 16, -90, 8 _s, {}, false);
}

void auton1()
{
	const int go_back_after_time = 41 _s;

	intake.move_voltage(MAXVOLTAGE);
	chassis_ptr->arcade(-15, 0);
	pros::delay(12 _s);
	//	tailPiston.set_value(true);
	// for(int i = 0; i < 12; i++){
	//	tailMotorA.move_absolute(-70 , 100);
	//	printf("moved to -90 * 3\n");
	//	pros::delay(500);
	//	printf("%f\n", tailMotorA.get_position());

	//	tailMotorA.move_absolute(50, 100);
	//	printf("moved to 0\n");

	//	pros::delay(500);
	//	printf("%f\n", tailMotorA.get_position());

	chassis_ptr->arcade(0, 0); // it stops going backward after one loop
	//}
	//	tailMotorA.move_absolute(0, 100);
	//	tailPiston.set_value(false);
	chassis_ptr->turnToHeading(120, 2.5 _s, false);
	wingR.set_value(true);
	chassis_ptr->moveToPoint(43, 15, 2.5 _s, {}, false);
	chassis_ptr->moveToPoint(96 - 4, 15, 2.5 _s, {maxSpeed: 70}, false);
	wingL.set_value(true);

	printf("HEADING NOW \n");

	//chassis_ptr->turnToHeading(45, 1 _s, false);
	chassis_ptr->moveToPose(118, 30, 26, 2.5 _s, {lead: .0}, false);
	chassis_ptr->arcade(127, 0);
	pros::delay(1000);
	chassis_ptr->arcade(0, 0);

if ((pros::millis() - auton_start_time) < (go_back_after_time - 2.5 _s)){
	chassis_ptr->moveToPose(118, 19, 26, .9 _s, {false}, false);
	chassis_ptr->moveToPose(128, 36, 26 + 180 - 1, .9 _s, {true}, false);
	chassis_ptr->arcade(-127, 0);
	pros::delay(800);
	chassis_ptr->arcade(0, 0);
}
if ((pros::millis() - auton_start_time) < (go_back_after_time - 2.5 _s)){

	chassis_ptr->turnToHeading(26 + 360 - 1 - 1, 1.25 _s, false);
	chassis_ptr->arcade(127, 0);
	pros::delay(800);
	chassis_ptr->arcade(0, 0);
}

	wingL.set_value(false);
	wingR.set_value(false);
	movetobar();
}

void auton2()
{
	// chassis_ptr->setPose(15.5,16,45);
	// chassis_ptr->turnToHeading()
	printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");
	intake.move_voltage(-MAXVOLTAGE);
	chassis_ptr->moveToPose(45, 60, 15, 100000, {}, false);
	printf("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\n");
	// chassis_ptr->moveToPoint(48, 60, 100000, {}, false);
	intake.move_voltage(0);
	wingR.set_value(true);
	wingL.set_value(true);
	// open wings
	chassis_ptr->moveToPoint(50, 44, 100000, {}, false);
	wingR.set_value(false);
	wingL.set_value(false);
	// close wings
	chassis_ptr->moveToPose(15.5, 16, 45, 10000, {false}, false);
	chassis_ptr->arcade(-40, 0);
	pros::delay(2000);
	chassis_ptr->arcade(0, 0);
	auton1();
}

void auton3()
{
	intake.move_voltage(-MAXVOLTAGE);
	chassis_ptr->moveToPose(45, 54, 45, 5000, {lead : 0.6, minSpeed : 20, earlyExitRange : 7}, false);

	chassis_ptr->turnToPoint(50, 44, 3000, {}, false);
	chassis_ptr->moveToPoint(50, 44, 3000, {}, false);
	intake.move_voltage(MAXVOLTAGE);
	chassis_ptr->arcade(-10, 0);
	pros::delay(1000);
	chassis_ptr->arcade(0, 0);
	intake.move_voltage(0);

	intake.move_voltage(-MAXVOLTAGE);
	chassis_ptr->turnToPoint(47, 62, 3000, {}, false);
	chassis_ptr->moveToPoint(47, 62, 3000, {}, false);

	chassis_ptr->turnToPoint(50, 44, 3000, {}, false);
	chassis_ptr->moveToPoint(50, 44, 3000, {}, false);
	intake.move_voltage(MAXVOLTAGE);
	chassis_ptr->arcade(-10, 0);
	pros::delay(1000);
	chassis_ptr->arcade(0, 0);
	intake.move_voltage(0);

	intake.move_voltage(-MAXVOLTAGE);
	chassis_ptr->moveToPose(48 + 7, 56, 75, 3000, {}, false);

	chassis_ptr->moveToPose(50, 44, 135, 3000, {lead : 0.6, minSpeed : 20, earlyExitRange : 4}, false);
	chassis_ptr->turnToPoint(50, 44, 3000, {}, false);
	chassis_ptr->moveToPoint(50, 44, 3000, {}, false);

	intake.move_voltage(MAXVOLTAGE);
	chassis_ptr->arcade(-10, 0);
	pros::delay(1000);
	chassis_ptr->arcade(0, 0);
	intake.move_voltage(0);

	intake.move_voltage(-MAXVOLTAGE);
	chassis_ptr->moveToPose(48 + 10, 62, 45, 3000, {lead : .02}, false);

	chassis_ptr->moveToPose(50, 44, 135, 3000, {lead : .02}, false);
	intake.move_voltage(MAXVOLTAGE);

	//	chassis_ptr->moveToPose(36, 36, 90, 3000, {lead : .6}, false);
	//
	//	wingR .set_value( true);
	//	intake.move_voltage(0);

	//	chassis_ptr->moveToPoint(36 + 24, 36, 3000, {}, false);

	chassis_ptr->moveToPose(15.5, 16, 45, 3000, {false}, false);
	chassis_ptr->arcade(-40, 0);
	pros::delay(500);
	chassis_ptr->arcade(0, 0);
}

void auton4()
{
	intake.move_voltage(-MAXVOLTAGE);
	chassis_ptr->moveToPose(45, 54, 0, 5000, {lead : 0.6, minSpeed : 20, earlyExitRange : 7}, false);

	chassis_ptr->turnToHeading(90, 600, false); // tmp testing
	//chassis_ptr->moveToPose(46, 46, 90, 2000, {lead : 0.6}, false);

	intake.move_voltage(MAXVOLTAGE);
	wingL.set_value(false); // knocks other ball
	wingR.set_value(true);
	pros::delay(300); // tiny negligible i think should do
	chassis_ptr->arcade(55, 0);
	pros::delay(700);
	chassis_ptr->arcade(0, 0);
	intake.move_voltage(0);
	wingL.set_value(false);
	wingR.set_value(false);

	// ------------------------------------------
	intake.move_voltage(-MAXVOLTAGE);
	// chassis_ptr->moveToPose(48+10, 48 + 12 + 5, -30, 5000, {lead : 0.6}, false);
	chassis_ptr->moveToPoint(48, 48, 2500, {false}, false);
	chassis_ptr->turnToHeading(0, 1000, {minSpeed: 1, earlyExitRange: 10}, false);

	chassis_ptr->moveToPose(45, 61, 0, 1500, {lead : 0.6 }, false);
	chassis_ptr->arcade(-20, 0);
	pros::delay(400);
	chassis_ptr->arcade(0, 0);
	chassis_ptr->moveToPose(46, 64.5, 90, 1500, {lead : 0.6}, false);

	intake.move_voltage(MAXVOLTAGE);
	wingL.set_value(true);
	wingR.set_value(true);
	pros::delay(300);
	chassis_ptr->arcade(55, 0);
	pros::delay(700);
	chassis_ptr->arcade(0, 0);
	intake.move_voltage(0);
	wingL.set_value(false);
	wingR.set_value(false);

	intake.move_voltage(MAXVOLTAGE);
	// ------------
	chassis_ptr->moveToPose(15.5 - 2, 16, 45, 3000, {false}, false);


	auton1();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
}

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
void actual_auton()
{

	auton_start_time = pros::millis(); 
	chassis_ptr->setPose(15.5, 16, 45);

	// auton1();
	// return;

	// chassis_ptr->moveToPose(45, 54, 45, 10000, {lead : 0.2}, false);

	// chassis_ptr->turnToPoint(20, 20, 10000, {}, false);
	// chassis_ptr->moveToPoint(20, 20, 10000, {}, false);
	// chassis_ptr->turnToPoint(30, 30, 10000, {}, false);
	// chassis_ptr->moveToPoint(30, 30, 10000, {}, false);

	// return;
	//  chassis_ptr->moveToPose(16, 80, 45, 10000, {false}, false);
	//  auton1();
	//  auton2();
	// auton3();
	auton1();
}
ASSET(path_txt)
using namespace Auton;
void autonomous()
{

	pros::Task t(checkPosition);
	auton_task = std::make_shared<pros::Task>(actual_auton);
}

// Make a function for setting the geofencing parameters
// Which x/y coordinates can it not pass?
// Check with the odometry x/y values
// if it's approaching, then go a

// stops robot movement if its detected that its has crossed tape

// void avoida(){
// 	if ((chassis_ptr->getPose().y)+ errorcircleradius) >48 && ){
// }

// }

// if anywhere there is point where center y coordinate is greater than 72

// float errorcircle = 2*M_PI*errorcircleradius;

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
void opcontrol()
{
	// printf("hi\n");
	// pros:: delay (20);
	double last_climb_switch_time = pros::millis();
	bool willie_driving = true;

	while (true)
	{
		// autonomous();
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		// 				 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		// 				 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int deadzone = 7;
		int joystickLeftY = master.get_analog(ANALOG_LEFT_Y);
		if (abs(joystickLeftY) < deadzone)
		{
			joystickLeftY = 0;
		}
		int joystickLeftX = master.get_analog(ANALOG_LEFT_X);
		if (abs(joystickLeftX) < deadzone)
		{
			joystickLeftX = 0;
		}
		int joystickRightY = master.get_analog(ANALOG_RIGHT_Y);
		if (abs(joystickRightY) < deadzone)
		{
			joystickRightY = 0;
		}
		int joystickRightX = master.get_analog(ANALOG_RIGHT_X);
		if (abs(joystickRightX) < deadzone)
		{
			joystickRightX = 0;
		}
		bool buttonA = master.get_digital(DIGITAL_A);
		bool buttonB = master.get_digital(DIGITAL_B);

		if (buttonA && buttonB)
		{
			willie_driving ^= 1;
			pros::delay(200);
		}

		// drive
		double leftSpeed = joystickLeftY / 127.0;	// [0,1]
		double rightSpeed = joystickRightY / 127.0; // [0,1]
		leftSpeed *= leftSpeed * leftSpeed / fabs(leftSpeed);
		rightSpeed *= rightSpeed * rightSpeed / fabs(rightSpeed);
		if (willie_driving)
		{
			leftSpeed = (joystickLeftY + joystickRightX) / 127.0;  // [0,1]
			rightSpeed = (joystickLeftY - joystickRightX) / 127.0; // [0,1]
		}
		double leftVoltage = leftSpeed * MAXVOLTAGE;
		double rightVoltage = rightSpeed * MAXVOLTAGE;

		wingR.set_value(master.get_digital(DIGITAL_R1));
		wingL.set_value(master.get_digital(DIGITAL_L1));

		bool buttonL2 = master.get_digital(DIGITAL_L2);
		bool buttonR2 = master.get_digital(DIGITAL_R2);
		bool buttonleft = master.get_digital(DIGITAL_LEFT);
		bool buttondown = master.get_digital(DIGITAL_DOWN);
		// tailPiston.set_value(false);
		//  if (buttonL2 && buttonR2)
		//  {
		//  	leftVoltage *= 0.5;
		//  	rightVoltage *= 0.5;
		//  	leftVoltage = rightVoltage;
		//  	intake.brake();
		//  }
		double tail_setpoint = 0.0;
		const double tail_kp = 2.5 * 12000.0 / 90.0;
		const double tail_kd = 1.0 * 12000.0 / 100.0;
		if (buttonleft && buttondown)
		{
			tailPiston.set_value(true);
			intake.move_voltage(0);
			tail_setpoint = -90.0;
			if (buttonR2)
			{
				tail_setpoint = 90.0;
			}
		}
		else
		{
			tailPiston.set_value(false);
			if (buttonL2)
			{
				// TODO WHEN LAST BUTTON PRESSED INTAKE IN, ACTIVATE INTAKE INWARDS, RIGHT IS INWARDS RN
				intake.move_voltage(12000);
			}
			else if (buttonR2)
			{
				intake.move_voltage(-12000);
			}
			else
			{
				intake.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
				intake.brake();
			}
		}
		auto voltage_cmd = tail_kp * (tail_setpoint - tailMotorA.get_position()) - tail_kd * (tailMotorA.get_actual_velocity());
		tailMotorA.move_voltage(voltage_cmd);

		leftDrive.move_voltage(leftVoltage);
		rightDrive.move_voltage(rightVoltage);

		static bool climbed = false;
		static bool lift_up = false;

		if (master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_Y))
		{
			lift_up = true;

			// if(climb_switch.get_value() && !climbed){
			// 	lift.set_value(false);
			// 	climbed = true;
			// 	last_climb_switch_time = pros::millis();
			// }
			// else if(pros::millis() - last_climb_switch_time > 3000){
			// 	lift.set_value(true);
			// 	climbed = false;
			// }
		}
		// else if(master.get_digital(DIGITAL_DOWN)){
		// 	lift.set_value(true);
		// }
		else
		{
			lift_up = false;
		}
		lift.set_value(lift_up);

		bool do_auto = master.get_digital(DIGITAL_UP) && master.get_digital(DIGITAL_LEFT) && master.get_digital(DIGITAL_RIGHT);
		static bool last_do_auto = false;
		if (do_auto && !last_do_auto)
		{
			last_do_auto = true;
			actual_auton();
		}
		else if (!do_auto)
		{
			last_do_auto = false;
		}

		pros::delay(20);
	}
}