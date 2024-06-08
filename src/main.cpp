#include "lemlib/api.hpp"
#include "main.hpp"
#include "robot-config.hpp"
#include "skills.cpp"

const int MAXVOLTAGE = 12000;
#define _s *1000
float errorcircleradius = 10.30776406;
std::shared_ptr<pros::Task> auton_task;

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

void printStat()
{
	printf("%05.1f: x: %.1f\" y: %.1f\" θ: %.1f°; imu: %.1f°; distance sensors: L: %.1fin B: %.1fin\n", pros::millis() / 1000., chassis_ptr->getPose().x, chassis_ptr->getPose().y, chassis_ptr->getPose().theta, imu_ptr2->get_rotation(), distance_left.get() / 25.4 + DIST_L_TO_CENTER, distance_back.get() / 25.4 + DIST_B_TO_CENTER);
}
void odomLogger()
{
	lemlib::Pose lastpose(0, 0, 0);
	unsigned long long counter = 0;
	while (true)
	{
		// print either when delta > .3 inches or > 1 deg or every 1sec
		if (chassis_ptr->getPose().distance(lastpose) > .2 || chassis_ptr->getPose().theta - lastpose.theta > 1 || !(counter % 5))
			printf("%05.1f: x: %.1f\" y: %.1f\" θ: %.1f°; imu: %.1f°; distance sensors: L: %.1fin B: %.1fin\n", pros::millis() / 1000., chassis_ptr->getPose().x, chassis_ptr->getPose().y, chassis_ptr->getPose().theta, imu_ptr2->get_rotation(), distance_left.get() / 25.4 + DIST_L_TO_CENTER, distance_back.get() / 25.4 + DIST_B_TO_CENTER);
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
		8.5, // kP
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

	chassis_ptr->setPose(15.5, 16, 45);
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

void movetobar(int safe) // 0 for skills, 2 for dri cuz george
{
	chassis_ptr->moveToPose(97, 13 + safe, -90, 2.25 _s, {lead : .40, minSpeed : 35, earlyExitRange : 5}, false);
	printf("Reached1\n");

	chassis_ptr->arcade(80, 0);
	pros::delay(3000);
	chassis_ptr->arcade(0, 0);
}

// pre: assume left and back sensors can see the wall and the robot is at the right angle for that (near the 0,0 corner)
void sensorHoming()
{
	// assume sensor perfectly perpendicular to wall and no sine/cosine adjustment
	lemlib::Pose pose = chassis_ptr->getPose();
	std::cout << "OLD POSE: " << lemlib::format_as(pose) << std::endl;
	pose.y = distance_back.get() / 25.4 + DIST_B_TO_CENTER;
	pose.x = distance_left.get() / 25.4 + DIST_L_TO_CENTER;
	std::cout << "NEW POSE: " << lemlib::format_as(pose) << std::endl;
	chassis_ptr->setPose(pose);
}
// pre: assume left and back sensors can see the wall and the robot is at the right angle for that (near the 0,0 corner)
void sensorHoming2()
{
	// assume sensor perfectly perpendicular to wall and no sine/cosine adjustment
	lemlib::Pose pose = chassis_ptr->getPose();
	std::cout << "OLD POSE: " << lemlib::format_as(pose) << std::endl;
	pose.x = 144 - (distance_back.get() / 25.4 + DIST_B_TO_CENTER);
	pose.y = (distance_left.get() / 25.4 + DIST_L_TO_CENTER);
	std::cout << "NEW POSE: " << lemlib::format_as(pose) << std::endl;
	chassis_ptr->setPose(pose);
}

void bowl_n(int n)
{
	chassis_ptr->arcade(-25, 0);
	tailPiston.set_value(true);
	pros::delay(300);
	for (int i = 0; i < n; i++)
	{
		tailMotorA.move_absolute(-120, 600);
		printf("moved to -90 * 3\n");
		pros::delay(375);
		printf("%f\n", tailMotorA.get_position());

		tailMotorA.move_absolute(120, 600);
		printf("moved to 0\n");

		pros::delay(375);
		printf("%f\n", tailMotorA.get_position());

		chassis_ptr->arcade(0, 0); // it stops going backward after one loop
	}
	tailMotorA.move_absolute(0, 100);
	pros::delay(300);

	tailPiston.set_value(false);
}
void subauton11(int bowl_num)
{
	bowl_n(bowl_num);
	intake.move_voltage(2 * MAXVOLTAGE / 3);
	chassis_ptr->turnToHeading(120, 2.5 _s, {minSpeed : 15, earlyExitRange : 10}, false);
	// wingR.set_value(true);
	chassis_ptr->moveToPoint(43, 9, 2.5 _s, {minSpeed : 25, earlyExitRange : 2}, false);

	wingL.set_value(true);
	chassis_ptr->moveToPose(96, 7, 80, 2.5 _s, {
		maxSpeed : 85,
		minSpeed : 25,
	},
							false);
	//// TMP for hypertuning commented out all of aboete
	// wingL.set_value(true);
	// chassis_ptr->setPose(96, 7, 80);

	printf("HEADING NOW \n");

	// chassis_ptr->turnToHeading(45, 1 _s, false);

	chassis_ptr->moveToPose(136, 36, 0, 2.75 _s, {forwards : true, lead : .4, maxSpeed : 70, minSpeed : 30, earlyExitRange : 2}, false);

	// chassis_ptr->moveToPose(118, 30, 26, 2.5 _s, {forwards : true, lead : .0, maxSpeed : 80}, false);

	chassis_ptr->arcade(-47, 0);
	pros::delay(300);
	chassis_ptr->arcade(127, 0);
	pros::delay(900);
	chassis_ptr->arcade(0, 0);

	wingL.set_value(false);
	chassis_ptr->tank(15, -127);

	pros::delay(600);
	chassis_ptr->arcade(0, 0);

	// wingR.set_value(false);
	chassis_ptr->turnToHeading(270, 1.75 _s, {minSpeed : 20, earlyExitRange : 2}, false);
	sensorHoming2();
}

void bowlloop(int bowl_num, int safe = 0)
{

	subauton11(bowl_num);

	printf("Reached0\n");
	chassis_ptr->moveToPose(97, 9 - safe, -85, 2.25 _s, {lead : .40, minSpeed : 35, earlyExitRange : 5}, false);
	chassis_ptr->arcade(-20, 0);
	pros::delay(400);
	chassis_ptr->arcade(0, 0);
	printf("Reached1\n");

	chassis_ptr->moveToPose(36, 14, -90, 3 _s, {minSpeed : 40, earlyExitRange : 4}, false);

	printf("reached1\n");
	chassis_ptr->moveToPose(28, 28, 0, 3 _s, {lead : 0.4, minSpeed : 20, earlyExitRange : 4}, false);

	printf("reached2\n");
	sensorHoming();
	// while (!master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN))
	//{
	//	pros::delay(10);
	// }

	printf("reached3\n");
	chassis_ptr->moveToPose(15.5, 16, 45, 3 _s, {false, minSpeed : 20, earlyExitRange : 2}, false);
	printf("reached4\n");
}

void pushmiddleballsloop()
{

	printf("HI1\n");
	tailPiston.set_value(true);
	tailMotorA.move_absolute(-90, 600);
	pros::delay(500);

	tailMotorA.move_absolute(100, 600);
	pros::delay(500);
	tailMotorA.move_absolute(00, 600);
	pros::delay(300);
	tailPiston.set_value(false);
	intake.move_voltage(-MAXVOLTAGE);
	printf("MOVING TO POSE\n");
	chassis_ptr->moveToPose(45, 54, 0, 5000, {lead : 0.6, minSpeed : 20, earlyExitRange : 7}, false);

	chassis_ptr->turnToHeading(90, 600, false); // tmp testing
												// chassis_ptr->moveToPose(46, 46, 90, 2000, {lead : 0.6}, false);

	intake.move_voltage(MAXVOLTAGE);
	wingL.set_value(1); // knocks other ball
						// wingR.set_value(true);
	pros::delay(300);	// tiny negligible i think should do
	chassis_ptr->arcade(55, 0);
	pros::delay(700);
	chassis_ptr->arcade(0, 0);
	intake.move_voltage(0);
	wingL.set_value(false);
	// wingR.set_value(false);

	// ------------------------------------------
	intake.move_voltage(-MAXVOLTAGE);
	// chassis_ptr->moveToPose(48+10, 48 + 12 + 5, -30, 5000, {lead : 0.6}, false);
	chassis_ptr->moveToPoint(48, 48, 2500, {false}, false);
	chassis_ptr->turnToHeading(0, 1000, {minSpeed : 20, earlyExitRange : 10}, false);

	chassis_ptr->moveToPose(45, 61, 0, 1500, {lead : 0.6}, false);
	chassis_ptr->arcade(-20, 0);
	pros::delay(400);
	chassis_ptr->arcade(0, 0);
	chassis_ptr->moveToPose(46, 64.5, 90, 1500, {lead : 0.6}, false);

	intake.move_voltage(MAXVOLTAGE);
	wingL.set_value(true);
	// wingR.set_value(true);
	pros::delay(300);
	chassis_ptr->arcade(55, 0);
	pros::delay(700);
	chassis_ptr->arcade(0, 0);
	intake.move_voltage(0);
	wingL.set_value(false);
	// wingR.set_value(false);

	intake.move_voltage(MAXVOLTAGE);
	// ------------
	chassis_ptr->moveToPose(15.5, 16, 45, 3000, {false}, false);
}

void pushmiddleballsloop_skills()
{

	printf("HI1\n");
	tailPiston.set_value(true);
	tailMotorA.move_absolute(-90, 600);
	pros::delay(500);

	tailMotorA.move_absolute(100, 600);
	pros::delay(500);
	tailMotorA.move_absolute(00, 600);
	pros::delay(300);
	tailPiston.set_value(false);
	intake.move_voltage(-MAXVOLTAGE);
	printf("MOVING TO POSE\n");

	chassis_ptr->moveToPose(42, 75, 0, 3500, {lead : 0.6, minSpeed : 15, earlyExitRange : 4}, false);
	chassis_ptr->turnToHeading(90, 1000, {minSpeed : 15, earlyExitRange : 10}, false);

	wingL.set_value(true);
	intake.move_voltage(MAXVOLTAGE);
	chassis_ptr->arcade(-50, 0);
	pros::delay(350);
	chassis_ptr->arcade(75, 0);
	pros::delay(800);
	chassis_ptr->arcade(0, 0);
	intake.move_voltage(0);
	wingL.set_value(false);
	// wingR.set_value(false);

	intake.move_voltage(MAXVOLTAGE);
	// ------------

	// chassis_ptr->moveToPose(24, 24, -90, 3 _s, {forwards: false, lead: -.6, minSpeed : 20, earlyExitRange : 4}, false);

	// sensorHoming();
	chassis_ptr->moveToPose(15.5, 16, 45, 3000, {false}, false);
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

void release_intake_fold()
{
	intake.move_voltage(-MAXVOLTAGE);
	pros::delay(300);
	intake.move_voltage(0);
}

void climbatend()
{
	// wait till the end of driver control, and close lift no matter what, saves us in case of half climb
	pros::delay(74.5 _s);
	lift.set_value(0);
}

void climbatendskills()
{
	pros::delay(59.7 _s);
	lift.set_value(0);
}

void actual_auton()
{

	static bool first = false;
	if (skills) // skills
	{

		pros::Task whatev(climbatendskills);
		release_intake_fold();
		bowlloop(5);
		bowlloop(7);
		subauton11(7);
		lift.set_value(1);
		movetobar(0);
		//lift.set_value(0);
	}
	else
	{
		if (!first) // isolation mode
		{
			first = true;
			printf("===========================START AUTON======================\n");
			printStat();
			release_intake_fold();
			pushmiddleballsloop();
			bowlloop(8);
			bowl_n(10);

			printf("===========================END AUTON======================\n");
			printStat();
		}
		else // interactive mode
		{
			pros::Task whatev(climbatend);

			printf("===========================START DRIVE======================\n");
			printStat();

			bowlloop(3); // already have some from auton, don't need that many more in driver
			bowlloop(6);
			//{ // one option, delay and then bowl
			//	pros::delay(15 _s);
			//	subauton11(6);
			//}
			//{ // other opt, save time, bowl while george climbing, and then push them after
			// // untested, but likely to work code
			//	pros::Task task{[=]
			//					{
			//						bowl_n(6);
			//					}};
			//	pros::delay(15 _s);
			//	subauton11(0);
			//}
			{ // no george so just run
				subauton11(6);
			}
			lift.set_value(1);
			movetobar(0);
			//lift.set_value(0);

			printf("===========================END DRIVE======================\n");
			printStat();
		}
	}
}
void autonomous()
{
	actual_auton();
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
	unsigned willie_driving = 1;

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
			willie_driving = (willie_driving + 1) % 3;
			pros::delay(200);
		}

		// drive
		double leftSpeed = joystickLeftY / 127.0;	// [0,1]
		double rightSpeed = joystickRightY / 127.0; // [0,1]
		leftSpeed *= leftSpeed * leftSpeed / fabs(leftSpeed);
		rightSpeed *= rightSpeed * rightSpeed / fabs(rightSpeed);
		if (willie_driving == 1)
		{
			leftSpeed = (joystickLeftY + joystickRightX) / 127.0;  // [0,1]
			rightSpeed = (joystickLeftY - joystickRightX) / 127.0; // [0,1]
		}
		else if (willie_driving == 2)
		{
			leftSpeed = (joystickLeftY + joystickLeftX) / 127.0;  // [0,1]
			rightSpeed = (joystickLeftY - joystickLeftX) / 127.0; // [0,1]
		}
		double leftVoltage = leftSpeed * MAXVOLTAGE;
		double rightVoltage = rightSpeed * MAXVOLTAGE;

		// wingR.set_value(master.get_digital(DIGITAL_R1));
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
		double tail_kp = 0.0;
		double tail_kd = 0.0;
		if (buttonleft && buttondown)
		{
			tail_kp = 3.0 * 12000.0 / 90.0;
			tail_kd = 0.0;
			tailPiston.set_value(true);
			intake.move_voltage(0);
			tail_setpoint = -90.0;
			if (buttonR2)
			{
				tail_setpoint = 110.0;
			}
		}
		else
		{
			tail_kp = 1.5 * 12000.0 / 90.0;
			tail_kd = 0.25 * 12000.0 / 100.0;
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