#include "main.hpp"
#include "robot-config.hpp"
#include "lemlib/api.hpp"

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
	while (true)
	{
		std::cout << chassis_ptr->getPose().x << " " << chassis_ptr->getPose().y << "  " << chassis_ptr->getPose().theta << "deg imu: " << imu_ptr->get_rotation() << "deg   "<<imu_ptr2->get_rotation() << "deg" << std::endl;
		pros::delay(300);
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
		10.75,			 // change to actual track width
		3.25,		 // wheel diameter
		360,		 // wheel rpm
		1,			 // chasepower default
	};

	lemlib::ControllerSettings lateralController{
		32,	 // kP
		0, // ki
		4,	 // kD
			0, //windup range is what??
		0.5,	 // smallErrorRange
		100, // smallErrorTimeout
		1,	 // largeErrorRange
		500, // largeErrorTimeout
		5	 // slew rate
	};

	// turning PID
	lemlib::ControllerSettings angularController{
		4,	 // kP
		0, // ki
		40,	 // kD
		0, //winuprange is what
		1,	 // smallErrorRange
		100, // smallErrorTimeout
		3,	 // largeErrorRange
		500, // largeErrorTimeout
		0	 // slew rate
	};

	encoder_ptr = std::make_shared<pros::ADIEncoder>('H', 'G', true);
	trackingwheel_ptr = std::make_shared<lemlib::TrackingWheel>(encoder_ptr.get(), 3.25, 0, 1);
	imu_ptr = std::make_shared<pros::Imu>(10);
		imu_ptr2 = std::make_shared<pros::Imu>(14);

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
	//imu_ptr->reset(  false);
	//imu_ptr2->reset( true);


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

using namespace Auton;
void autonomous()
{
	//chassis_ptr->moveToPoint(-24, 24, 100000, {}, false );
	chassis_ptr->moveToPoint(-24, 0, 100000, {}, false );
	 chassis_ptr->moveToPoint(-24, 24, 100000, {}, false );
	chassis_ptr->moveToPoint(0, 24, 100000, {}, false );
	chassis_ptr->moveToPose(0, 0,0, 100000, {}, false );

	//chassis_ptr->turnToHeading(0,100000, {}, false );
	//chassis_ptr->moveToPose(0, 0, 0,100000, {}, false );
	//chassis_ptr->moveToPose(0, -24, 0,100000, {}, false );


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
void opcontrol()
{
	// printf("hi\n");
	// pros:: delay (20);
	bool catapultSeated = false;
	double last_climb_switch_time = pros::millis();
	bool willie_driving = true;

	while (true)
	{

		// autonomous();
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		// 				 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		// 				 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int deadzone = 20;
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

		if (buttonA && buttonB && willie_driving)
		{
			willie_driving = false;
		}

		// drive
		const int MAXVOLTAGE = 12000;
		double leftSpeed = joystickLeftY / 127.0;	// [0,1]
		double rightSpeed = joystickRightY / 127.0; // [0,1]
		if (willie_driving)
		{
			leftSpeed = (joystickLeftY + joystickRightX) / 127.0;  // [0,1]
			rightSpeed = (joystickLeftY - joystickRightX) / 127.0; // [0,1]
		}
		double leftVoltage = leftSpeed * MAXVOLTAGE;
		double rightVoltage = rightSpeed * MAXVOLTAGE;

		wingR.set_value(master.get_digital(DIGITAL_R1));

		bool buttonL2 = master.get_digital(DIGITAL_L2);
		bool buttonR2 = master.get_digital(DIGITAL_R2);
		if (buttonL2 && buttonR2)
		{
			leftVoltage *= 0.5;
			rightVoltage *= 0.5;
			leftVoltage = rightVoltage;
			intake.brake();
		}
		else if (buttonL2)
		{
			intakeMotorA.move_voltage(12000);
		}
		else if (buttonR2)
		{
			intakeMotorA.move_voltage(-12000);
		}
		else
		{
			intake.brake();
		}

		leftDrive.move_voltage(leftVoltage);
		rightDrive.move_voltage(rightVoltage);

		static bool climbed = false;

		if (master.get_digital(DIGITAL_L1))
		{
			if (climb_switch.get_value() && !climbed)
			{
				lift.set_value(false);
				climbed = true;
				last_climb_switch_time = pros::millis();
			}
			else if (pros::millis() - last_climb_switch_time > 3000)
			{
				lift.set_value(true);
				climbed = false;
			}
		}
		else if (master.get_digital(DIGITAL_DOWN))
		{
			lift.set_value(true);
		}
		else
		{
			lift.set_value(false);
		}

		if (master.get_digital(DIGITAL_UP) && master.get_digital(DIGITAL_LEFT) && master.get_digital(DIGITAL_RIGHT))
		{
			autonomous();
		}

		pros::delay(20);
	}
}