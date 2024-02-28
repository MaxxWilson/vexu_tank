#include "main.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Robot is working");

	pros::lcd::register_btn1_cb(on_center_button);
	motorInit();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	while (true)
	{
		master.set_text(0, 1, "Ready to Start");
		master.set_text(1, 1, "Battery Level: " + std::to_string(pros::battery::get_capacity()));
		pros::delay(50);
	}
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
	setup();
	double max_speed = 0.5;
	double turning_radius = 5; // in
	// do match loading oscillations for x amount of match loads
	process_match_loads(5, max_speed);
	push_across_field(turning_radius, 15, max_speed);
	push_into_goal(turning_radius, TILE_LENGTH + 5, max_speed);

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
	bool catapultSeated = false;
	// autonomous();
	while (true)
	{
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

		bool switchPressed = catapultSwitch.get_value();
		// velocity is in rpm
		// double leftVelocity = (joystickLeftY) / 127.0 * RPM / 2;
		// double rightVelocity = (joystickRightY) / 127.0 * RPM / 2;
		// leftDrive.move_velocity(leftVelocity);
		// rightDrive.move_velocity(rightVelocity);

		// drive
		const int MAXVOLTAGE = 12000;
		double leftSpeed = (joystickLeftY + joystickRightX) / 127.0; //[0,1]
		double rightSpeed = (joystickLeftY - joystickRightX) / 127.0;//[0,1]
		double leftVoltage = leftSpeed * MAXVOLTAGE;
		double rightVoltage = rightSpeed * MAXVOLTAGE;
		leftDrive.move_voltage(leftVoltage);
		rightDrive.move_voltage(rightVoltage);

		// if (buttonA)
		// {
		// 	catapultMotor.move_velocity(100);
		// } /*else if (buttonB){
		// 	 catapultMotor.move_velocity(-100);

		//  } */
		// else
		// {
		// 	catapultMotor.brake();
		// }

		if (!switchPressed && !catapultSeated)
		{
			catapultMotor.move_velocity(100);

			pros::lcd::set_text(2, std::to_string(buttonA));
			pros::lcd::set_text(1, "catapult is up");
		}
		else
		{
			catapultSeated = true;
			if (buttonA)
			{
				catapultMotor.move_velocity(30);
				catapultSeated = false;
			}
			else
			{
				catapultMotor.brake();
			}
			pros::lcd::set_text(1, "catapult is down");
		}

		bool buttonR1 = master.get_digital(DIGITAL_R1);
		bool buttonR2 = master.get_digital(DIGITAL_R2);
		if (buttonR2)
		{
			intakeMotorA.move_velocity(150);
			intakeMotorB.move_velocity(200);
		}
		else if (buttonR1)
		{
			intakeMotorA.move_velocity(-150);
			intakeMotorB.move_velocity(-200);
		}
		else
		{
			intake.brake();
		}

		bool buttonL1 = master.get_digital(DIGITAL_L1);
		bool buttonL2 = master.get_digital(DIGITAL_L2);
		if (buttonL2)
		{
			rollerMotor.move_velocity(100);
		}
		else if (buttonL1)
		{
			rollerMotor.move_velocity(-100);
		}
		else
		{
			rollerMotor.brake();
		}

		pros::delay(20);
	}
}