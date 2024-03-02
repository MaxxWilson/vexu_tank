#include "main.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button(){
	static bool pressed = false;
	pressed = !pressed;
	if(pressed){
		pros::lcd::set_text(2, "I was pressed!");
	}
	else{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize(){
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
void disabled(){
	while(true){
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
void competition_initialize(){
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
void autonomous(){
	setup();
	intakeMotorA.move_voltage(12000);
	lift.set_value(true);
	pros::delay(100);
	lift.set_value(false);
	pros::delay(100);
	wingR.set_value(true);
	pros::delay(500);
	intakeMotorA.move_voltage(500);
	process_match_loads(22);
	wingR.set_value(false);
	driveForwardCubic(-2, 0.2);
	pros::delay(500);
	arcTurnPD(80.0, 2.0, turn_direction_e_t::RIGHT, 5.0);
	pros::delay(500);
	driveForwardCubic(10, 0.5);
	pros::delay(500);
	intakeMotorA.move_voltage(-12000);
	pros::delay(500);
	leftDrive.move_voltage(10000);
	rightDrive.move_voltage(10000);
	pros::delay(250);
	leftDrive.move_voltage(0);
	rightDrive.move_voltage(0);
	pros::delay(500);
	driveForwardCubic(-8, 0.3);
	pros::delay(500);
	arcTurnPD(-56.0, 1.5, turn_direction_e_t::RIGHT, 3.0);
	pros::delay(500);
	// wingR.set_value(true);
	driveForwardCubic(-8, 0.4);
	pros::delay(500);
	arcTurnPD(-56.0, 1.5, turn_direction_e_t::RIGHT, 3.0);
	pros::delay(500);
	driveForwardCubic(-10, 0.4);
	pros::delay(500);
	arcTurnPD(-10.0, 0.5, turn_direction_e_t::RIGHT, 3.0);
	pros::delay(500);
	driveForwardCubic(-70, 1.1);
	pros::delay(500);
	lift.set_value(true);
	pros::delay(500);
	auto start_time = pros::millis();
	while(!climb_switch.get_value() && (pros::millis() - start_time) < 3000){
		leftDrive.move_voltage(10000);
		rightDrive.move_voltage(10000);
	}
	lift.set_value(false);
	leftDrive.move_voltage(0);
	rightDrive.move_voltage(0);
	pros::delay(500);
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
void opcontrol(){
	bool catapultSeated = false;
	double last_climb_switch_time = pros::millis();
	// autonomous();
	while(true){
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		// 				 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		// 				 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int deadzone = 20;
		int joystickLeftY = master.get_analog(ANALOG_LEFT_Y);
		if(abs(joystickLeftY) < deadzone){
			joystickLeftY = 0;
		}
		int joystickLeftX = master.get_analog(ANALOG_LEFT_X);
		if(abs(joystickLeftX) < deadzone){
			joystickLeftX = 0;
		}
		int joystickRightY = master.get_analog(ANALOG_RIGHT_Y);
		if(abs(joystickRightY) < deadzone){
			joystickRightY = 0;
		}
		int joystickRightX = master.get_analog(ANALOG_RIGHT_X);
		if(abs(joystickRightX) < deadzone){
			joystickRightX = 0;
		}
		bool buttonA = master.get_digital(DIGITAL_A);
		bool buttonB = master.get_digital(DIGITAL_B);

		// drive
		const int MAXVOLTAGE = 12000;
		double leftSpeed = joystickLeftY / 127.0;  // [0,1]
		double rightSpeed = joystickRightY / 127.0; // [0,1]
		double leftVoltage = leftSpeed * MAXVOLTAGE;
		double rightVoltage = rightSpeed * MAXVOLTAGE;

		if(master.get_digital(DIGITAL_LEFT)){
			leftVoltage *= 0.5;
			rightVoltage *= 0.5;
			leftVoltage = rightVoltage;
		}

		wingR.set_value(master.get_digital(DIGITAL_R1));

		leftDrive.move_voltage(leftVoltage);
		rightDrive.move_voltage(rightVoltage);

		bool buttonL2 = master.get_digital(DIGITAL_L2);
		bool buttonR2 = master.get_digital(DIGITAL_R2);
		if(buttonL2){
			intakeMotorA.move_voltage(12000);
		}
		else if(buttonR2){
			intakeMotorA.move_voltage(-12000);
		}
		else{
			intake.brake();
		}

		static bool climbed = false;

		if(master.get_digital(DIGITAL_L1)){
			if(climb_switch.get_value() && !climbed){
				lift.set_value(false);
				climbed = true;
				last_climb_switch_time = pros::millis();
			}
			else if(pros::millis() - last_climb_switch_time > 500){
				lift.set_value(true);
				climbed = false;
			}
		}
		else if(master.get_digital(DIGITAL_DOWN)){
			lift.set_value(true);
		}
		else{
			lift.set_value(false);
		}

		if(master.get_digital(DIGITAL_UP) && master.get_digital(DIGITAL_LEFT) && master.get_digital(DIGITAL_RIGHT)){
			autonomous();
		}


		pros::delay(20);
	}
}