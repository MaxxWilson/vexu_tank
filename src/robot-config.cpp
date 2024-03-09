#include "robot-config.hpp"

 const double RPM = 600. * 36. / 48.;
// VEXcode generated functions
// define variable for remote controller enable/disable
// bool RemoteControlCodeEnabled = true;
 const pros::motor_encoder_units_e_t encoder_units = pros::E_MOTOR_ENCODER_ROTATIONS;
 const pros::motor_gearset_e_t driveGearset = pros::E_MOTOR_GEAR_600;
//  const pros::motor_gearset_e_t rollerGearset = pros::E_MOTOR_GEAR_100;
 const pros::motor_gearset_e_t intakeGearset = pros::E_MOTOR_GEAR_600;

// device constructors
 pros::Controller master(pros::E_CONTROLLER_MASTER);
 pros::Motor leftDriveMotorA = pros::Motor(17, driveGearset, true, encoder_units);
 pros::Motor leftDriveMotorB = pros::Motor(18, driveGearset, false, encoder_units); // out
 pros::Motor leftDriveMotorC = pros::Motor(19, driveGearset, true, encoder_units);
 pros::Motor leftDriveMotorD = pros::Motor(20, driveGearset, true, encoder_units); // ?
 pros::Motor_Group leftDrive = pros::Motor_Group({leftDriveMotorA});
 pros::Motor rightDriveMotorA = pros::Motor(12, driveGearset, false, encoder_units); // out
 pros::Motor rightDriveMotorB = pros::Motor(13, driveGearset, false, encoder_units); // out
 pros::Motor rightDriveMotorC = pros::Motor(14, driveGearset, true, encoder_units);
 pros::Motor rightDriveMotorD = pros::Motor(15, driveGearset, false, encoder_units);
 pros::Motor_Group rightDrive = pros::Motor_Group({rightDriveMotorA});
//  pros::Motor rollerMotor = pros::Motor(2, rollerGearset, false, encoder_units);
 pros::Motor intakeMotorA = pros::Motor(16, intakeGearset, true, encoder_units);
//  pros::Motor intakeMotorB = pros::Motor(4, intakeGearset, true, encoder_units);
 pros::Motor_Group intake = pros::Motor_Group({intakeMotorA});
//  pros::Motor catapultMotor = pros::Motor(1, rollerGearset, true, encoder_units);

// sensors
 pros::ADIDigitalOut wingL = pros::ADIDigitalOut('B');
 pros::ADIDigitalOut wingR = pros::ADIDigitalOut('C');
 pros::ADIDigitalOut lift = pros::ADIDigitalOut('D');
 pros::ADIDigitalIn climb_switch = pros::ADIDigitalIn('A');

 std::vector<pros::Motor> motorList = {
	leftDriveMotorA,
	leftDriveMotorB,
	leftDriveMotorC,
	leftDriveMotorD,
	rightDriveMotorA,
	rightDriveMotorB,
	rightDriveMotorC,
	rightDriveMotorD,
	intakeMotorA,
};
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void motorInit(void){
	pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_BRAKE;
	for(pros::Motor motor : motorList){
		motor.set_brake_mode(brakeMode);
	}
}