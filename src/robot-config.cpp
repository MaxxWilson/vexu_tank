#include "robot-config.hpp"

extern const double RPM = 600. * 36. / 48.;
// VEXcode generated functions
// define variable for remote controller enable/disable
// bool RemoteControlCodeEnabled = true;
extern const pros::motor_encoder_units_e_t encoder_units = pros::E_MOTOR_ENCODER_ROTATIONS;
extern const pros::motor_gearset_e_t driveGearset = pros::E_MOTOR_GEAR_600;
// extern const pros::motor_gearset_e_t rollerGearset = pros::E_MOTOR_GEAR_100;
extern const pros::motor_gearset_e_t intakeGearset = pros::E_MOTOR_GEAR_200;

// device constructors
extern pros::Controller master(pros::E_CONTROLLER_MASTER);
extern pros::Motor leftDriveMotorA = pros::Motor(12, driveGearset, true, encoder_units);
extern pros::Motor leftDriveMotorB = pros::Motor(13, driveGearset, true, encoder_units); // out
extern pros::Motor leftDriveMotorC = pros::Motor(14, driveGearset, true, encoder_units);
extern pros::Motor leftDriveMotorD = pros::Motor(15, driveGearset, false, encoder_units); //?
extern pros::Motor_Group leftDrive = pros::Motor_Group({leftDriveMotorA, leftDriveMotorB, leftDriveMotorC, leftDriveMotorD});
extern pros::Motor rightDriveMotorA = pros::Motor(16, driveGearset, false, encoder_units); // out
extern pros::Motor rightDriveMotorB = pros::Motor(17, driveGearset, false, encoder_units); // out
extern pros::Motor rightDriveMotorC = pros::Motor(18, driveGearset, false, encoder_units);
extern pros::Motor rightDriveMotorD = pros::Motor(19, driveGearset, true, encoder_units);
extern pros::Motor_Group rightDrive = pros::Motor_Group({rightDriveMotorA, rightDriveMotorB, rightDriveMotorC, rightDriveMotorD});
// extern pros::Motor rollerMotor = pros::Motor(2, rollerGearset, false, encoder_units);
extern pros::Motor intakeMotorA = pros::Motor(11, intakeGearset, false, encoder_units);
// extern pros::Motor intakeMotorB = pros::Motor(4, intakeGearset, true, encoder_units);
extern pros::Motor_Group intake = pros::Motor_Group({intakeMotorA});
// extern pros::Motor catapultMotor = pros::Motor(1, rollerGearset, true, encoder_units);

// sensors
extern pros::ADIDigitalOut wingL = pros::ADIDigitalOut('C');
extern pros::ADIDigitalOut wingR = pros::ADIDigitalOut('B');
extern pros::ADIDigitalOut release = pros::ADIDigitalOut('D');
extern pros::ADIDigitalOut climbup = pros::ADIDigitalOut('A');

extern std::vector<pros::Motor> motorList = {
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
void motorInit(void)
{
    pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_BRAKE;
    for (pros::Motor motor : motorList)
    {
        motor.set_brake_mode(brakeMode);
    }
}