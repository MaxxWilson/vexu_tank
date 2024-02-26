#pragma once
#include "api.hpp"

// extern brain Brain;
extern const double RPM;                      // max rotational velocity

// constants
extern const pros::motor_encoder_units_e_t encoder_units;
extern const pros::motor_gearset_e_t driveGearset;
extern const pros::motor_gearset_e_t rollerGearset;
extern const pros::motor_gearset_e_t intakeGearset;

// motors
extern pros::Controller master;
extern pros::Motor leftDriveMotorA;
extern pros::Motor leftDriveMotorB;
extern pros::Motor leftDriveMotorC;
extern pros::Motor leftDriveMotorD;
extern pros::Motor rightDriveMotorA;
extern pros::Motor rightDriveMotorB;
extern pros::Motor rightDriveMotorC;
extern pros::Motor rightDriveMotorD;
extern pros::Motor_Group leftDrive;
extern pros::Motor_Group rightDrive;
extern pros::Motor rollerMotor;
extern pros::Motor intakeMotorA;
extern pros::Motor intakeMotorB;
extern pros::Motor_Group intake;
extern pros::Motor catapultMotor;
extern std::vector<pros::Motor> motorList;

//sensors
extern pros::ADIDigitalOut wingL;
extern pros::ADIDigitalOut wingR;
extern pros::ADIDigitalOut release;
extern pros::ADIDigitalOut climbup;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void motorInit(void);