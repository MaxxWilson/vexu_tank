#pragma once
#include "api.hpp"
#include "robot-config.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>

#include "Eigen/Dense"

const double GEAR_RATIO = 48.0 / 36.0;
const double RPS = RPM / 60;                 // rotations / sec
const double CIRC = M_PI * 3.25;                // in
const double MAX_SPEED = RPS * CIRC;         // in / sec
const double MAX_ACCEL = 368.11;              // in / s^2
const double WHEEL_DIST_S = 9.5;             // (in) sideways
const double WHEEL_DIST_F = 9.5;             // (in) sideways
const double WHEEL_R = WHEEL_DIST_S / 2;
const double MAX_ANG_VEL = 10.77;
const double TILE_LENGTH = 23.5;
const double WHEEL_ROTATION_TO_INCHES = 1 / GEAR_RATIO * CIRC;
const double WHEEL_RPM_TO_INCHES_PER_SEC = WHEEL_ROTATION_TO_INCHES / 60.0;

const double DRIVE_FORWARD_POSITION_GAIN = 500.0;
const double DRIVE_FORWARD_VELOCITY_GAIN = 300.0;
const double DRIVE_FORWARD_FF_VELOCITY_GAIN = 12000.0 / MAX_SPEED;

const double POINT_TURN_POSITION_GAIN = 2500.0;
const double POINT_TURN_VELOCITY_GAIN = 1000.0;
const double POINT_TURN_FF_VELOCITY_GAIN = 12000.0 / MAX_ANG_VEL;

const double ARC_TURN_POSITION_GAIN = 4000.0;
const double ARC_TURN_VELOCITY_GAIN = 2000.0;
const double ARC_TURN_FF_VELOCITY_GAIN = 2.0 * 12000.0 / (MAX_ANG_VEL / 2);

const double ARC_TURN_POSITION_GAIN_PD = 40000.0;
const double ARC_TURN_VELOCITY_GAIN_PD = 0.0;

namespace Auton {

typedef enum turn_direction_e_t {
	RIGHT,
	LEFT
}turn_direction_e_t;

void setup();
double velocity(double time, double dist, double max_speed, double acceleration);
void driveForward(double dist, double max_speed, double acceleration);
void driveBackward(double dist, double max_speed, double acceleration);
void turnLeft(double angle, double max_speed, double acceleration);
void turnLeftArc(double angle, double max_speed, double acceleration);
void turnRight(double angle, double max_speed, double acceleration);
void roller();

// Maxx Motions
void driveForwardCubic(double dist, double duration);
void arcTurnCubic(double angle_deg, double duration, turn_direction_e_t direction);
void pointTurnCubic(double angle_deg, double duration);
void arcTurnPD(double angle_deg, double duration, turn_direction_e_t direction, double aggression = 1.0);
void pointTurnPD(double angle_deg, double duration);
double getPositionAverage(pros::Motor_Group& motor_group);
double getVelocityAverage(pros::Motor_Group& motor_group);

// wisco-autons
void process_match_loads(int num_match_loads);
void push_into_goal(double turning_radius, double line_path, double max_speed);
void push_across_field(double turning_radius, double line_path, double max_speed);

Eigen::MatrixXf computeCubicCoeff(double t0, double tf, std::vector<double> vec_q0, std::vector<double> vec_qf);

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>  > computeCubicTraj(std::vector<double> vec_q0,
                                                                                                                  std::vector<double> vec_qf,
                                                                                                                  double t0, double tf, int n);

double linearInterpolate(const std::vector<double> &x_data,
                         const std::vector<double> &y_data,
                         const double desired_x);

double clampedLinearInterpolate(const std::vector<double> &x_data,
                                const std::vector<double> &y_data,
                                const double desired_x);

}