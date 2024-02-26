#pragma once
#include "api.hpp"
#include "robot-config.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>

const double RPS = RPM / 60;                 // rotations / sec
const double CIRC = M_PI * 3;                // in
const double MAX_SPEED = RPS * CIRC;         // in / sec
const double WHEEL_DIST_S = 9.5;             // (in) sideways
const double WHEEL_DIST_F = 10;              // (in) forward
const double WHEEL_R = WHEEL_DIST_S / 2;     // sqrt(WHEEL_DIST_S * WHEEL_DIST_S + WHEEL_DIST_F * WHEEL_DIST_F);
const double WHEEL_R_ARC = WHEEL_DIST_S / 2; // sqrt(4 * WHEEL_DIST_S * WHEEL_DIST_S + WHEEL_DIST_F * WHEEL_DIST_F);

namespace Auton
{
    void setup();
    double velocity(double time, double dist, double max_speed, double acceleration);
    void driveForward(double dist, double max_speed, double acceleration);
    void driveBackward(double dist, double max_speed, double acceleration);
    void turnLeft(double angle, double max_speed, double acceleration);
    void turnLeftArc(double angle, double max_speed, double acceleration);
    void turnRight(double angle, double max_speed, double acceleration);
    //void roller();
}