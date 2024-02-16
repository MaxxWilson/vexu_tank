#include "autonomous.hpp"

namespace Auton
{
    void setup()
    {
        leftDrive.move_velocity(0);
        rightDrive.move_velocity(0);
    }
    double velocity(double time, double dist, double max_speed, double acceleration)
    {
        max_speed *= MAX_SPEED;
        acceleration *= MAX_SPEED;
        double velocity = 0;
        double acceleration_dist = max_speed * max_speed / (acceleration * 2);
        if (dist < acceleration_dist * 2)
        {
            double total_time = 2 * sqrt(dist / acceleration);
            if (time > total_time)
            {
                velocity = -0.01;
            }
            else
            {
                max_speed = sqrt(dist * acceleration);
                if (time < total_time / 2)
                {
                    velocity = (acceleration * time);
                }
                else
                {
                    velocity = 2 * max_speed - acceleration * time;
                }
            }
        }
        else
        {
            double acceleration_time = max_speed / acceleration;
            double time_until_deceleration = dist / max_speed;
            double total_time = time_until_deceleration + acceleration_time;

            if (time > total_time)
            {
                velocity = -0.01;
            }
            else
            {
                if (time > time_until_deceleration)
                {
                    velocity = max_speed - (acceleration * (time - (time_until_deceleration)));
                }
                else if (time > acceleration_time)
                {
                    velocity = max_speed;
                }
                else
                {
                    velocity = acceleration * time;
                }
            }
        }
        return velocity / MAX_SPEED * RPM;
    }
    // double time_from_pos(double position, double target_dist, double max_speed, double acceleration) // not finished
    // {
    //     max_speed *= MAX_SPEED;
    //     acceleration *= MAX_SPEED;
    //     double time = 0;
    //     double acceleration_dist = max_speed * max_speed / (acceleration * 2);
    //     if (target_dist < acceleration_dist * 2)
    //     {
    //         double total_time = 2 * sqrt(target_dist / acceleration);
    //         if (time > total_time)
    //         {
    //             time = -0.01;
    //         }
    //         else
    //         {
    //             max_speed = sqrt(target_dist * acceleration);
    //             if (time < total_time / 2)
    //             {
    //                 time = (acceleration * time);
    //             }
    //             else
    //             {
    //                 time = 2 * max_speed - acceleration * time;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         double time_1 = max_speed / acceleration;
    //         double time_2 = target_dist / max_speed;
    //         double time_3 = time_1 + time_2;
    //         double dist_1 = max_speed;
    //         double dist_2 = target_dist - max_speed * max_speed / acceleration / 2;
    //         // double dist_3 = target_dist;
    //         if (position > target_dist)
    //         {
    //             time = -0.01;
    //         }
    //         else if (position > dist_2)
    //         {
    //             time = time_3 - sqrt(2 * (target_dist - position) / acceleration);
    //         }
    //         else if (position > dist_1)
    //         {
    //             time = position / max_speed + time_1 / 2;
    //         }
    //         else
    //         {
    //             time = sqrt(2 * position / acceleration);
    //         }
    //     }
    //     return time;
    // }
    void driveForward(double dist, double max_speed, double acceleration)
    {
        double time = 0;
        double speed = 0;
        double direction = 1;
        if (dist < 0)
        {
            direction = -1;
            dist *= -1;
        }
        while (speed >= 0)
        {
            speed = velocity(time, dist, max_speed, acceleration);
            leftDrive.move_velocity(direction * speed);
            rightDrive.move_velocity(direction * speed);
            time += 0.020;
            pros::delay(20);
        }
        // double revs_required = dist / CIRC;
        // if (leftDrive.rotation(rev) > revs_required){
        //   driveForward(CIRC * -(leftDrive.rotation(rev) - revs_required), max_speed, acceleration);
        // }
        leftDrive.brake();
        rightDrive.brake();
    }
    void driveBackward(double dist, double max_speed, double acceleration)
    {
        driveForward(-dist, max_speed, acceleration);
    }
    void turnLeft(double angle, double max_speed, double acceleration)
    {
        double time = 0;
        double speed = 0;
        double direction = 1;
        if (angle < 0)
        {
            direction = -1;
            angle *= -1;
        }
        double dist = WHEEL_R * M_PI * angle / 180 / cos(atan(WHEEL_DIST_F / WHEEL_DIST_S));
        while (speed >= 0)
        {
            speed = velocity(time, dist, max_speed, acceleration);
            leftDrive.move_velocity(direction * -speed);
            rightDrive.move_velocity(direction * speed);
            time += 0.020;
            pros::delay(20);
        }
        leftDrive.brake();
        rightDrive.brake();
    }
    void turnRight(double angle, double max_speed, double acceleration)
    {
        turnLeft(-angle, max_speed, acceleration);
    }
    void turnLeftArc(double angle, double max_speed, double acceleration)
    {
        double time = 0;
        double speed = 0;
        double direction = 1;
        if (angle < 0)
        {
            direction = -1;
            angle *= -1;
        }
        double dist = WHEEL_R_ARC * M_PI * angle / 180 / cos(atan(WHEEL_DIST_F / WHEEL_DIST_S / 2));
        while (speed >= 0)
        {
            speed = velocity(time, dist, max_speed, acceleration);
            rightDrive.move_velocity(direction * speed);
            leftDrive.brake();
            time += 0.020;
            pros::delay(20);
        }
        leftDrive.brake();
        rightDrive.brake();
    }
    void roller(){
        rollerMotor.move_velocity(200);
        pros::delay(400);
        rollerMotor.brake();
    }
}