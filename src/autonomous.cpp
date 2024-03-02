#include "autonomous.hpp"

namespace Auton {

using Eigen::MatrixXf;

MatrixXf computeCubicCoeff(double t0, double tf, std::vector<double> vec_q0, std::vector<double> vec_qf){
	// vec_q0 = {position, velocity}
	// Ax = B
	// A = cubic function matrix
	// x = coefficients
	// B = initial and final values

	MatrixXf A(4, 4);
	MatrixXf B(4, 1);

	A(0, 0) = 1;
	A(0, 1) = t0;
	A(0, 2) = std::pow(t0, 2);
	A(0, 3) = std::pow(t0, 3);

	A(1, 0) = 0;
	A(1, 1) = 1;
	A(1, 2) = 2 * t0;
	A(1, 3) = 3 * std::pow(t0, 2);

	A(2, 0) = 1;
	A(2, 1) = tf;
	A(2, 2) = std::pow(tf, 2);
	A(2, 3) = std::pow(tf, 3);

	A(3, 0) = 0;
	A(3, 1) = 1;
	A(3, 2) = 2 * tf;
	A(3, 3) = 3 * std::pow(tf, 2);

	B(0, 0) = vec_q0[0];
	B(1, 0) = vec_q0[1];
	B(2, 0) = vec_qf[0];
	B(3, 0) = vec_qf[1];

	return A.inverse() * B;
};

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double> > computeCubicTraj(std::vector<double> vec_q0,
                                                                                                                 std::vector<double> vec_qf, double t0, double tf, int n){
	// A = coefficients
	// n = number of timesteps
	auto A = computeCubicCoeff(t0, tf, vec_q0, vec_qf);

	std::vector<double> a = {A(0, 0), A(1, 0), A(2, 0), A(3, 0)};

	std::vector<double> qd;
	std::vector<double> d_qd;
	std::vector<double> dd_qd;
	std::vector<double> time;
	double step = (tf - t0) / n;
	for(double t = t0; t < tf; t += step){
		double qdi = a[0] + a[1] * t + a[2] * std::pow(t, 2) + a[3] * std::pow(t, 3);
		double d_qdi = a[1] + 2 * a[2] * t + 3 * a[3] * std::pow(t, 2);
		double dd_qdi = 6 * a[3] * t + 2 * a[2];

		qd.push_back(qdi);
		d_qd.push_back(d_qdi);
		dd_qd.push_back(dd_qdi);
		time.push_back(t);
	}

	return std::make_tuple(time, qd, d_qd, dd_qd);
};

double linearInterpolate(const std::vector<double> &x_data,
                         const std::vector<double> &y_data,
                         const double desired_x) {
	// check if empty
	if((x_data.size() == 0) || (y_data.size() == 0)){
		throw std::runtime_error("[linearInterpolate] Error: cannot interpolate empty vector");
	}

	// check vector lengths
	if(x_data.size() != y_data.size()){
		throw std::runtime_error("[linearInterpolate] Error: size of input vectors are unequal");
	}

	// get upper and lower bounds as an index
	auto upper_index = std::upper_bound(x_data.begin(), x_data.end(), desired_x) - x_data.begin();
	auto lower_index = std::lower_bound(x_data.begin(), x_data.end(), desired_x) - x_data.begin();

	// return exact match
	if(upper_index != lower_index){
		return y_data[lower_index];
	}

	// extrapolate from first two points if desired x before range
	if(upper_index == 0){
		return ((y_data[1] - y_data[0]) / (x_data[1] - x_data[0]))
		       * (desired_x - x_data[0]) + y_data[0];
	}

	// extrapolate from last two points if desired x after range
	if(lower_index == x_data.size()){
		auto y_last = y_data.end() - 1;
		auto x_last = x_data.end() - 1;
		return ((*y_last - *(y_last - 1)) / (*x_last - *(x_last - 1)))
		       * (desired_x - *x_last) + *y_last;
	}

	// interpolate
	return ((y_data[upper_index] - y_data[lower_index - 1]) / (x_data[upper_index] - x_data[lower_index - 1]))
	       * (desired_x - x_data[lower_index - 1]) + y_data[lower_index - 1];
}

double clampedLinearInterpolate(const std::vector<double> &x_data,
                                const std::vector<double> &y_data,
                                const double desired_x) {
	if(desired_x <= x_data.front()){
		return y_data.front();
	}

	if(desired_x >= x_data.back()){
		return y_data.back();
	}

	return linearInterpolate(x_data, y_data, desired_x);
}

void setup(){
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
}
double velocity(double time, double dist, double max_speed, double acceleration){
	dist *= GEAR_RATIO;
	max_speed *= MAX_SPEED;
	acceleration *= MAX_SPEED;
	double velocity = 0;
	double acceleration_dist = max_speed * max_speed / (acceleration * 2);
	if(dist < acceleration_dist * 2){
		double total_time = 2 * sqrt(dist / acceleration);
		if(time > total_time){
			velocity = -0.01;
		}
		else{
			max_speed = sqrt(dist * acceleration);
			if(time < total_time / 2){
				velocity = (acceleration * time);
			}
			else{
				velocity = 2 * max_speed - acceleration * time;
			}
		}
	}
	else{
		double acceleration_time = max_speed / acceleration;
		double time_until_deceleration = dist / max_speed;
		double total_time = time_until_deceleration + acceleration_time;

		if(time > total_time){
			velocity = -0.01;
		}
		else{
			if(time > time_until_deceleration){
				velocity = max_speed - (acceleration * (time - (time_until_deceleration)));
			}
			else if(time > acceleration_time){
				velocity = max_speed;
			}
			else{
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

double getPositionAverage(pros::Motor_Group& motor_group){
	auto positions = motor_group.get_positions();
	double position = 0;
	for(const auto pos : positions){
		position += pos;
	}

	return position / positions.size();
}

double getVelocityAverage(pros::Motor_Group& motor_group){
	auto velocities = motor_group.get_actual_velocities();
	double velocity = 0;
	for(const auto vel : velocities){
		velocity += vel;
	}

	return velocity / velocities.size();
}

void driveForwardCubic(double dist,double duration){
	double time = 0;
	double speed = 0;

	// Generate Cubic Trajectory
	auto traj = computeCubicTraj(std::vector<double>{0.0, 0.0}, std::vector<double>{dist, 0.0}, 0, duration, 100);
	auto time_trajectory = std::get<0>(traj);
	auto position_trajectory = std::get<1>(traj);
	auto velocity_trajectory = std::get<2>(traj);
	auto left_start = getPositionAverage(leftDrive) * WHEEL_ROTATION_TO_INCHES;
	auto right_start = getPositionAverage(rightDrive) * WHEEL_ROTATION_TO_INCHES;

	while(time <= duration){
		auto des_position = clampedLinearInterpolate(time_trajectory, position_trajectory, time);
		auto des_velocity = clampedLinearInterpolate(time_trajectory, velocity_trajectory, time);
		auto curr_position_left = getPositionAverage(leftDrive) * WHEEL_ROTATION_TO_INCHES - left_start;
		auto curr_position_right = getPositionAverage(rightDrive) * WHEEL_ROTATION_TO_INCHES - right_start;
		auto curr_velocity_left = getVelocityAverage(leftDrive) * WHEEL_RPM_TO_INCHES_PER_SEC;
		auto curr_velocity_right = getVelocityAverage(rightDrive) * WHEEL_RPM_TO_INCHES_PER_SEC;

		//
		double right_voltage =
			(des_position - curr_position_right) * DRIVE_FORWARD_POSITION_GAIN +
			(des_velocity - curr_velocity_right) * DRIVE_FORWARD_VELOCITY_GAIN +
			des_velocity * DRIVE_FORWARD_FF_VELOCITY_GAIN;
		double left_voltage =
			(des_position - curr_position_left) * DRIVE_FORWARD_POSITION_GAIN +
			(des_velocity - curr_velocity_left) * DRIVE_FORWARD_VELOCITY_GAIN +
			des_velocity * DRIVE_FORWARD_FF_VELOCITY_GAIN;
		// Add PD Control Law for position and velocity
		leftDrive.move_voltage(left_voltage);
		rightDrive.move_voltage(right_voltage);

		time += 0.010;
		pros::delay(10);
	}
	leftDrive.brake();
	rightDrive.brake();
}

void arcTurnCubic(double angle_deg, double duration, turn_direction_e_t direction){
	double time = 0;
	double speed = 0;
	double angle_rad = angle_deg * 3.14159 / 180.0;

	pros::Motor_Group *drive;
	pros::Motor_Group *braked;

	if(direction == turn_direction_e_t::LEFT){
		drive = &leftDrive;
		braked = &rightDrive;
	}
	else{
		drive = &rightDrive;
		braked = &leftDrive;
	}

	// Generate Cubic Trajectory
	auto traj = computeCubicTraj(std::vector<double>{0.0, 0.0}, std::vector<double>{angle_rad, 0.0}, 0, duration, 100);
	auto time_trajectory = std::get<0>(traj);
	auto position_trajectory = std::get<1>(traj);
	auto velocity_trajectory = std::get<2>(traj);
	auto start = getPositionAverage(*drive) * WHEEL_ROTATION_TO_INCHES;

	while(time <= duration + 0.5){
		auto des_angle = clampedLinearInterpolate(time_trajectory, position_trajectory, time);
		auto des_velocity = clampedLinearInterpolate(time_trajectory, velocity_trajectory, time);

		// Estimate Angle
		auto curr_angle = (getPositionAverage(*drive) * WHEEL_ROTATION_TO_INCHES - start) / (WHEEL_DIST_S);
		auto curr_angular_velocity = getVelocityAverage(*drive) * WHEEL_RPM_TO_INCHES_PER_SEC / (WHEEL_DIST_S);

		//
		double voltage =
			(des_angle - curr_angle) * ARC_TURN_POSITION_GAIN +
			(des_velocity - curr_angular_velocity) * ARC_TURN_VELOCITY_GAIN +
			des_velocity * ARC_TURN_FF_VELOCITY_GAIN;

		if(time > duration){
			voltage = (des_angle - curr_angle) * ARC_TURN_POSITION_GAIN * 2.0;
		}

		std::cout << (des_angle - curr_angle) << std::endl;

		// Add PD Control Law for position and velocity
		drive->move_voltage(voltage);
		braked->brake();

		time += 0.010;
		pros::delay(10);
	}
	leftDrive.brake();
	rightDrive.brake();
}

void arcTurnPD(double angle_deg, double duration, turn_direction_e_t direction, double aggression){
	double time = 0;
	double speed = 0;
	double angle_rad = angle_deg * 3.14159 / 180.0;

	pros::Motor_Group *drive;
	pros::Motor_Group *braked;

	if(direction == turn_direction_e_t::LEFT){
		drive = &leftDrive;
		braked = &rightDrive;
	}
	else{
		drive = &rightDrive;
		braked = &leftDrive;
	}

	// Generate Cubic Trajectory
	auto start = getPositionAverage(*drive) * WHEEL_ROTATION_TO_INCHES;

	while(time <= duration){
		// Estimate Angle
		auto curr_angle = (getPositionAverage(*drive) * WHEEL_ROTATION_TO_INCHES - start) / (WHEEL_DIST_S);
		auto curr_angular_velocity = getVelocityAverage(*drive) * WHEEL_RPM_TO_INCHES_PER_SEC / (WHEEL_DIST_S);

		//
		double voltage =
			(angle_rad - curr_angle) * ARC_TURN_POSITION_GAIN_PD * aggression +
			(-curr_angular_velocity) * ARC_TURN_VELOCITY_GAIN_PD;

		// Add PD Control Law for position and velocity
		drive->move_voltage(voltage);
		braked->brake();

		time += 0.010;
		pros::delay(10);
	}
	leftDrive.brake();
	rightDrive.brake();
}


void pointTurnCubic(double angle_deg, double duration){
	double time = 0;
	double speed = 0;
	double angle_rad = angle_deg * 3.14159 / 180.0;

	// Generate Cubic Trajectory
	auto traj = computeCubicTraj(std::vector<double>{0.0, 0.0}, std::vector<double>{angle_rad, 0.0}, 0, duration, 100);
	auto time_trajectory = std::get<0>(traj);
	auto position_trajectory = std::get<1>(traj);
	auto velocity_trajectory = std::get<2>(traj);
	auto acceleration_trajectory = std::get<3>(traj);
	auto left_start = getPositionAverage(leftDrive) * WHEEL_ROTATION_TO_INCHES;
	auto right_start = getPositionAverage(rightDrive) * WHEEL_ROTATION_TO_INCHES;

	while(time <= duration){
		auto des_angle = clampedLinearInterpolate(time_trajectory, position_trajectory, time);
		auto des_velocity = clampedLinearInterpolate(time_trajectory, velocity_trajectory, time);

		// Estimate Angle
		auto curr_angle_left = -(getPositionAverage(leftDrive) * WHEEL_ROTATION_TO_INCHES - left_start) / (WHEEL_DIST_S / 2);
		auto curr_angle_right = (getPositionAverage(rightDrive) * WHEEL_ROTATION_TO_INCHES - right_start) / (WHEEL_DIST_S / 2);
		auto curr_angular_velocity_left = getVelocityAverage(leftDrive) * WHEEL_RPM_TO_INCHES_PER_SEC / (WHEEL_DIST_S / 2);
		auto curr_angular_velocity_right = getVelocityAverage(rightDrive) * WHEEL_RPM_TO_INCHES_PER_SEC / (WHEEL_DIST_S / 2);

		//
		double right_voltage =
			(des_angle - curr_angle_right) * POINT_TURN_POSITION_GAIN +
			(des_velocity - curr_angular_velocity_right) * POINT_TURN_VELOCITY_GAIN +
			des_velocity * POINT_TURN_FF_VELOCITY_GAIN;
		double left_voltage =
			(des_angle - curr_angle_left) * POINT_TURN_POSITION_GAIN +
			(des_velocity - curr_angular_velocity_left) * POINT_TURN_VELOCITY_GAIN +
			des_velocity * POINT_TURN_FF_VELOCITY_GAIN;

		// Add PD Control Law for position and velocity
		leftDrive.move_voltage(-left_voltage);
		rightDrive.move_voltage(right_voltage);

		time += 0.010;
		pros::delay(10);
	}

	leftDrive.brake();
	rightDrive.brake();
}

void driveForward(double dist, double max_speed, double acceleration){
	double time = 0;
	double speed = 0;
	double direction = 1;
	if(dist < 0){
		direction = -1;
		dist *= -1;
	}
	while(speed >= 0){
		speed = velocity(time, dist, max_speed, acceleration);
		leftDrive.move_velocity(direction * speed);
		rightDrive.move_velocity(direction * speed);
		time += 0.010;
		pros::delay(10);
	}
	// double revs_required = dist / CIRC;
	// if (leftDrive.rotation(rev) > revs_required){
	//   driveForward(CIRC * -(leftDrive.rotation(rev) - revs_required), max_speed, acceleration);
	// }
	leftDrive.brake();
	rightDrive.brake();
}

void driveBackward(double dist, double max_speed, double acceleration){
	driveForward(-dist, max_speed, acceleration);
}
void turnLeft(double angle, double max_speed, double acceleration){
	double time = 0;
	double speed = 0;
	double direction = 1;
	if(angle < 0){
		direction = -1;
		angle *= -1;
	}
	double dist = WHEEL_R * M_PI * angle / 180 / cos(atan(WHEEL_DIST_F / WHEEL_DIST_S));
	while(speed >= 0){
		speed = velocity(time, dist, max_speed, acceleration);
		leftDrive.move_velocity(direction * -speed);
		rightDrive.move_velocity(direction * speed);
		time += 0.020;
		pros::delay(20);
	}
	leftDrive.brake();
	rightDrive.brake();
}
void turnRight(double angle, double max_speed, double acceleration){
	turnLeft(-angle, max_speed, acceleration);
}
void turnLeftArc(double angle, double max_speed, double acceleration){
	double time = 0;
	double speed = 0;
	double direction = 1;
	if(angle < 0){
		direction = -1;
		angle *= -1;
	}
	double dist = WHEEL_R * M_PI * angle / 180 / cos(atan(WHEEL_DIST_F / WHEEL_DIST_S / 2));
	while(speed >= 0){
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

// Specific methods for wisco-autons
void process_match_loads(int num_match_loads){
	for(int match_load = 0; match_load < num_match_loads; match_load++){
		driveForwardCubic(-9.0, 0.27);
		pros::delay(300);
		driveForwardCubic(8.6, 0.27);
		pros::delay(750);
	}
}

/**
 * Assuming starting from blue diagonal bar
 */
void push_across_field(double turning_radius, double line_path, double max_speed){
	double time = 0;
	double r_speed = 0;
	double l_speed = 0;
	double dist_across_field = 90;
	double direction = -1.0;

	// Caluclate arc length of left and right side
	double l_arc = (turning_radius + WHEEL_DIST_S * 0.5) * atan(line_path / (turning_radius + WHEEL_DIST_S * 0.5));
	double r_arc = (turning_radius - WHEEL_DIST_S * 0.5) * atan(line_path / (turning_radius - WHEEL_DIST_S * 0.5));

	// driveBackward(14.0, max_speed, 0.5);

	// Turn while pushing match loads
	while(l_speed >= 0){
		r_speed = velocity(time, l_arc, max_speed, 0.5);
		l_speed = velocity(time, r_arc, max_speed, 0.5);
		leftDrive.move_velocity(direction * l_speed);
		rightDrive.move_velocity(direction * r_speed);
		time += 0.020;
		pros::delay(10);
	}

	// Push till reach opposing diagonal bar
	// driveForward(TILE_LENGTH * 4, max_speed, 0.5);

	leftDrive.brake();
	rightDrive.brake();
}

void push_into_goal(double turning_radius, double line_path, double max_speed){
	driveBackward(4.0, max_speed, 0.5);
	pros::delay(100);
	driveForward(4.0, max_speed, 0.5);
	pros::delay(100);

	double time = 0;
	double r_speed = 0;
	double l_speed = 0;
	double direction = 1;

	// Caluclate arc length of left and right side
	double l_arc = (turning_radius + WHEEL_DIST_S * 0.5) * atan(line_path / (turning_radius + WHEEL_DIST_S * 0.5));
	double r_arc = (turning_radius - WHEEL_DIST_S * 0.5) * atan(line_path / (turning_radius - WHEEL_DIST_S * 0.5));

	// Turn while pushing match loads
	while(l_speed >= 0){
		r_speed = velocity(time, l_arc, max_speed, 0.5);
		l_speed = velocity(time, r_arc, max_speed, 0.5);
		leftDrive.move_velocity(direction * l_speed);
		rightDrive.move_velocity(direction * r_speed);
		time += 0.020;
		pros::delay(20);
	}

	int num_pushes = 5;
	for(int push = 0; push < num_pushes; push++){
		driveBackward(14.0, max_speed, 0.5);
		pros::delay(100);
		driveForward(14.0, max_speed, 0.5);
		pros::delay(100);
	}
}

}