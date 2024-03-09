// #pragma once
// // #include "rclcpp/rclcpp.hpp"
// #include <memory>
// #include <vector>

// namespace ghost_tank {

// class TankOdometry {
// public:
// 	TankOdometry();
// 	// void setAngle(double angle) {
// 	// 	m_Angle = angle;
// 	// }

// 	// void setXPosition(double x) {
// 	// 	m_Xposition = x;
// 	// }

// 	// void setYPosition(double y) {
// 	// 	m_Yposition = y;
// 	// }
// 	double get_rotation( );
// 	uint8_t IMU_PORT= 1;
//     //pros::Imu(const std::uint8_t port);
// 	void updateEncoders(double wheeldata);

// 	std::vector<double> getRobotPositionWorld();
// 	void init();
// private:
// 	double m_Xposition = 0.0;
// 	double m_Yposition = 0.0;
// 	double m_Angle = 0.0;
// 	double m_deltaAngle = 0.0;
// 	const double m_ticks = 300;
// 	//const double m_Wheeldistance = 5.5;
// 	const double m_circumference = 10.2101761242;
// };

// }