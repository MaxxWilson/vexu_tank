// #include "main.hpp"
// #include "odometry.hpp"
//  #include <cmath>
//  #include <iostream>

 

// pros::ADIEncoder enc('A', 'B', true); // ports A and B, reversed
// pros::Rotation rot(1, false); // port 1, not reversed

// lemlib::TrackingWheel tracking_wheel(&enc, 3.25, 0, 2);

// pros::Imu inertial_sensor(2); //replacd that 2 with actual port IMU is connected to

// lemlib::Drivetrain_t drivetrain {
//     &leftDrive, // left drivetrain motors
//     &rightDrive, // right drivetrain motors
//     10, // change to actual track width
//     3.25, // wheel diameter
//     600 // wheel rpm (is it 600?)
// };

// lemlib::ChassisController_t lateralController {
//     8, // kP
//     30, // kD
//     1, // smallErrorRange
//     100, // smallErrorTimeout
//     3, // largeErrorRange
//     500, // largeErrorTimeout
//     5 // slew rate
// };
 
// // turning PID
// lemlib::ChassisController_t angularController {
//     4, // kP
//     40, // kD
//     1, // smallErrorRange
//     100, // smallErrorTimeout
//     3, // largeErrorRange
//     500, // largeErrorTimeout
//     0 // slew rate
// };

// lemlib::OdomSensors_t sensors {
// 	&tracking_wheel,
// 	nullptr,
// 	nullptr,
// 	nullptr,
// 	&inertial_sensor
// };

// lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// void screen() {
//     // loop forever
//     while (true) {
//         lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
//         pros::lcd::print(0, "x: %f", pose.x); // print the x position
//         pros::lcd::print(1, "y: %f", pose.y); // print the y position
//         pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
//         pros::delay(10);
//     }
// }

// void initialize() {
// 	 pros::lcd::initialize(); 
//     chassis.calibrate();
// 	pros::Task screenTask(screen);
// }











































// #include "main.hpp"
// #include "odometry.hpp"
// #include <cmath>
// #include <iostream>
// namespace ghost_tank {

// 	pros::Task* trackingTask = nullptr;
// TankOdometry::TankOdometry(){
// 	std::cout << "Constructor" << std::endl;
// }

// void TankOdometry::updateEncoders(double wheeldata){
// 	std::cout << "updateEncoders" << std::endl;
// double wheel = wheeldata;
// double Xinitial = 0;        // initial position of robot
// double Yinitial = 0;  
// double prevencoderdata = 0;
// double prevIMU= 0;
// double IMU = prevIMU;
// //To get initial encoder reading
// // if((m_Xposition == Xinitial) && (m_Yposition == Yinitial) ){
// // 		prevX = m_Xposition;
// // 		prevY = m_Yposition;
// // 	}

// double distance = (((wheel - prevencoderdata) / m_ticks) * m_circumference);
// pros::Imu imu_sensor(IMU_PORT);
// double deltaIMU= imu_sensor.get_rotation();
// prevencoderdata= wheel;

// if (deltaIMU==0){
// m_Xposition= distance *cos(IMU) ;
// m_Xposition= distance *cos(IMU) ;
// }
// else{
// double deltadistancex = distance *cos(deltaIMU);
// double deltadistancey = distance * sin(deltaIMU);
// m_Xposition += deltadistancex;   
// m_Yposition += deltadistancey; 
// IMU+= deltaIMU;
// m_Angle= IMU;
// }

// }
// std::vector<double> TankOdometry::getRobotPositionWorld(){
// 	std::cout << "getRobotPositionWorld" << std::endl;
// 	return {m_Xposition, m_Yposition, m_Angle};
// }


// void TankOdometry::init() {
//     if (trackingTask == nullptr) {
//         trackingTask = new pros::Task {[=] {
//             while (true) {
//                 updateEncoders(//put in sensor);
//                 pros::delay(10);
//             }
//         }};
//     }
// }
// }