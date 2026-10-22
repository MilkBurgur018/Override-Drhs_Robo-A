#include "globals.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"

int AutonIndex = 0;

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({12, 13}, pros::MotorGearset::green);// Creates a motor group with forwards ports 0 0 0 
pros::MotorGroup right_mg({-2, -3}, pros::MotorGearset::green);// Creates a motor group with forwards ports 0 0 0
pros::MotorGroup lift({-1, 11}, pros::MotorGearset::green);

pros::adi::Pneumatics claws('A', false); 
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg,                      // left motor group
                               &right_mg,                    // right motor group
                               13.5,                           // track width (inches) — PLACEHOLDER
                               lemlib::Omniwheel::OLD_325,     // wheel diameter — PLACEHOLDER, confirm wheel type
                               360,                          // drivetrain rpm — PLACEHOLDER, confirm motor gearing
                               2                             // horizontal drift — PLACEHOLDER
);
pros::Imu imu(10); //imu port
//rotation sensors
pros::Rotation para_rot_l(19);  //parallel rotation sensor no.1 (left side)
pros::Rotation para_rot_r(21);  //parallel rotation sensor no.2 (right side)
pros::Rotation the_hozzy_one(20); //the horizontal rotation sensor (horizontal drift)
//Tracking Wheels init
// left and right VTW are 2.5" behind the tracking center based on robot front & 3" deviated from the center to either side. The hozzy one is center.
lemlib::TrackingWheel leftVTW(&para_rot_l, lemlib::Omniwheel::OLD_325, -2.5, 1.0/1.0);
lemlib::TrackingWheel rightVTW(&para_rot_r, lemlib::Omniwheel::OLD_325, -2.5, 1.0/1.0);
lemlib::TrackingWheel theHTW(&the_hozzy_one, lemlib::Omniwheel::OLD_325, 0, 1.0/1.0);
//config
lemlib::OdomSensors sensors(&leftVTW, // vertical tracking wheel 1
                            &rightVTW, // vertical tracking wheel 2
                            &theHTW, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);
//PID config
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// reverted sigmoid stick expo curve for throttle
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);
//steering RSSEC
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, 
                        &steer_curve
);
