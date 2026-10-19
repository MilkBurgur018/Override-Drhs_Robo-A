#pragma once
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/motor_group.hpp"

extern int AutonIndex;

extern pros::Controller master;

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::MotorGroup lift;

extern pros::adi::Pneumatics claw;

extern lemlib::Drivetrain drivetrain;

extern pros::Imu imu;

extern pros::Rotation para_rot_l;
extern pros::Rotation para_rot_r;
extern pros::Rotation the_hozzy_one;

extern lemlib::TrackingWheel leftVTW;
extern lemlib::TrackingWheel rightVTW;
extern lemlib::TrackingWheel theHTW;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;