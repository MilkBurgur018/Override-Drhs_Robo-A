#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

#include "globals.hpp"

void on_center_button() {

}

void initialize() {

}

//disabled robot state
void disabled() {}

//post init pre auton state
void competition_initialize() {}

//auton
void autonomous() {}

//driver
void opcontrol() {
	while (true) {
		// Arcade control scheme
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);		
		chassis.arcade(leftY, rightX, false, 0.75);    //75% priority
		pros::delay(20);                               // Run for 20 ms then update
	}
}

//TODO tracking wheel offsets. check lem lib documentation.