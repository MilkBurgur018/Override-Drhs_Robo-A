#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

#include "globals.hpp"
#include "auton.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

bool pneumastate = false;

void on_center_button() {

}

void initialize() {
	//debug auton selecting
	lv_obj_t * auton_label = lv_label_create(lv_screen_active());
    lv_obj_align(auton_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text_fmt(auton_label, "Current Auton Index: %d", AutonIndex);

	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

//disabled robot state
void disabled() {}

//post init pre auton state
void competition_initialize() {}

//auton
void autonomous() {
	switch (AutonIndex) {
		case 0:
			kP_kD_Calibration();
	}
}

//driver
void opcontrol() {
	while (true) {
		// Arcade control scheme
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);		
		chassis.arcade(leftY, rightX, false, 0.75);    //75% priority on steering

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      	AutonIndex++;
    	}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      	AutonIndex--;
    	}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			lift.move_voltage(-12000);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			lift.move_voltage(12000);
		} else {
			lift.move_velocity(0);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			if (pneumastate) {
				pneumastate = false;
				claw.set_value(pneumastate);
			} else {
				pneumastate = true;
				claw.set_value(pneumastate);
			}
		}
		pros::delay(20);                               // Run for 20 ms then update
	}
}

//TODO: kP kD tune