#include "main.h"
// #include "motor.hpp"
#include "everything.hpp"
// #include "motor.cpp"
using namespace okapi;
#define IMU_PORT 10

pros::ADIGyro gyro('H',0.91);
pros::Imu imu_sensor(IMU_PORT);


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	imu_sensor.reset();
	int time = pros::millis();
	  int iter = 0;
	  while (imu_sensor.is_calibrating()) {
	    printf("IMU calibrating... %d\n", iter);
	    iter += 10;
	    pros::delay(10);
			pros::lcd::register_btn1_cb(on_center_button);
		}
		printf("IMU is done calibrating (took %d ms)\n", iter - time);


	//imu_sensor.reset();
	// int time = pros::millis();
	//   int iter = 0;
	//   while (imu_sensor.is_calibrating()) {
	//     printf("IMU calibrating... %d\n", iter);
	//     iter += 10;
	    pros::delay(10);
	pros::lcd::register_btn1_cb(on_center_button);
}
// printf("IMU is done calibrating (took %d ms)\n", iter - time);


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 void autoTray(int voltage,int degrees){
  tray.move_absolute(degrees,voltage);

}
 void resetDriveEncoders(){
   driveLeftBack.tare_position();
   driveLeftFront.tare_position();
   driveRightBack.tare_position();
   driveRightFront.tare_position();
 }

 double avgDriveEncoderValue(){
   return (fabs(driveLeftFront.get_position())+
   fabs(driveLeftBack.get_position())+
   fabs(driveRightFront.get_position())+
   fabs(driveRightBack.get_position())) / 4;
 }
 double leftEncoderValue(){
   return  (fabs(driveLeftFront.get_position())+
    fabs(driveLeftBack.get_position())) / 2;
 }


double rightEncoderValue(){
  return  (fabs(driveRightFront.get_position())+
   fabs(driveRightBack.get_position())) / 2;
}
 void setTray(){
   for(int i=0;i<3400;i=i+10) {
     set_tray_pid(i);
     pros::delay(20);
   }
 }
 void resetIMU(){
	imu_sensor.reset();
}

 double get_true_heading(){
	if(imu_sensor.get_heading() > 180){
		return -(360 -imu_sensor.get_heading());
	}
	else if(imu_sensor.get_heading() < 180){
		return imu_sensor.get_heading();
	}
	else{
		return imu_sensor.get_heading();
	}
}

 void turnIMU(double angle){

    // resetIMU();
 	 // delay(2100);
    double kP = 0.75;
    //double kI = 0;
    //double kD = 0;
    double error = 0;
 	 double output = 0;
 	 error = (angle-get_true_heading());
    while(fabs(error) > 5){

 		 pros::lcd::set_text(2, std::to_string(get_true_heading()));
 		 output = kP*error;

 		 if(fabs(output) > 127){
         output = 127*fabs(output)/output;
      }
      else if(fabs(output) < 20){
        output = 30*fabs(output)/output;
      }
 		 set_tank(output, output);
 		 error = (angle-get_true_heading());
      delay(50);
    }

    set_tank(0, 0);


    // imu_sensor.get_heading();
    // imu_sensor.get_rotation();
  }
 void retractTray(){
   set_tray_pid(0);
   pros::delay(20);
 }



 void turn(int voltage, int angle){
   //intc direction = abs(units)/units;//Will either be 1 or -1
   //Reset Motors and Gyros
   resetDriveEncoders();

   while(leftEncoderValue() < angle) {
     set_tank(voltage  , voltage );
     pros::delay(10);
     set_tank(0,0);
   }
 }

// int units int voltage
// youre a loser

void translate(int units,int voltage){
   //Define a direction based on units provided
   int direction = abs(units)/units;//Will either be 1 or -1
   //Reset Motors and Gyros
   resetDriveEncoders();
  // gyro.reset();
   //Drive Forward until units are reached
   while(avgDriveEncoderValue() < abs(units)) {
     set_tank(voltage * direction, -voltage * direction);
     pros::delay(10);
   }
   //Brief Brake
   set_tank(10 * direction,10 * direction);
   pros::delay(50);
   //Set Drive back to neutral
   set_tank(0,0);

 }


// only takes in positive angles
// its bad but bear with it
// only able to turn right for right now
// also only takes in the angle you want to turn to, not the angle to turn
 // void turnIMU(double angle){
 //
 //   // resetIMU();
	//  // delay(2100);
 //   double kP = 0.2;
 //   //double kI = 0;
 //   //double kD = 0;
 //   double error = 0;
	//  double output = 0;
	//  error = (angle-get_true_heading());
 //   while(fabs(error) > 5){
 //
	// 	 pros::lcd::set_text(2, std::to_string(get_true_heading()));
	// 	 output = kP*error;
 //
	// 	 if(fabs(output) > 127){
 //        output = 127*fabs(output)/output;
 //     }
 //     else if(fabs(output) < 30){
 //       output = 30*fabs(output)/output;
 //     }
	// 	 set_tank(output, -output);
	// 	 error = (angle-get_true_heading());
 //     delay(50);
 //   }
 //
 //   set_tank(0, 0);
 //
 //
 //   // imu_sensor.get_heading();
 //   // imu_sensor.get_rotation();
 // }




void autonomous() {
		translate(500,50);
		delay(200);
		translate(-500,50);
		delay(200);
	  autoTray(200,2800);
		delay(200);
		translate(200,50);
		delay(100);
		translate(-170,50);
		delay(300);
		autoTray(-127,-2800);
		delay(1000);
		arm.move(1000);
		delay(400);
		arm.move(-1000);
		delay(200);
		arm.move(0);
		delay(200);
	  left_roller.move(127);
 	  right_roller.move(-127);
 	  translate(1500,70);
    delay(700);
 	  left_roller.move(0);
	  right_roller.move(0);
    delay(200);
		//voltage and angle
	  turn(-50,525);
    delay(300);
    left_roller.move(127);
	  right_roller.move(-127);
 	  translate(1500,70);
    left_roller.move(0);
    right_roller.move(0);
    turn(-60,100);
    delay(200);
		translate(500,60);
		delay(700);
		autoTray(200, 2800);
		delay(1000);
    	// left_roller.move(-127);
	  	// right_roller.move(127);
		translate(50, 20);
		delay(200);
		translate(-1000, 50);
}


 void
tray_control(void*) {
	pros::Controller master(CONTROLLER_MASTER);
	pros::Task tray_t(tray_pid);
	bool b_toggle = false;
	while (true) {
		if (master.get_digital(DIGITAL_Y)) {
			b_toggle = !b_toggle;

			if (b_toggle) {
				for(int i=0;i<3500;i=i+25) {
					set_tray_pid(i);
					pros::delay(20);
				}
			} else {
				set_tray_pid(0);
			}

			while (master.get_digital(DIGITAL_Y)) {
				pros::delay(10);
			}
		}

		pros::delay(20);
	}
}
void
arm_control(void*) {
	pros::Controller master(CONTROLLER_MASTER);
	pros::Task arm_t(arm_pid);
	bool was_pid;
	while (true) {
		if (master.get_digital(DIGITAL_B)) {
			was_pid = true;
			arm_t.resume();
			set_arm_pid(2300);
		} else if (master.get_digital(DIGITAL_DOWN)) {
			was_pid = true;
			arm_t.resume();
			set_arm_pid(1800);
		} else {
			if (master.get_digital(DIGITAL_R1)||master.get_digital(DIGITAL_R2)) {
				was_pid = false;
				set_arm((master.get_digital(DIGITAL_R1)-master.get_digital(DIGITAL_R2))*127);
			} else {
				if (!was_pid) {
					set_arm(0);
				}
			}
		}

		if (!was_pid) {
			arm_t.suspend();
		}

		pros::delay(20);
	}
}

void opcontrol() {
	pros::Controller master(CONTROLLER_MASTER);
		pros::Task tray_control_t(tray_control);
		bool b_toggle = false;
	// if(master.get_digital(DIGITAL_RIGHT)){
	// 	LEDLights.state(100);
	// }
	// else if(master.get_digital(DIGITAL_LEFT)){
	// 	LEDLights.state(0);
	pros::Task t(arm_control);
	while (true) {

		set_tank(master.get_analog(ANALOG_LEFT_Y), -master.get_analog(ANALOG_RIGHT_Y));

		if (master.get_digital(DIGITAL_A)) {
			b_toggle = !b_toggle;


			if (b_toggle) {
				for(int i=0;i<600;i=i+21) {
					set_tray_pid(i);
					pros::delay(20);
				}
			} else {
				set_tray_pid(0);
			}


			while (master.get_digital(DIGITAL_A)) {
				pros::delay(10);
			}
		}
		//set_arm((master.get_digital(DIGITAL_R1)-master.get_digital(DIGITAL_R2))*127);

		if (master.get_digital(DIGITAL_L1)) {
			set_rollers(127);
		} else if (master.get_digital(DIGITAL_L2)) {
			set_rollers(-127);
		} else if (master.get_digital(DIGITAL_RIGHT)) {
			set_rollers(70);
		} else {
			set_rollers(0);
		}


		pros::delay(20);
	}

	}
