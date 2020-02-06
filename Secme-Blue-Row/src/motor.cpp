#include "main.h"
// #include "motor.hpp"
#include "everything.hpp"

#define IMU_PORT 10

pros::Motor driveLeftBack(2,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveLeftFront(5,pros::E_MOTOR_GEARSET_18,false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightBack(3,pros::E_MOTOR_GEARSET_18,true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightFront(4,pros::E_MOTOR_GEARSET_18,true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Imu imu_sensor(IMU_PORT);
pros::Motor tray(15, MOTOR_GEARSET_36,true);
pros::Motor arm(18, MOTOR_GEARSET_18);
pros::Motor left_roller(20, MOTOR_GEARSET_18);
pros::Motor right_roller(11, MOTOR_GEARSET_18,false);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// okapi::TwoEncoderOdometry ok();
// okapi::Timer daTimer();
// okapi::Rate daRate;
// okapi::DefaultOdomChassisController drivetrain(okapi::Logger::getDefaultLogger(), );



//Math
int
sgn(int input) {
  if (input>0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}
int
clipnum(int input, int clip) {
  if (input > clip)
    return clip;
  else if (input < clip)
    return -clip;
  return input;
}

void
reset() {
  left_roller.set_brake_mode(MOTOR_BRAKE_HOLD);
  right_roller.set_brake_mode(MOTOR_BRAKE_HOLD);
  arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  arm.set_zero_position(0);
  tray.set_zero_position(0);
}

// void resetIMU(){
//   imu_sensor.reset();
// }
//
//
// void turnIMU(double angle){
//
//   resetIMU();
//   double kP = 1;
//   //double kI = 0;
//   //double kD = 0;
//   double error = 0;
//
//   while(error > 0){
//     error = kP*(imu_sensor.get_yaw() - angle);
//     if(error > 100){
//        error = 100;
//     }
//     else if(error < 20){
//       error = 20;
//     }
//
//
//     set_tank(error, -error);
//     delay(50);
//   }
//
//   set_tank(0, 0);
//
//
//   // imu_sensor.get_heading();
//   // imu_sensor.get_rotation();
// }

//Set motors
void set_tank(int left,int right){
  driveLeftFront = left;
  driveLeftBack = left;
  driveRightFront = right;
  driveRightBack = right;

}
//DRIVE CONTROL FUNCTIONS
void setDriveMotors() {
  int leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightJoystick = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  if(abs(leftJoystick) < 10){
    leftJoystick = 0;
  }
  if(abs(rightJoystick) < 10){
    rightJoystick = 0;
  }
  set_tank(leftJoystick,rightJoystick);

}

void
set_tray(int input) {
  tray.move(input);
}

void
set_arm(int input) {
  arm.move(input);
}

void
set_rollers(int input) {
  left_roller.move(-input);
  right_roller.move(input);
}

//PID
int t_target;
void
set_tray_pid(int input) {
  t_target = input;
}
void
tray_pid(void*) {
	while (true) {
		set_tray((t_target-tray.get_position())*0.4);
		pros::delay(20);
	}
}

int a_target;
void
set_arm_pid(int input) {
  a_target = input;
}
void
arm_pid(void*) {
  while (true) {
    set_arm((a_target-arm.get_position())*0.5);
    pros::delay(20);
  }
}
