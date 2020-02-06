#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

  // #include "everything.hpp"
  void set_tank(int input_l, int input_r);
  void set_tray(int input);
  void set_arm(int input);
  void set_rollers(int input);
  void set_tray_pid(int input);
  void tray_pid(void*);
  void set_arm_pid(int input);
  void arm_pid(void*);

  /*
    resets the arm and tray encoders
    @noParam
  */
  void reset();

//   /*
//     resets the IMU
//     @noParams
//   */
//   void resetIMU();
// // extern pros::Controller controller;
//   /*
//
//   */
//   void turnIMU(double angle);
// extern pros::ADIGyro gyro;
#endif
