#include "main.h"

// Chassis constructor
Drive chassis=Drive(
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-1, -2, -3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{11, 12, 13}

  // IMU Port
  /**
   * unused now
  */
  ,8

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.0

  // wheels distance
  ,12.0

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
   pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.1); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  // // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);
  // Autonomous Selector using LLEMU

  // Initialize chassis and auton selector

  chassis.initialize();
  ez::as::initialize();

  set_wings(false);
  set_hanger(false);
  
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
  
}

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

void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.
  set_wings(false);
  set_hanger(false);
  pros::delay(200);
  // auton_1();// 防守方案
  // auton_2();// 攻击方案
  auton_3();//1分钟全自动方案
  // ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.

  
}



/**
 * \return a vector of 5 bools, which are the state of the 5 buttons on the controller
*/
std::vector<int32_t> get_controller_button(){

  auto r1=master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
  auto r2=master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
  auto l1=master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
  auto l2=master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
  auto lift=master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
  auto up=master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
  auto down=master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
  auto right=master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
  return {r1,r2,l1,l2,lift,up,down,right};

}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on.
  while (true)
  {
    chassis.tank(); // Tank control
    auto buttons_state=get_controller_button();
    if(buttons_state[0]){
      set_intake(true,100);
    }
    else if(buttons_state[1]){
      set_intake(false,100);
    }
    else{
      set_intake(false,0);
    }
    if(buttons_state[2]){
      set_wings(true);
    }
    else if(buttons_state[3]){
      set_wings(false);
    }
    if(buttons_state[4]){
      set_lift(true,80);
    }
    if(buttons_state[5]){
      set_hanger(false);
    }else if(buttons_state[6]){
      set_hanger(true);
    }
    if(buttons_state[7]){
      set_lift_mid();
    }
    pros::delay(ez::util::DELAY_TIME); // 让代码休眠一下以防止过度占用处理器资源
  }

}
