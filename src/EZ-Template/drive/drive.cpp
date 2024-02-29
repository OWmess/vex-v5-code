/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "drive.hpp"

#include <list>
#include <utility>

#include "main.h"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"

using namespace ez;

// Constructor for integrated encoders
Drive::Drive(pros::MotorGroup &left_group, pros::MotorGroup &right_group,
             pros::Imu &imu_ref, double wheel_diameter, double ticks, double ratio)
    : imu(imu_ref),left_motors(left_group), right_motors(right_group),
      ez_auto([this] { this->ez_auto_task(); }) {


  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();
  set_defaults();
}

Drive::Drive(pros::MotorGroup &left_group, pros::MotorGroup &right_group,pros::Imu &imu_ref, double wheel_diameter, double ticks, double ratio,double wheel_distance)
:Drive(left_group,right_group,imu_ref,wheel_diameter,ticks,ratio){
  this->WHEEL_DISTANCE=wheel_distance;
}

void Drive::set_defaults() {
  // PID Constants
  headingPID = {11, 0, 20, 0};
  forward_drivePID = {0.45, 0, 5, 0};
  backward_drivePID = {0.45, 0, 5, 0};
  turnPID = {5, 0.003, 35, 15};
  swingPID = {7, 0, 45, 0};
  leftPID = {0.45, 0, 5, 0};
  rightPID = {0.45, 0, 5, 0};
  set_turn_min(30);
  set_swing_min(30);

  // Slew constants
  set_slew_min_power(80, 80);
  set_slew_distance(7, 7);

  // Exit condition constants
  set_exit_condition(turn_exit, 100, 3, 500, 7, 500, 5000);
  set_exit_condition(swing_exit, 100, 3, 500, 7, 500, 5000);
  set_exit_condition(drive_exit, 80, 50, 300, 150, 500, 5000);

  // Modify joystick curve on controller (defaults to disabled)
  toggle_modify_curve_with_controller(true);

  // Left / Right modify buttons
  set_left_curve_buttons(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Enable auto printing and drive motors moving
  toggle_auto_drive(true);
  toggle_auto_print(true);

  // Disables limit switch for auto selector
  as::limit_switch_lcd_initialize(nullptr, nullptr);
}

double Drive::get_tick_per_inch() {
  CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

  TICK_PER_REV = (50.0 * (3600.0 / CARTRIDGE)) * RATIO;  // with no cart, the encoder reads 50 counts per rotation

  TICK_PER_INCH = (TICK_PER_REV / CIRCUMFERENCE);
  return TICK_PER_INCH;
}

void Drive::set_pid_constants(PID* pid, double p, double i, double d, double p_start_i) {
  pid->set_constants(p, i, d, p_start_i);
}

void Drive::set_tank(int left, int right) {
  if (pros::millis() < 1500) return;

  for(int i=0;i<left_motors.size();i++){
    if (!pto_active_check(left_motors[i])) left_motors[i].move_voltage(left * (12000.0 / 127.0));  // If the motor is in the pto list, don't do anything to the motor.
  }
  for(int i=0;i<right_motors.size();i++){
    if (!pto_active_check(right_motors[i])) right_motors[i].move_voltage(right * (12000.0 / 127.0));  // If the motor is in the pto list, don't do anything to the motor.
  }
}

void Drive::set_drive_current_limit(int mA) {
  if (abs(mA) > 2500) {
    mA = 2500;
  }
  CURRENT_MA = mA;
  for(int i=0;i<left_motors.size();i++){
    if (!pto_active_check(left_motors[i])) left_motors[i].set_current_limit(abs(mA));  // If the motor is in the pto list, don't do anything to the motor.
  }
  for(int i=0;i<right_motors.size();i++){
    if (!pto_active_check(right_motors[i])) right_motors[i].set_current_limit(abs(mA));  // If the motor is in the pto list, don't do anything to the motor.
  }
}

// Motor telemetry
void Drive::reset_drive_sensor() {
  // left_motors.front().tare_position();
  // right_motors.front().tare_position();
  left_motors.tare_position();
  right_motors.tare_position();
}

double Drive::right_sensor() {
  double position=right_motors[right_condition_index].get_position();
  if(std::isinf(position)){
    for(int i=0;i<right_motors.size();i++){
      cout<<"new right motor index: "<<right_motors[i].get_flags()<<"\n";
      if(!std::isinf(right_motors[i].get_position())){
        right_condition_index=i;
        break;
      }
    }
    position=right_motors[right_condition_index].get_position();
  }
  
  return position;
}

int Drive::right_velocity() { return right_motors[right_condition_index].get_actual_velocity(); }
double Drive::right_mA() { return right_motors[right_condition_index].get_current_draw(); }
bool Drive::right_over_current() { return right_motors[right_condition_index].is_over_current(); }

double Drive::left_sensor() {
  double position=left_motors[left_condition_index].get_position();
  if(std::isinf(position)){
    for(int i=0;i<left_motors.size();i++){
      cout<<"new left motor index: "<<left_motors[i].get_flags()<<"\n";
      if(!std::isinf(left_motors[i].get_position())){
        left_condition_index=i;
        break;
      }
    }
    position=left_motors[left_condition_index].get_position();
  }
  return position;
}

int Drive::left_velocity() { return left_motors[left_condition_index].get_actual_velocity(); }
double Drive::left_mA() { return left_motors[left_condition_index].get_current_draw(); }
bool Drive::left_over_current() { return left_motors[left_condition_index].is_over_current(); }

void Drive::reset_gyro(double new_heading) { imu.set_rotation(new_heading); }
double Drive::get_gyro() {
  //   //中值滤波
  // constexpr static int windows_size=5;
  // static std::deque<double> windows;
  // windows.push_back(imu.get_rotation());
  // if(windows.size()>windows_size)
  //   windows.pop_front();
  // std::sort(windows.begin(),windows.end());
  // gyro_data=std::accumulate(windows.begin(),windows.end(),0.0)/windows.size();
  return imu.get_rotation(); 

}

void Drive::imu_loading_display(int iter) {
  // If the lcd is already initialized, don't run this function
  if (pros::lcd::is_initialized()) return;

  // Boarder
  int boarder = 50;

  // Create the boarder
  pros::screen::set_pen(COLOR_WHITE);
  for (int i = 1; i < 3; i++) {
    pros::screen::draw_rect(boarder + i, boarder + i, 480 - boarder - i, 240 - boarder - i);
  }

  // While IMU is loading
  if (iter < 2000) {
    static int last_x1 = boarder;
    pros::screen::set_pen(0x00FF6EC7);  // EZ Pink
    int x1 = (iter * ((480 - (boarder * 2)) / 2000.0)) + boarder;
    pros::screen::fill_rect(last_x1, boarder, x1, 240 - boarder);
    last_x1 = x1;
  }
  // Failsafe time
  else {
    static int last_x1 = boarder;
    pros::screen::set_pen(COLOR_RED);
    int x1 = ((iter - 2000) * ((480 - (boarder * 2)) / 1000.0)) + boarder;
    pros::screen::fill_rect(last_x1, boarder, x1, 240 - boarder);
    last_x1 = x1;
  }
}

bool Drive::imu_calibrate(bool run_loading_animation) {
  imu.reset();
  int iter = 0;
  while (true) {
    iter += util::DELAY_TIME;

    if (run_loading_animation) imu_loading_display(iter);

    if (iter >= 2000) {
      if (!(imu.get_status() & pros::c::E_IMU_STATUS_CALIBRATING)) {
        imu.tare();
        break;
      }
      if (iter >= 3000) {
        printf("No IMU plugged in, (took %d ms to realize that)\n", iter);
        return false;
      }
    }
    pros::delay(util::DELAY_TIME);
  }
  master_controller.rumble(".");
  printf("IMU is done calibrating (took %d ms)\n", iter);
  return true;
}

// Brake modes
void Drive::set_drive_brake(pros::motor_brake_mode_e_t brake_type) {
  CURRENT_BRAKE = brake_type;
  for(int i=0;i<left_motors.size();i++){
    if (!pto_active_check(left_motors[i])) left_motors[i].set_brake_mode(brake_type);  // If the motor is in the pto list, don't do anything to the motor.
  }
  for(int i=0;i<right_motors.size();i++){
    if (!pto_active_check(right_motors[i])) right_motors[i].set_brake_mode(brake_type);  // If the motor is in the pto list, don't do anything to the motor.
  }
}

void Drive::initialize() {
  init_curve_sd();
  imu_calibrate(true);
  reset_drive_sensor();
}

void Drive::toggle_auto_drive(bool toggle) { drive_toggle = toggle; }
void Drive::toggle_auto_print(bool toggle) { print_toggle = toggle; }

void Drive::set_pid_logger(bool toggle) { pid_logger = toggle; }