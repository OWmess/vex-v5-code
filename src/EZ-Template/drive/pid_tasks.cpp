/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include "pros/misc.hpp"
#include <chrono>
#include <cmath>
using namespace ez;

void Drive::ez_auto_task() {
  while (true) {
    // Autonomous PID
    if (get_mode() == DRIVE)
      drive_pid_task();
    else if (get_mode() == TURN)
      turn_pid_task();
    else if (get_mode() == SWING)
      swing_pid_task();
    if(get_mode()==TRUN_GYRO_FREE)
      turn_pid_gyro_free_task();
    if (pros::competition::is_autonomous() && !util::AUTON_RAN)
      util::AUTON_RAN = true;
    else if (!pros::competition::is_autonomous())
      set_mode(DISABLE);
      ;

    pros::delay(util::DELAY_TIME);
  }
}

// Drive PID task
void Drive::drive_pid_task() {
  // Compute PID
  double l_sensor = left_sensor();
  double r_sensor = right_sensor();
  static double prev_l_sensor=l_sensor;
  static double prev_r_sensor=r_sensor;
  double gyro_pos = get_gyro();

  //inclined check
  bool incline=false;
  static const char* path="/usd/incline.csv";

  float degree=abs(fmod(imu_initial_heading,180)==0?imu.get_pitch():imu.get_roll());
  double delta_l_sensor=l_sensor-prev_l_sensor;
  double delta_r_sensor=r_sensor-prev_r_sensor;
  if(incline_check){
    // 根据imu的初始朝向判断是pitch还是roll,取绝对值
    // float degree=abs(fmod(imu_initial_heading,180)==0?imu.get_pitch():imu.get_roll());

    incline_deg_vec.push_back(degree);
    //保证incline_deg_vec的长度不超过5
    if(incline_deg_vec.size()>5)
      incline_deg_vec.pop_front();
    //求队列最大项
    float max_element=*std::max_element(incline_deg_vec.begin(),incline_deg_vec.end());

    if(max_element>incline_deg_threshold){
      incline=true;
      //如果倾斜角度大于阈值，则将target加上上一次的偏移量
      leftPID.set_target(leftPID.get_target()+delta_l_sensor);
      rightPID.set_target(rightPID.get_target()+delta_r_sensor);

    }

    std::cout<<"degree:"<<degree<<std::endl;
  }
  util::write_to_csv(path,degree,delta_l_sensor,delta_r_sensor,leftPID.get_target(),rightPID.get_target(),std::to_string(incline));





  leftPID.compute(l_sensor);
  rightPID.compute(r_sensor);
  headingPID.compute(gyro_pos);
  if(pid_logger){
    l_sensor_vec.emplace_back(l_sensor);
    r_sensor_vec.emplace_back(r_sensor);
    gyro_vec.emplace_back(gyro_pos);
  }
  // Compute slew
  double l_slew_out = slew_calculate(left_slew, left_sensor());
  double r_slew_out = slew_calculate(right_slew, right_sensor());

  // Clip leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
  double l_drive_out = util::clip_num(leftPID.output, l_slew_out, -l_slew_out);
  double r_drive_out = util::clip_num(rightPID.output, r_slew_out, -r_slew_out);


  // Toggle heading
  double gyro_out = heading_on&&!incline ? headingPID.output : 0;

  if(isnan(gyro_out)) {//check if gyro data is nan(gyro is not working)
    gyro_out = 0;
    printf("check gyro\n");
  }

  // Combine heading and drive
  double l_out = l_drive_out + gyro_out;
  double r_out = r_drive_out - gyro_out;
  // Set motors
  if (drive_toggle)
    set_tank(l_out, r_out);
  prev_l_sensor=l_sensor;
  prev_r_sensor=r_sensor;
}

// Turn PID task
void Drive::turn_pid_task() {
  // Compute PID
  double gyro_pos=get_gyro();
  turnPID.compute(gyro_pos);
  if(pid_logger)
    gyro_vec.emplace_back(gyro_pos);

  // Clip gyroPID to max speed
  double gyro_out = util::clip_num(turnPID.output, max_speed, -max_speed);

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (turnPID.constants.ki != 0 && (fabs(turnPID.get_target()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
    if (get_turn_min() != 0)
      gyro_out = util::clip_num(gyro_out, get_turn_min(), -get_turn_min());
  }

  // Set motors
  if (drive_toggle)
    set_tank(gyro_out, -gyro_out);
}

// Swing PID task
void Drive::swing_pid_task() {
  // Compute PID
  double gyro_pos=get_gyro();
  swingPID.compute(gyro_pos);
  if(pid_logger)
    gyro_vec.emplace_back(gyro_pos);
  // Clip swingPID to max speed
  double swing_out = util::clip_num(swingPID.output, max_speed, -max_speed);

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (swingPID.constants.ki != 0 && (fabs(swingPID.get_target()) > swingPID.constants.start_i && fabs(swingPID.error) < swingPID.constants.start_i)) {
    if (get_swing_min() != 0)
      swing_out = util::clip_num(swing_out, get_swing_min(), -get_swing_min());
  }

  if (drive_toggle) {
    // Check if left or right swing, then set motors accordingly
    if (current_swing == LEFT_SWING)
      set_tank(swing_out, 0);
    else if (current_swing == RIGHT_SWING)
      set_tank(0, -swing_out);
  }
}

void Drive::turn_pid_gyro_free_task(){
    // Compute PID
  double l_out=leftPID.compute(left_sensor());
  double r_out=rightPID.compute(right_sensor());
  double l_max_speed=ez::util::sgn(l_out)*max_speed;
  double r_max_speed=ez::util::sgn(r_out)*max_speed;
  if(abs(l_out)>l_max_speed)
    l_out=l_max_speed;
  if(abs(r_out)>r_max_speed)
    r_out=r_max_speed;
  set_tank(l_out, r_out);

}