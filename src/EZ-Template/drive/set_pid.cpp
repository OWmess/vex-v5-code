/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Updates max speed
void Drive::set_max_speed(int speed) {
  max_speed = util::clip_num(abs(speed), 127, -127);
}

void Drive::reset_pid_targets() {
  headingPID.set_target(0);
  leftPID.set_target(0);
  rightPID.set_target(0);
  forward_drivePID.set_target(0);
  backward_drivePID.set_target(0);
  turnPID.set_target(0);
}

void Drive::set_angle(double angle) {
  headingPID.set_target(angle);
  reset_gyro(angle);
}

void Drive::set_mode(e_mode p_mode) {
  mode = p_mode;
}

void Drive::set_turn_min(int min) { turn_min = abs(min); }
int Drive::get_turn_min() { return turn_min; }

void Drive::set_swing_min(int min) { swing_min = abs(min); }
int Drive::get_swing_min() { return swing_min; }

e_mode Drive::get_mode() { return mode; }

// Set drive PID
void Drive::set_drive_pid(double target, int speed, bool slew_on, bool toggle_heading) {
  TICK_PER_INCH = get_tick_per_inch();
  // Print targets
  if (print_toggle) printf("Drive Started... Target Value: %f (%f ticks)", target, target * TICK_PER_INCH);
  if (slew_on && print_toggle) printf(" with slew");
  if (print_toggle) printf("\n");

  // Global setup
  set_max_speed(speed);
  heading_on = toggle_heading;
  bool is_backwards = false;
  l_start = left_sensor();
  r_start = right_sensor();
  double l_target_encoder, r_target_encoder;

  // Figure actual target value
  l_target_encoder = l_start + (target * TICK_PER_INCH);
  r_target_encoder = r_start + (target * TICK_PER_INCH);

  // Figure out if going forward or backward
  if (l_target_encoder < l_start && r_target_encoder < r_start) {
    auto consts = backward_drivePID.get_constants();
    leftPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
    rightPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
    is_backwards = true;
    if(pid_logger)
      logger.create_log(DriveMode::BACKWARD, consts, l_target_encoder, r_target_encoder);
  } else {
    auto consts = forward_drivePID.get_constants();
    leftPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
    rightPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
    is_backwards = false;
    if(pid_logger)
      logger.create_log(DriveMode::FORWARD, consts, l_target_encoder, r_target_encoder);
  }

  // Set PID targets
  leftPID.set_target(l_target_encoder);
  rightPID.set_target(r_target_encoder);

  // Initialize slew
  slew_initialize(left_slew, slew_on, max_speed, l_target_encoder, left_sensor(), l_start, is_backwards);
  slew_initialize(right_slew, slew_on, max_speed, r_target_encoder, right_sensor(), r_start, is_backwards);

  // Run task
  set_mode(DRIVE);
}

// Set turn PID
void Drive::set_turn_pid(double target, int speed) {
  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", target);

  // Set PID targets
  turnPID.set_target(target);
  headingPID.set_target(target);  // Update heading target for next drive motion
  set_max_speed(speed);
  if(pid_logger)
    logger.create_log(DriveMode::TURN, turnPID.get_constants(), target);

  // Run task
  set_mode(TURN);
}

// Set swing PID
void Drive::set_swing_pid(e_swing type, double target, int speed) {
  // Print targets
  if (print_toggle) printf("Swing Started... Target Value: %f\n", target);
  current_swing = type;

  // Set PID targets
  swingPID.set_target(target);
  headingPID.set_target(target);  // Update heading target for next drive motion
  set_max_speed(speed);
  if(pid_logger)
    logger.create_log(DriveMode::SWING, swingPID.get_constants(), target);
    
  // Run task
  set_mode(SWING);
}

void Drive::set_turn_pid_gyro_free(double target, int speed) {
  TICK_PER_INCH=get_tick_per_inch();
  //左右两轮间距(inch)
  target=WHEEL_DISTANCE*M_PI*target/360.0;
  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", target);
  // Set PID targets

  

  l_start=left_sensor();
  r_start=right_sensor();
  printf("l_start:%f,r_start:%f\n",l_start,r_start);
  double l_target_encoder, r_target_encoder;
  // Figure actual target value
  l_target_encoder = l_start + (target * TICK_PER_INCH);
  r_target_encoder = r_start - (target * TICK_PER_INCH);


  printf("l_target_encoder:%f,r_target_encoder:%f\n",l_target_encoder,r_target_encoder);
  auto consts=turnPID_gyro_free.get_constants();
  leftPID.set_constants(consts.kp,consts.ki,consts.kd,consts.start_i);
  rightPID.set_constants(consts.kp,consts.ki,consts.kd,consts.start_i);

  leftPID.set_target(l_target_encoder);
  rightPID.set_target(r_target_encoder);
  set_max_speed(speed);


  // Run task
  set_mode(TRUN_GYRO_FREE);
}

 void Drive::set_drive_pid_with_incline_check(double target, int speed, bool slew_on, bool toggle_heading,float deg,int imu_initial_heading){
  this->imu_initial_heading=imu_initial_heading;
  this->incline_deg_threshold=deg;
  this->incline_check=true;
  set_drive_pid(target,speed,slew_on,toggle_heading);
 }

 void Drive::set_arc_drive_pid(double target, int left_speed, int right_speed, bool slew_on) {
  TICK_PER_INCH = get_tick_per_inch();
  // Print targets
  if (print_toggle) printf("Arc Drive Started... Target Value: %f (%f ticks)", target, target * TICK_PER_INCH);
  if (slew_on && print_toggle) printf(" with slew");
  if (print_toggle) printf("\n");

  float spd = (left_speed - right_speed) / 2.0 + right_speed;
  float l_target = left_speed / spd * target;
  float r_target = right_speed / spd * target;
  // printf("spd: %f,l_target:%f,r_target:%f\n",spd,l_target,r_target);
  // Global setup
  heading_on = false;
  bool l_is_backwards = false;
  bool r_is_backwards = false;
  l_start = left_sensor();
  r_start = right_sensor();
  double l_target_encoder, r_target_encoder;

  // Figure actual target value
  l_target_encoder = l_start + (l_target * TICK_PER_INCH);
  r_target_encoder = r_start + (r_target * TICK_PER_INCH);
  // printf("l_target_encoder: %lf ,r_target_encoder: %lf",l_target_encoder,r_target_encoder);
  // Figure out if going forward or backward
  auto straight_consts = forward_drivePID.get_constants();
  auto backward_consts = backward_drivePID.get_constants();
  if (l_target_encoder < l_start) {
    leftPID.set_constants(backward_consts.kp, backward_consts.ki, backward_consts.kd, backward_consts.start_i);
    l_is_backwards = true;
  } else {
    leftPID.set_constants(straight_consts.kp, straight_consts.ki, straight_consts.kd, straight_consts.start_i);
  }
  if (r_target_encoder < r_start) {
    rightPID.set_constants(backward_consts.kp, backward_consts.ki, backward_consts.kd, backward_consts.start_i);
    r_is_backwards = true;
  } else {
    rightPID.set_constants(straight_consts.kp, straight_consts.ki, straight_consts.kd, straight_consts.start_i);
  }

  // Set PID targets
  leftPID.set_target(l_target_encoder);
  rightPID.set_target(r_target_encoder);

  // Initialize slew
  slew_initialize(left_slew, slew_on, left_speed, l_target_encoder, left_sensor(), l_start, l_is_backwards);
  slew_initialize(right_slew, slew_on, right_speed, r_target_encoder, right_sensor(), r_start, r_is_backwards);

  // Run task
  set_mode(DRIVE);
 }

 void Drive::set_arc_turn_pid(double target, int left_speed,int right_speed) {
  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", target);

  // Set PID targets
  turnPID.set_target(target);
  headingPID.set_target(target);  // Update heading target for next drive motion
  this->l_max_spd=left_speed;
  this->r_max_spd=right_speed;
  if(pid_logger)
    logger.create_log(DriveMode::TURN, turnPID.get_constants(), target);

  // Run task
  set_mode(ARC_TURN);
}