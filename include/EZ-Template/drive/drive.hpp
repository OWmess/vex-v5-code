/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <iostream>
#include <tuple>

#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "pros/motors.h"
#include "EZ-Template/PID_Logger.hpp"
using namespace ez;

class Drive {
 public:
  /**
   * Joysticks will return 0 when they are within this number.  Set with set_joystick_threshold()
   */
  int JOYSTICK_THRESHOLD;

  /**
   * Global current brake mode.
   */
  pros::motor_brake_mode_e_t CURRENT_BRAKE = pros::E_MOTOR_BRAKE_COAST;

  /**
   * Global current mA.
   */
  int CURRENT_MA = 2500;

  /**
   * Current swing type.
   */
  e_swing current_swing;

  /**
   * Vector of pros motors for the left chassis.
   */
  std::vector<pros::Motor> left_motors;

  /**
   * Vector of pros motors for the right chassis.
   */
  std::vector<pros::Motor> right_motors;

  /**
   * Vector of pros motors that are disconnected from the drive.
   */
  std::vector<int> pto_active;

  /**
   * @brief pto list
   * 
   */
  std::vector<int> pto_list;
  /**
   * Inertial sensor.
   */
  pros::Imu imu;

  /**
   * Left tracking wheel.
   */
  pros::ADIEncoder left_tracker;

  /**
   * Right tracking wheel.
   */
  pros::ADIEncoder right_tracker;

  /**
   * Left rotation tracker.
   */
  pros::Rotation left_rotation;

  /**
   * Right rotation tracker.
   */
  pros::Rotation right_rotation;

  /**
   * PID objects.
   */
  PID headingPID;
  PID turnPID;
  PID forward_drivePID;
  PID leftPID;
  PID rightPID;
  PID backward_drivePID;
  PID swingPID;
  PID turnPID_gyro_free;
  /**
   * Current mode of the drive.
   */
  e_mode mode;

  /**
   * Sets current mode of drive.
   */
  void set_mode(e_mode p_mode);

  /**
   * Returns current mode of drive.
   */
  e_mode get_mode();

  /**
   * Calibrates imu and initializes sd card to curve.
   */
  void initialize();

  /**
   * Tasks for autonomous.
   */
  pros::Task ez_auto;

  /**
   * Creates a Drive Controller using internal encoders.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your drive wheels.  Remember 4" is 4.125"!
   * \param ticks
   *        Motor cartridge RPM
   * \param ratio
   *        External gear ratio, wheel gear / motor gear.
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio);

  /**
   * Creates a Drive Controller using encoders plugged into the brain.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your sensored wheels.  Remember 4" is 4.125"!
   * \param ticks
   *        Ticks per revolution of your encoder.
   * \param ratio
   *        External gear ratio, wheel gear / sensor gear.
   * \param left_tracker_ports
   *        Input {1, 2}.  Make ports negative if reversed!
   * \param right_tracker_ports
   *        Input {3, 4}.  Make ports negative if reversed!
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports);

  /**
   * Creates a Drive Controller using encoders plugged into a 3 wire expander.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your sensored wheels.  Remember 4" is 4.125"!
   * \param ticks
   *        Ticks per revolution of your encoder.
   * \param ratio
   *        External gear ratio, wheel gear / sensor gear.
   * \param left_tracker_ports
   *        Input {1, 2}.  Make ports negative if reversed!
   * \param right_tracker_ports
   *        Input {3, 4}.  Make ports negative if reversed!
   * \param expander_smart_port
   *        Port the expander is plugged into.
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports, int expander_smart_port);

  /**
   * Creates a Drive Controller using rotation sensors.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your sensored wheels.  Remember 4" is 4.125"!
   * \param ratio
   *        External gear ratio, wheel gear / sensor gear.
   * \param left_tracker_port
   *        Make ports negative if reversed!
   * \param right_tracker_port
   *        Make ports negative if reversed!
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ratio, int left_rotation_port, int right_rotation_port);

  /**
   * Sets drive defaults.
   */
  void set_defaults();

  /////
  //
  // User Control
  //
  /////

  /**
   * Sets the chassis to controller joysticks using tank control.  Run is usercontrol.
   * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
   */
  void tank();

  /**
   * Sets the chassis to controller joysticks using standard arcade control.  Run is usercontrol.
   * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   */
  void arcade_standard(e_type stick_type);

  /**
   * Sets the chassis to controller joysticks using flipped arcade control.  Run is usercontrol.
   * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   */
  void arcade_flipped(e_type stick_type);

  /**
   * Initializes left and right curves with the SD card, reccomended to run in initialize().
   */
  void init_curve_sd();

  /**
   * Sets the default joystick curves.
   *
   * \param left
   *        Left default curve.
   * \param right
   *        Right default curve.
   */
  void set_curve_default(double left, double right = 0);

  /**
   * Runs a P loop on the drive when the joysticks are released.
   *
   * \param kp
   *        Constant for the p loop.
   */
  void set_active_brake(double kp);

  /**
   * Enables/disables modifying the joystick input curves with the controller.  True enables, false disables.
   *
   * \param input
   *        bool input
   */
  void toggle_modify_curve_with_controller(bool toggle);

  /**
   * Sets buttons for modifying the left joystick curve.
   *
   * \param decrease
   *        a pros button enumerator
   * \param increase
   *        a pros button enumerator
   */
  void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
   * Sets buttons for modifying the right joystick curve.
   *
   * \param decrease
   *        a pros button enumerator
   * \param increase
   *        a pros button enumerator
   */
  void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
   * Outputs a curve from 5225A In the Zone.  This gives more control over the robot at lower speeds.  https://www.desmos.com/calculator/rcfjjg83zx
   *
   * \param x
   *        joystick input
   */
  double left_curve_function(double x);

  /**
   * Outputs a curve from 5225A In the Zone.  This gives more control over the robot at lower speeds.  https://www.desmos.com/calculator/rcfjjg83zx
   *
   * \param x
   *        joystick input
   */
  double right_curve_function(double x);

  /**
   * Sets a new threshold for the joystick.  The joysticks wil not return a value if they are within this.
   *
   * \param threshold
   *        new threshold
   */
  void set_joystick_threshold(int threshold);

  /**
   * Resets drive sensors at the start of opcontrol.
   */
  void reset_drive_sensors_opcontrol();

  /**
   * Sets minimum slew distance constants.
   *
   * \param l_stick
   *        input for left joystick
   * \param r_stick
   *        input for right joystick
   */
  void joy_thresh_opcontrol(int l_stick, int r_stick);

  /////
  //
  // PTO
  //
  /////

  /**
   * Checks if the motor is currently in pto_list.  Returns true if it's already in pto_list.
   *
   * \param check_if_pto
   *        motor to check.
   */
  bool pto_active_check(pros::Motor check_if_pto);

  /**
   * Adds motors to the pto list, removing them from the drive.
   *
   * \param pto_list
   *        list of motors to remove from the drive.
   */
  void pto_add(std::vector<int> pto_list);

  /**
   * Removes motors from the pto list, adding them to the drive.  You cannot use the first index in a pto.
   *
   * \param pto_list
   *        list of motors to add to the drive.
   */
  void pto_remove(std::vector<int> pto_list);

  /**
   * Adds/removes motors from drive.  You cannot use the first index in a pto.
   *
   * \param pto_list
   *        list of motors to add/remove from the drive.
   * \param toggle
   *        if true, adds to list.  if false, removes from list.
   */
  void pto_toggle(bool toggle);


  /**
   * @brief initialize pto 
   * 
   * @param pto_list 
   * @return Drive 
   */
  void with_pto(std::initializer_list<int> pto_list);


  /**
   * @brief Get the pto motors object
   * 
   * @return std::vector<std::reference_wrapper<pros::Motor>> 
   */
  std::vector<pros::Motor> get_pto_motors();
  /////
  //
  // PROS Wrapers
  //
  /////

  /**
   * Sets the chassis to voltage
   *
   * \param left
   *        voltage for left side, -127 to 127
   * \param right
   *        voltage for right side, -127 to 127
   */
  void set_tank(int left, int right);

  /**
   * Changes the way the drive behaves when it is not under active user control
   *
   * \param brake_type
   *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
   */
  void set_drive_brake(pros::motor_brake_mode_e_t brake_type);

  /**
   * Sets the limit for the current on the drive.
   *
   * \param mA
   *        input in milliamps
   */
  void set_drive_current_limit(int mA);

  /**
   * Toggles set drive in autonomous. True enables, false disables.
   */
  void toggle_auto_drive(bool toggle);

  /**
   * Toggles printing in autonomous. True enables, false disables.
   */
  void toggle_auto_print(bool toggle);

  /////
  //
  // Telemetry
  //
  /////

  /**
   * The position of the right motor.
   */
  double right_sensor();

  /**
   * The velocity of the right motor.
   */
  int right_velocity();

  /**
   * The watts of the right motor.
   */
  double right_mA();

  /**
   * Return TRUE when the motor is over current.
   */
  bool right_over_current();

  /**
   * The position of the left motor.
   */
  double left_sensor();

  /**
   * The velocity of the left motor.
   */
  int left_velocity();

  /**
   * The watts of the left motor.
   */
  double left_mA();

  /**
   * Return TRUE when the motor is over current.
   */
  bool left_over_current();

  /**
   * Reset all the chassis motors, reccomended to run at the start of your autonomous routine.
   */
  void reset_drive_sensor();

  /**
   * Resets the current gyro value.  Defaults to 0, reccomended to run at the start of your autonomous routine.
   *
   * \param new_heading
   *        New heading value.
   */
  void reset_gyro(double new_heading = 0);

  /**
   * Resets the imu so that where the drive is pointing is zero in set_drive_pid(turn)
   */
  double get_gyro();

  /**
   * Calibrates the IMU, reccomended to run in initialize().
   *
   * \param run_loading_animation
   *        bool for running loading animation
   */
  bool imu_calibrate(bool run_loading_animation = true);

  /**
   * Loading display while the IMU calibrates.
   */
  void imu_loading_display(int iter);

  /////
  //
  // Autonomous Functions
  //
  /////

  /**
   * Sets the robot to move forward using PID.
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from slew_min to speed over slew_distance.  only use when you're going over about 14"
   * \param toggle_heading
   *        toggle for heading correction
   */
  void set_drive_pid(double target, int speed, bool slew_on = false, bool toggle_heading = true);

  /**
   * @brief Set the drive pid with incline check object,if the incline is greater than the set value, 
   *        mileage will not be recorded
   * @param target target value in inches
   * @param speed  0 to 127, max speed during motion
   * @param slew_on  ramp up from slew_min to speed over slew_distance.  only use when you're going over about 14"
   * @param toggle_heading toggle for heading correction
   * @param deg incline degree ,based on the horizontal plane
   * @param imu_initial_heading imu initial heading, 0 or 90 or 180 or 270
   */
  void set_drive_pid_with_incline_check(double target, int speed, bool slew_on = false, bool toggle_heading = true,float deg=10.f,int imu_initial_heading=0);

  /**
   * @brief Set the arc drive pid object
   * 
   * @param target target value in inches
   * @param left_speed left motors speed
   * @param right_speed  right motors speed
   * @param slew_on ramp up from slew_min to speed over slew_distance.  only use when you're going over about 14"
   */
  void set_arc_drive_pid(double target,int left_speed,int right_speed,bool slew_on = false);
  /**
   * Sets the robot to turn using PID.
   *
   * \param target
   *        target value in degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void set_turn_pid(double target, int speed);


  /**
   * @brief sets the robot to turn using PID with differential speed
   * \param target      target value in degrees
   *  \param left_speed   left motors speed
   *  \param right_speed  right motors speed
   */
   void set_arc_turn_pid(double target, int left_speed,int right_speed);
  /**
   * Turn using only the left or right side.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value in degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void set_swing_pid(e_swing type, double target, int speed);

  /**
   * Resets all PID targets to 0.
   */
  void reset_pid_targets();

  /**
   * Resets all PID targets to 0.
   */
  void set_angle(double angle);

  /**
   * Lock the code in a while loop until the robot has settled.
   */
  void wait_drive();

  /**
   * Lock the code in a while loop until this position has passed.
   *
   * \param target
   *        when driving, this is inches.  when turning, this is degrees.
   */
  void wait_until(double target);

  /**
   * Autonomous interference detection.  Returns true when interfered, and false when nothing happened.
   */
  bool interfered = false;

  /**
   * Changes max speed during a drive motion.
   *
   * \param speed
   *        new clipped speed
   */
  void set_max_speed(int speed);

  /**
   * Set Either the headingPID, turnPID, forwardPID, backwardPID, activeBrakePID, or swingPID
   */
  void set_pid_constants(PID *pid, double p, double i, double d, double p_start_i);

  /**
   * Sets minimum power for swings when kI and startI are enabled.
   *
   * \param min
   *        new clipped speed
   */
  void set_swing_min(int min);

  /**
   * The minimum power for turns when kI and startI are enabled.
   *
   * \param min
   *        new clipped speed
   */
  void set_turn_min(int min);

  /**
   * Returns minimum power for swings when kI and startI are enabled.
   */
  int get_swing_min();

  /**
   * Returns minimum power for turns when kI and startI are enabled.
   */
  int get_turn_min();

  /**
   * Sets minimum slew speed constants.
   *
   * \param fwd
   *        minimum power for forward drive pd
   * \param rev
   *        minimum power for backwards drive pd
   */
  void set_slew_min_power(int fwd, int rev);

  /**
   * Sets minimum slew distance constants.
   *
   * \param fw
   *        minimum distance for forward drive pd
   * \param bw
   *        minimum distance for backwards drive pd
   */
  void set_slew_distance(int fwd, int rev);

  /**
   * Set's constants for exit conditions.
   *
   * \param &type
   *        turn_exit, swing_exit, or drive_exit
   * \param p_small_exit_time
   *        Sets small_exit_time.  Timer for to exit within smalL_error.
   * \param p_small_error
   *        Sets smalL_error. Timer will start when error is within this.
   * \param p_big_exit_time
   *        Sets big_exit_time.  Timer for to exit within big_error.
   * \param p_big_error
   *        Sets big_error. Timer will start when error is within this.
   * \param p_velocity_exit_time
   *        Sets velocity_exit_time.  Timer will start when velocity is 0.
   */
  void set_exit_condition(int type, int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout);

  /**
   * Exit condition for turning.
   */
  const int turn_exit = 1;

  /**
   * Exit condition for swinging.
   */
  const int swing_exit = 2;

  /**
   * Exit condition for driving.
   */
  const int drive_exit = 3;

  /**
   * Returns current tick_per_inch()
   */
  double get_tick_per_inch();

  /**
   * Returns current tick_per_inch()
   */
  void modify_curve_with_controller();

  // Slew
  struct slew_ {
    int sign = 0;
    double error = 0;
    double x_intercept = 0;
    double y_intercept = 0;
    double slope = 0;
    double output = 0;
    bool enabled = false;
    double max_speed = 0;
  };

  slew_ left_slew;
  slew_ right_slew;

  /**
   * Initialize slew.
   *
   * \param input
   *        slew_ enum
   * \param slew_on
   *        is slew on?
   * \param max_speed
   *        target speed during the slew
   * \param target
   *        target sensor value
   * \param current
   *        current sensor value
   * \param start
   *        starting position
   * \param backwards
   *        slew direction for constants
   */
  void slew_initialize(slew_ &input, bool slew_on, double max_speed, double target, double current, double start, bool backwards);

  /**
   * Calculate slew.
   *
   * \param input
   *        slew_ enum
   * \param current
   *        current sensor value
   */
  double slew_calculate(slew_ &input, double current);

  void set_turn_pid_gyro_free(double target, int speed);

  /**
   * Creates a Drive Controller using internal encoders.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your drive wheels.  Remember 4" is 4.125"!
   * \param ticks
   *        Motor cartridge RPM
   * \param ratio
   *        External gear ratio, wheel gear / motor gear.
   * \param wheel_distance
   *        left and right wheel distance
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio,double wheel_distance);
  /**
   * odometry variables and functions
   */

  /**
   * initialize odometry variables
   */
  Drive &with_odom(const float &ForwardTracker_center_distance,const float &SidewaysTracker_center_distance);

  void drive_to_point(double x, double y,int speed,bool ibackwards=false ,bool slew_on = false, bool toggle_heading = true);

  void trun_to_point(double x, double y,int speed);

  /**
   * 设置最小行驶功率
   * \param left_motor 左侧
   * \param right_motor 右侧
  */
  void set_tank_min_power(int left_motor,int right_motor);
 private:  // !Auton

  bool drive_toggle = true;
  bool print_toggle = true;
  int swing_min = 0;
  int turn_min = 0;

  /**
   * Heading bool.
   */
  bool heading_on = true;

  /**
   * Active brake kp constant.
   */
  double active_brake_kp = 0;

  /**
   * Tick per inch calculation.
   */
  double TICK_PER_REV;
  double TICK_PER_INCH;
  double CIRCUMFERENCE;

  double CARTRIDGE;
  double RATIO;
  double WHEEL_DIAMETER;

  /**
   * Max speed for autonomous.
   */
  int max_speed;
  int l_max_spd;
  int r_max_spd;

  /**
   * Tasks
   */
  void drive_pid_task();
  void swing_pid_task();
  void turn_pid_task();
  void ez_auto_task();
  void turn_pid_gyro_free_task();
  void arc_turn_pid_task();
  /**
   * Constants for slew
   */
  double SLEW_DISTANCE[2];
  double SLEW_MIN_POWER[2];

  /**
   * Starting value for left/right
   */
  double l_start = 0;
  double r_start = 0;

  /**
   * Enable/disable modifying controller curve with controller.
   */
  bool disable_controller = true;  // True enables, false disables.

  /**
   * Is tank drive running?
   */
  bool is_tank;

#define DRIVE_INTEGRATED 1
#define DRIVE_ADI_ENCODER 2
#define DRIVE_ROTATION 3

  /**
   * Is tracking?
   */
  int is_tracker = DRIVE_INTEGRATED;

  /**
   * Save input to sd card
   */
  void save_l_curve_sd();
  void save_r_curve_sd();

  /**
   * Struct for buttons for increasing/decreasing curve with controller
   */
  struct button_ {
    bool lock = false;
    bool release_reset = false;
    int release_timer = 0;
    int hold_timer = 0;
    int increase_timer;
    pros::controller_digital_e_t button;
  };

  button_ l_increase_;
  button_ l_decrease_;
  button_ r_increase_;
  button_ r_decrease_;

  /**
   * Function for button presses.
   */
  void button_press(button_ *input_name, int button, std::function<void()> changeCurve, std::function<void()> save);

  /**
   * The left and right curve scalers.
   */
  double left_curve_scale;
  double right_curve_scale;

  /**
   * Increase and decrease left and right curve scale.
   */
  void l_decrease();
  void l_increase();
  void r_decrease();
  void r_increase();

  /**
   * 左右轮的轮间距
   */
  double WHEEL_DISTANCE;

  int l_tank_min_power=0;
  int r_tank_min_power=0;



  /**
   * 是否记录pid数据
  */
  bool pid_logger=false;

  /**
   * 用于记录pid数据到文件
   */
  PIDLogger logger;
  /**
   * 暂存左右电机编码值或陀螺仪角度，用于pid数据记录
  */
  std::vector<double> l_sensor_vec;
  std::vector<double> r_sensor_vec;
  std::vector<double> gyro_vec;
  inline void clear_vec(){
    l_sensor_vec.clear();
    r_sensor_vec.clear();
    gyro_vec.clear();
  }


  bool incline_check;
  float incline_deg_threshold;
  int imu_initial_heading;
  std::deque<float> incline_deg_vec;

  int left_condition_index=0;
  int right_condition_index=0;
public:
  /**
   * 用于记录pid数据
   */
  void set_pid_logger(bool logger);

};
