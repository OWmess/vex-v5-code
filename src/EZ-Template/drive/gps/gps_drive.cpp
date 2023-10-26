#include "EZ-Template/drive/gps/gps_drive.hpp"

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/drive/gps/kalman_filter.hpp"
#include "EZ-Template/drive/gps/pose.hpp"
#include "EZ-Template/util.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "fmt/core.h"
#include "pros/rtos.hpp"

#define FMT_HEADER_ONLY

//********************************************

#define GPS_RATE 5
#define CHASE_POWER 2
using namespace ez::util;
Gps_Drive::Gps_Drive(Drive &drive_chassis, const std::uint8_t gps_port) : gps_sensor(gps_port), drive_chassis(drive_chassis), gps_task([this]() { this->gps_task_fn(); }) {
  // 设置GPS传感器的陀螺仪数据更新频率
  gps_sensor.set_data_rate(GPS_RATE);
  initlize_kf();
}

Gps_Drive::Gps_Drive(Drive &drive_chassis, const std::uint8_t gps_port, double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset) : Gps_Drive(drive_chassis, gps_port) {
  initlize_gps(xInitial, yInitial, headingInitial, xOffset, yOffset);
}

void Gps_Drive::initlize_gps(double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset) {
  gps_sensor.initialize_full(xInitial, yInitial, headingInitial, xOffset, yOffset);
  Eigen::MatrixXd x0(6, 1);
  x0 << xInitial, 0, 0, yInitial, 0, 0;
  kf.init(0, x0);
}

void Gps_Drive::initlize_kf() {
  int n = 6;
  int m = 1;
  // 初始化卡尔曼滤波器
  constexpr double dt = GPS_RATE / 1000.0;  // 测量频率

  Eigen::MatrixXd F(n, n);  // 状态转移矩阵
  Eigen::MatrixXd H(2, 6);  // 观测矩阵
  Eigen::MatrixXd Q(n, n);  // 过程噪声协方差
  Eigen::MatrixXd R(2, 2);  // 测量噪声协方差
  Eigen::MatrixXd P(n, n);  // 估计误差协方差

  /**
   * 状态向量 X(1,6)：[x, x', x'', y, y', y'']
   */
  /**
   *  F:状态转移矩阵
   */
  F << 1, dt, 0.5 * pow(dt, 2), 0, 0, 0,
      0, 1, dt, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, dt, 0.5 * pow(dt, 2),
      0, 0, 0, 0, 1, dt,
      0, 0, 0, 0, 0, 1;
  /**
   * H:观测矩阵
   */
  H << 1, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0;

  // 估计过程噪声协方差
  Q << pow(dt, 4) / 4.f, pow(dt, 3) / 2.f, pow(dt, 2) / 2.f, 0, 0, 0,
      pow(dt, 3) / 2.f, pow(dt, 2), dt, 0, 0, 0,
      pow(dt, 2) / 2.f, dt, 1, 0, 0, 0,
      0, 0, 0, pow(dt, 4) / 4.f, pow(dt, 3) / 2.f, pow(dt, 2) / 2.f,
      0, 0, 0, pow(dt, 3) / 2.f, pow(dt, 2), dt,
      0, 0, 0, pow(dt, 2) / 2.f, dt, 1;
  Q = Q * 0.35;  // 乘上加速度方差
  // 测量噪声协方差
  R << 0.005, 0,
      0, 0.005;
  // 估计误差协方差
  P << .05, .05, .05, 0, 0, 0,
      .05, .05, .05, 0, 0, 0,
      .05, .05, .05, 0, 0, 0,
      0, 0, 0, .05, .05, .05,
      0, 0, 0, .05, .05, .05,
      0, 0, 0, .05, .05, .05;

  kf = KalmanFilter{dt, F, H, Q, R, P};

  kf.init();
}

void Gps_Drive::gps_task_fn() {
  while (true) {
    auto status_raw = gps_sensor.get_status();
    auto heading = gps_sensor.get_heading();
    Eigen::VectorXd y(2);
    y << status_raw.x, status_raw.y;
    kf.update(y);

    set_position(Pose{static_cast<float>(kf.state()(0)), static_cast<float>(kf.state()(3)), static_cast<float>(to_rad(heading))});
    pros::delay(GPS_RATE);
  }
}

void Gps_Drive::move_to(float x, float y, float heading, int max_speed, bool forward, float chasePower, float lead) {
  // 读取底盘PID参数并配置
  auto [kp1, ki1, kd1, si1] = forward ? drive_chassis.forward_drivePID.constants : drive_chassis.backward_drivePID.constants;
  PID linear_pid{kp1, ki1, kd1, si1};
  auto [kp2, ki2, kd2, si2] = drive_chassis.turnPID.constants;
  PID angular_pid{kp2, ki2, kd2, si2};

  // 设置PID退出条件
  linear_pid.set_exit_condition(80, 50, 300, 150, 500, 5000);
  angular_pid.set_exit_condition(100, 3, 500, 7, 500, 5000);



  auto gps_move_fn = [this, &linear_pid, &angular_pid, &forward, &max_speed, &chasePower, &lead,x,y,heading]() {

    float prev_linear_power = 0;
    Pose target_pose{x, y, static_cast<float>(M_PI_2) - to_rad(heading)};

    if (!forward) target_pose.theta = fmod(target_pose.theta + M_PI, 2 * M_PI);
    bool close = false;
    if (chasePower == 0) chasePower = CHASE_POWER;
    while (true) {
      Pose now_pose = get_position();
      if (!forward) now_pose.theta += M_PI;

      now_pose.theta = M_PI_2 - now_pose.theta;  //???

      if (now_pose.distance(target_pose) < 7.5 && !close) {
        close = true;
        max_speed = fmax(fabs(prev_linear_power), 30.0);
      }
      // 计算carrot point
      Pose carrot = close ? target_pose : target_pose - (Pose(cos(target_pose.theta), sin(target_pose.theta)) * lead * now_pose.distance(target_pose));

      float angular_error;
      if(!close)
        angular_error=angleError(now_pose.angle(carrot), now_pose.theta,true);
      else
        angular_error=angleError(target_pose.theta, now_pose.theta,true);

      float linear_error=now_pose.distance(carrot)* cos(angular_error);
      if(!forward) linear_error=-linear_error;

      //计算pid输出
      float linear_power = linear_pid.compute(linear_error, 0);
      float angular_power=-angular_pid.compute(to_deg(angular_error), 0);


      // calculate radius of turn
      float curvature = fabs(getCurvature(now_pose, carrot));
      if (curvature == 0) curvature = -1;
      float radius = 1 / curvature;

      // calculate the maximum speed at which the robot can turn
      // using the formula v = sqrt( u * r * g )
      if (radius != -1) {
          float maxTurnSpeed = sqrt(chasePower * radius * 9.8);
          // the new linear power is the minimum of the linear power and the max turn speed
          if (linear_power > maxTurnSpeed && !close) linear_power = maxTurnSpeed;
          else if (linear_power < -maxTurnSpeed && !close) linear_power = -maxTurnSpeed;
      }

      // prioritize turning over moving
      float overturn = fabs(angular_power) + fabs(linear_power) - max_speed;
      if (overturn > 0) linear_power -= linear_power > 0 ? overturn : -overturn;
      prev_linear_power = linear_power;

      // calculate motor powers
      drive_chassis.set_tank(linear_power + angular_power, linear_power - angular_power);

      pros::delay(10);
    }
  };
  moving_mutex.take();
  pros::Task gps_move_task(gps_move_fn);
  moving_mutex.give();
}

void Gps_Drive::set_position(const Pose &position) {
  position_mutex.take(10);
  this->position = position;
  position_mutex.give();
}

Pose Gps_Drive::get_position() {
  position_mutex.take(10);
  return position;
  position_mutex.give();
}

/**
 * @brief Get the signed curvature of a circle that intersects the first pose and the second pose
 *
 * @note The circle will be tangent to the theta value of the first pose
 * @note The curvature is signed. Positive curvature means the circle is going clockwise, negative means
 * counter-clockwise
 * @note Theta has to be in radians and in standard form. That means 0 is right and increases counter-clockwise
 *
 * @param pose the first pose
 * @param other the second pose
 * @return float curvature
 */
float Gps_Drive::getCurvature(Pose pose, Pose other){
    // calculate whether the pose is on the left or right side of the circle
    float side = sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

void Gps_Drive::wait_drive() {
  moving_mutex.take();
  moving_mutex.give();
}