#include "gps/gps_drive.hpp"
#include <map>
#include <vector>

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "fmt/core.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "EZ-Template/units.h"
#include <fstream>
#define FMT_HEADER_ONLY

//********************************************

#define GPS_RATE 10
#define CHASE_POWER 2
using namespace ez::util;

constexpr float gps_cycle=GPS_RATE/1000.0;


template<typename T, typename... Args>
inline void write_to_csv(const std::string& filename, const T first,const Args... args) {
    std::ofstream file;
    file.open(filename, std::ios::app); // Open file in append mode
    file << first;
    ((file << "," << args), ...);
    file << "\n";
    file.close();
}

inline float inch_to_meter(float inch) {
  return inch * 0.0254;
}


Gps_Drive::Gps_Drive(Drive &drive_chassis, const std::uint8_t gps_port) : gps_sensor(gps_port), drive_chassis(drive_chassis),
gps_task([this]() { this->gps_task_fn(); }) {
  // 设置GPS传感器的陀螺仪数据更新频率
  chassis_config = drive_chassis.get_config();
  tick_per_inch=drive_chassis.get_tick_per_inch();
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
  constexpr double dt = gps_cycle; // 测量频率

  Eigen::MatrixXd F(n, n);  // 状态转移矩阵
  Eigen::MatrixXd H(4, 6);  // 观测矩阵
  Eigen::MatrixXd Q(n, n);  // 过程噪声协方差
  Eigen::MatrixXd R(4, 4);  // 测量噪声协方差
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
      0, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0;

  // 估计过程噪声协方差
  Q << pow(dt, 4) / 4.f, pow(dt, 3) / 2.f, pow(dt, 2) / 2.f, 0, 0, 0,
      pow(dt, 3) / 2.f, pow(dt, 2), dt, 0, 0, 0,
      pow(dt, 2) / 2.f, dt, 1, 0, 0, 0,
      0, 0, 0, pow(dt, 4) / 4.f, pow(dt, 3) / 2.f, pow(dt, 2) / 2.f,
      0, 0, 0, pow(dt, 3) / 2.f, pow(dt, 2), dt,
      0, 0, 0, pow(dt, 2) / 2.f, dt, 1;
  Q = Q * 0.1;  // 乘上加速度方差
  // 测量噪声协方差
  R << 0.0001, 0,0,0,
      0, 0.0001,0,0,
      0,0,0.005,0,
      0,0,0,0.005;
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
  float prev_heading = 0;
  double prev_left_dist=0;
  double prev_right_dist=0;
  const char* gps_data_path = "/usd/gps_data.csv";
  auto scalar_time=pros::millis();
  pros::delay(5000);
  while (true) {
    auto status_raw = gps_sensor.get_status();
    float heading = to_rad(gps_sensor.get_heading());//角度都为弧度制
    double gps_error = gps_sensor.get_error();
    // double left_traveled_dist=get_traveled_dist(drive_chassis.left_sensor());
    // double right_traveled_dist=get_traveled_dist(drive_chassis.right_sensor());
    float delta_heading=heading-prev_heading;
    float avg_heading=heading-delta_heading/2;

    // //以下距离单位为inch
    // float delta_left_dist=left_traveled_dist-prev_left_dist;
    // float delta_right_dist=right_traveled_dist-prev_right_dist;
    // float delta_dist=(delta_left_dist+delta_right_dist)/2;

    //:w=v/r , v=w*r 
    double l_spd=inch_to_meter(((drive_chassis.left_velocity()/60.0)/chassis_config.ratio)*2*M_PI*(chassis_config.wheel_diameter/2));
    double r_spd=inch_to_meter(((drive_chassis.right_velocity()/60.0)/chassis_config.ratio)*2*M_PI*(chassis_config.wheel_diameter/2));
    
    //以下速度单位为m/s^2
  
    // float pose_spd=inch_to_meter(delta_dist)/gps_cycle;
    float pose_spd=(l_spd+r_spd)/2;

    pros::screen::print(pros::E_TEXT_MEDIUM,3,"spd: %5.2f",pose_spd);
//rpm:79   spd:0.21
    if(!(fabs(delta_heading)<1e-5)){
      pose_spd=2*sin(delta_heading/2)*(pose_spd/delta_heading);
    }

    Pose odom_spd;
    odom_spd.x=pose_spd*sin(avg_heading); 
    odom_spd.y=pose_spd*cos(avg_heading);
    odom_spd.theta=heading;
    

    // prev_left_dist=left_traveled_dist;
    // prev_right_dist=right_traveled_dist;
    prev_heading = heading;


    Eigen::VectorXd y(4);
    Eigen::MatrixXd r(4,4);

    y << status_raw.x, status_raw.y, odom_spd.x, odom_spd.y; 
    r << pow(gps_error, 2), 0, 0, 0,
        0, pow(gps_error, 2), 0, 0,
        0, 0, 0.005, 0,
        0, 0, 0, 0.005; 
    kf.update(y);

    set_position(Pose{static_cast<float>(kf.state()(0)), static_cast<float>(kf.state()(3)), static_cast<float>(heading)});

    pros::screen::print(pros::E_TEXT_MEDIUM,0,"kf  x:%5.2f,y:%5.2f,theta:%6.2f",get_position().x,get_position().y,to_deg(heading));
    pros::screen::print(pros::E_TEXT_MEDIUM,1,"raw x:%5.2f,y:%5.2f",status_raw.x,status_raw.y);
    pros::screen::print(pros::E_TEXT_MEDIUM,2,"spd x:%5.2f,y:%5.2f,theta:%6.2f",odom_spd.x,odom_spd.y,to_deg(odom_spd.theta));
    pros::screen::print(pros::E_TEXT_MEDIUM,4,"RPM: l: %5.2f,r: %5.2f",drive_chassis.left_velocity(),drive_chassis.right_velocity());
    pros::screen::print(pros::E_TEXT_MEDIUM,5,"gps err: %6lf",gps_sensor.get_error());

    write_to_csv(gps_data_path,odom_spd.x,kf.state()(1),odom_spd.y,kf.state()(4),kf.state()(0),kf.state()(3),status_raw.x,status_raw.y,to_deg(heading));

    pros::Task::delay_until(&scalar_time,GPS_RATE);
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
    
    auto comp_state=pros::competition::get_status();
    while (comp_state== pros::competition::get_status()) {
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

double Gps_Drive::get_traveled_dist(double tick) {
  return tick / tick_per_inch;
}



