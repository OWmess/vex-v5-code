
#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/chassis/kalman_filter.hpp"
#include "pros/gps.hpp"
namespace lemlib {
/**
 * @brief Set the sensors to be used for odometry
 *
 * @param sensors the sensors to be used
 * @param drivetrain drivetrain to be used
 */
void setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain);
/**
 * @brief Get the pose of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return Pose
 */
Pose getPose(bool radians = false);
/**
 * @brief Set the Pose of the robot
 *
 * @param pose the new pose
 * @param radians true if theta is in radians, false if in degrees. False by default
 */
void setPose(Pose pose, bool radians = false);
/**
 * @brief Get the speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return lemlib::Pose
 */
Pose getSpeed(bool radians = false);
/**
 * @brief Get the local speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return lemlib::Pose
 */
Pose getLocalSpeed(bool radians = false);
/**
 * @brief Estimate the pose of the robot after a certain amount of time
 *
 * @param time time in seconds
 * @param radians False for degrees, true for radians. False by default
 * @return lemlib::Pose
 */
Pose estimatePose(float time, bool radians = false);
/**
 * @brief Update the pose of the robot
 *
 */
void update();
/**
 * @brief Initialize the odometry system
 *
 */
void init();

// 初始化卡尔曼滤波器
constexpr double dt = 0.01; // 测量周期
inline Eigen::MatrixXd F(6, 6);  // 状态转移矩阵
inline Eigen::MatrixXd H(4, 6);  // 观测矩阵
inline Eigen::MatrixXd Q(6, 6);  // 过程噪声协方差
inline Eigen::MatrixXd R(4, 4);  // 测量噪声协方差
inline Eigen::MatrixXd P(6, 6);  // 估计误差协方差
inline KalmanFilter kalmanFilter;

inline pros::GPS *gps;

/**
 * @brief Initialize the Kalman Filter
 *
 */
void kalmanFilterInit();

Pose getKFPose(bool radians);
} // namespace lemlib
