#include "main.h"
#include "kalman.hpp"
/**
 * 设置正常运行时的底盘速度
*/
#define DRIVE_SPEED  100
#define TURN_SPEED   80
#define SWING_SPEED  80

/**
 * @brief      设置底盘控制相关常数，如PID参数等
 *              该函数在程序初始化时调用，PID参数调整遵循Ziegler-Nichols方法
*/
void default_constants() {
  chassis.set_slew_min_power(50, 50);//设置最小启动速度，用于缓加速
  chassis.set_slew_distance(7, 7);//设置缓加速的距离
  ///设置PID参数，第一个参数为PID结构体，后面四个参数分别为P、I、D、最大输出
  chassis.set_pid_constants(&chassis.headingPID,4, 0.000, 13, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 1, 0, 2, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.5, 0, 4, 0);
  chassis.set_pid_constants(&chassis.turnPID, 4, 0.01,25, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
  chassis.set_pid_constants(&chassis.turnPID_gyro_free, 0.45, 0, 2, 0);

}



/**
* 防守方自动程序
*/
void guard(){
  ///init catapult
  control.set_catapult_state(DOWN);
  
  ///****
  chassis.set_turn_pid(40,40);
  control.set_intake_state(INTAKE);
  chassis.wait_drive();
  chassis.set_drive_pid(12,25,true);
  chassis.wait_drive();
  pros::delay(800);
  control.set_catapult_state(MIDDLE);
  pros::delay(200);

  chassis.set_drive_pid(-20,DRIVE_SPEED,true);
  chassis.wait_drive();
  chassis.set_turn_pid(-3,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10,DRIVE_SPEED,true);
  chassis.wait_drive();
  pros::delay(300);
  chassis.set_turn_pid(10,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-50,120,true);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(20,DRIVE_SPEED,true);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  pros::delay(50);

  chassis.set_drive_pid(-40,DRIVE_SPEED,true);
  chassis.wait_drive();
  chassis.set_swing_pid(RIGHT_SWING,-12,SWING_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(50,80,true);
  chassis.wait_drive();
}

/**
* 进攻方自动程序
*/
void attack(){
  control.set_catapult_state(BRAKE);
  control.set_intake_state(INTAKE);
  chassis.set_drive_pid(39,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(15,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  pros::delay(100);
  chassis.set_drive_pid(-15,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(INTAKE);
  chassis.set_swing_pid(LEFT_SWING,0,SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(15,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(20,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);

  chassis.set_drive_pid(-20,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(INTAKE);
  chassis.set_turn_pid(-90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(12,DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(100);
  chassis.set_drive_pid(-12,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(23,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  pros::delay(100);
  chassis.set_drive_pid(-10,DRIVE_SPEED);
  control.set_intake_state(INTAKE);
  chassis.wait_drive();
  chassis.set_turn_pid(230,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(27,DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(100);
  chassis.set_drive_pid(-30,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10,DRIVE_SPEED);
  chassis.wait_drive();

  control.set_intake_state(OUTTAKE);
  pros::delay(100);
  chassis.set_drive_pid(-20,DRIVE_SPEED);
  chassis.wait_drive();


}

void conservatively_attack(){
  control.set_catapult_state(BRAKE);
  control.set_intake_state(INTAKE);
  chassis.set_drive_pid(39,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(15,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  pros::delay(100);
  chassis.set_drive_pid(-20,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(180,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(26,DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(100);
  chassis.set_turn_pid(250,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(30,60);
  chassis.wait_drive();
}

void test_pid(){
  // float P =0.5,I=0,D=2;
  // chassis.set_pid_logger(true);
  // for(int i=0;i<5;++i){
  //   chassis.set_drive_pid(50,DRIVE_SPEED);
  //   chassis.wait_drive();
  //   chassis.set_drive_pid(-50,DRIVE_SPEED);
  //   chassis.wait_drive();

  //   chassis.set_pid_constants(&chassis.forward_drivePID, P, I, D, 0);
  //   chassis.set_pid_constants(&chassis.backward_drivePID, P, I, D, 0);
  //   P*=2;
  //   D*=2;
  // }
    int n = 3; // Number of states
  int m = 1; // Number of measurements

  double dt = 1.0/30; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
  R << 5;
  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt,A, C, Q, R, P);

  // List of noisy position measurements (y)
  std::vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
  };

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  double t = 0;
  x0 << measurements[0], 0, -9.81;
  kf.init(t, x0);

  // Feed measurements into filter, output estimated states

  Eigen::VectorXd y(m);
  std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
  for(int i = 0; i < measurements.size(); i++) {
    t += dt;
    y << measurements[i];
    kf.update(y);
    std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
        << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
  }
}