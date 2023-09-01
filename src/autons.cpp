#include "autons.hpp"
#include "main.h"
#include "pros/gps.hpp"
#include "pros/rtos.hpp"





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
  /**
  * 设置PID的退出条件，函数的声明会在后文给出。
  * 该函数在底盘Drive类构造时已使用默认参数自动调用，在这里再次用的原因是使用默认参数时，无法反映车子在停止时是否抖动，需要让PID的退出条件更加严格，以保证PID参数的精确。
  */
  chassis.set_exit_condition(chassis.turn_exit, 800, 3, 800, 7, 5000, 5000);
  chassis.set_exit_condition(chassis.swing_exit, 800, 3, 800, 7, 5000, 5000);
  chassis.set_exit_condition(chassis.drive_exit, 800, 50, 800, 150, 5000, 5000);


  chassis.set_drive_pid(45, 120,true);
  chassis.wait_drive();
  chassis.set_turn_pid(90, 60);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(0,80);
  chassis.wait_drive();
  chassis.set_swing_pid(LEFT_SWING, 120, 100);
  chassis.wait_drive();
  chassis.set_swing_pid(LEFT_SWING,0, 60);
  chassis.wait_drive();
  chassis.set_drive_pid(-45,100,false);
  chassis.wait_drive();
  chassis.set_drive_pid(45,120,true);
  chassis.wait_drive();
  // //开始的PID参数
  // float start_p=0.5,start_i=0,start_d=0.5;
  // //结束的PID参数
  // constexpr float stop_p=4,stop_i=0,stop_d=10;
  // //PID参数的步长
  // constexpr float step_p=0.5,step_i=0.1,step_d=1;
  // //打开PID数据日志
  // // chassis.set_pid_logger(true);

  // while(start_p<=stop_p){
  //   while(start_d<=stop_d){

  //       while(start_i<=stop_i){
  //           //设置PID参数
  //           chassis.set_pid_constants(&chassis.forward_drivePID, start_p, start_i, start_d, 0);
  //           chassis.set_pid_constants(&chassis.backward_drivePID, start_p, start_i, start_d, 0);
  //           //让车子跑一下
  //           chassis.set_drive_pid(30,110,false,true);
  //           chassis.wait_drive();
  //           chassis.set_drive_pid(-30,110,false,true);
  //           chassis.wait_drive(); 
  //           start_i+=step_i;
  //       }
  //       start_i=0;
  //       start_d+=step_d;
  //   }
  //   start_d=0;
  //   start_p+=step_p;
  // }
  // printf("test finished\n");
}