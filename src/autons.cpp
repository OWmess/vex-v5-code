#include "autons.hpp"
#include <deque>
#include "EZ-Template/util.hpp"
#include "control.hpp"
#include "main.h"
#include "pros/rtos.hpp"

/**
 * 设置正常运行时的底盘速度
*/
#define DRIVE_SPEED  120
#define TURN_SPEED   100
#define SWING_SPEED  80

/**
 * @brief      设置底盘控制相关常数，如PID参数等
 *              该函数在程序初始化时调用
*/
void default_constants() {
  chassis.set_slew_min_power(40, 40);//设置最小启动速度，用于缓加速
  chassis.set_slew_distance(7, 7);//设置缓加速的距离
  ///设置PID参数，第一个参数为PID结构体，后面四个参数分别为P、I、D、积分初始值
  chassis.set_pid_constants(&chassis.headingPID,4, 0.000, 10, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 1, 0, 2, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.5, 0, 4, 0);
  chassis.set_pid_constants(&chassis.turnPID, 4, 0.05,25, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
  chassis.set_pid_constants(&chassis.turnPID_gyro_free, 0.45, 0, 2, 0);

}



/**
* 防守方自动程序
*/
void guard() {
  ///init catapult
  control.set_armer_state(ON);
  pros::delay(500);

  chassis.set_drive_pid(-12,40,true);
  chassis.wait_drive();

  chassis.set_turn_pid(95,35);
  chassis.wait_drive();
  
  control.set_armer_state(OFF);
  chassis.set_drive_pid(-30, DRIVE_SPEED,true);//-53
  chassis.wait_drive();
  // control.set_armer_state(ON);

  chassis.set_turn_pid(65,35);
  chassis.wait_drive();
  chassis.set_drive_pid(-22, DRIVE_SPEED,true);//-53
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_turn_pid(130, TURN_SPEED);
  chassis.wait_drive();
  // control.set_armer_state(OFF);

  chassis.set_drive_pid(28,DRIVE_SPEED,true);
  chassis.wait_drive();

  pros::delay(200);
  chassis.set_drive_pid(-10, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40,DRIVE_SPEED,true);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, DRIVE_SPEED);
  chassis.wait_drive();

  control.set_intake_state(OUTTAKE);

  chassis.set_drive_pid(28,DRIVE_SPEED,true);
  chassis.wait_drive();
}


void guard_1() {
  chassis.set_drive_pid(18, DRIVE_SPEED,true);
  chassis.wait_drive();
  control.set_wings_state(ON);
  pros::delay(500);
  control.set_intake_state(OUTTAKE);

  chassis.set_swing_pid(ez::LEFT_SWING,45,80);
  chassis.wait_drive();
  pros::delay(300);
  control.set_intake_state(STOP);
  control.catapult_motors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.set_drive_pid(12, DRIVE_SPEED,true);
  chassis.wait_drive();
  pros::delay(200);

  chassis.set_drive_pid(-5, DRIVE_SPEED,true);
  chassis.wait_drive();
  pros::delay(200);

  chassis.set_drive_pid(10, DRIVE_SPEED,true);
  chassis.wait_drive();
  control.set_wings_state(ON);
  chassis.set_drive_pid(-5, DRIVE_SPEED,true);
  chassis.wait_drive();
  
  pros::delay(300);
  control.set_wings_state(OFF);

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(45,50,true);
  chassis.wait_drive();

  

  // chassis.set_turn_pid(0, TURN_SPEED);
  // chassis.wait_drive();
  // chassis.set_drive_pid(24, DRIVE_SPEED);
  // chassis.wait_drive();
  // chassis.set_turn_pid(-45, TURN_SPEED);
  // chassis.wait_drive();
  // control.set_intake_state(OUTTAKE);
  // chassis.set_drive_pid(32,80,true);
  // chassis.wait_drive();

}


void attack() {
  constexpr static int turn_speed=120;
  control.set_intake_state(INTAKE);
  chassis.set_drive_pid(20,DRIVE_SPEED);//33
  chassis.wait_drive();

  // chassis.set_turn_pid(90,turn_speed);
  chassis.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);

  chassis.set_drive_pid(10,125);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-8,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(INTAKE);
  chassis.set_swing_pid(LEFT_SWING,0,SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,turn_speed);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  chassis.set_drive_pid(20,120);
  chassis.wait_drive();

  chassis.set_drive_pid(-15,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(INTAKE);
  chassis.set_turn_pid(-85,turn_speed);
  chassis.wait_drive();
  chassis.set_drive_pid(10,DRIVE_SPEED);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-12,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,turn_speed);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  chassis.set_drive_pid(18,120);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-10,DRIVE_SPEED);
  control.set_intake_state(INTAKE);
  chassis.wait_drive();
  chassis.set_turn_pid(235,turn_speed);
  chassis.wait_drive();
  chassis.set_drive_pid(19,DRIVE_SPEED);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-15,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(70,turn_speed);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  chassis.set_drive_pid(20,120);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-18,DRIVE_SPEED);
  chassis.wait_drive();


}


void guard_aggressive(){
  constexpr static int turn_speed=120;
  constexpr static int swing_speed=120;
  control.set_intake_state(INTAKE);
  chassis.set_drive_pid(15,DRIVE_SPEED);//33  15
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, 15, swing_speed);
  chassis.wait_drive();
  // chassis.set_drive_pid(10,DRIVE_SPEED);//33
  // chassis.wait_drive();
  // chassis.set_swing_pid(ez::RIGHT_SWING, 0, swing_speed);
  // chassis.wait_drive();
  chassis.set_drive_pid(23,DRIVE_SPEED);//16  23
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid(-5,DRIVE_SPEED);//33
  chassis.wait_drive();

  chassis.set_turn_pid(-90,TURN_SPEED);//33
  chassis.wait_drive();

  pros::delay(200);
  chassis.set_turn_pid(-180,TURN_SPEED);//33
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  pros::delay(6500);
  control.set_intake_state(INTAKE);
  chassis.set_turn_pid(-158,TURN_SPEED);//33
  chassis.wait_drive();
  chassis.set_drive_pid(40,DRIVE_SPEED);//33
  chassis.wait_drive();

  chassis.set_turn_pid(-60,TURN_SPEED);//33
  chassis.wait_drive();

  chassis.set_drive_pid(20,DRIVE_SPEED);//33
  chassis.wait_drive();

  chassis.set_turn_pid(0,TURN_SPEED);//33
  chassis.wait_drive();

  chassis.set_drive_pid(15,DRIVE_SPEED);//33
  chassis.wait_drive();

  chassis.set_drive_pid(-15,DRIVE_SPEED);//33
  chassis.wait_drive();
}




/**
* 进攻方自动程序
*/
void attack_aggressive() {
  constexpr static int turn_speed=120;
  control.set_wings_state(ON);
  //take ball
  chassis.set_drive_pid(42,DRIVE_SPEED);//33
  chassis.wait_until(10);
  control.set_wings_state(OFF);
  control.set_intake_state(INTAKE);
  chassis.wait_drive();
  //turn to goal
  chassis.set_turn_pid(102,turn_speed);
  chassis.wait_drive();
  control.set_wings_state(ON);
  //out
  control.set_intake_state(OUTTAKE);
  //score
  chassis.set_drive_pid(20,120);
  chassis.wait_drive();
  control.set_wings_state(OFF);
  chassis.set_drive_pid(-20,DRIVE_SPEED);
  chassis.wait_drive();

  //second ball
  control.set_intake_state(INTAKE);
  chassis.set_turn_pid(-85,turn_speed);
  chassis.wait_drive();
  chassis.set_drive_pid(13,DRIVE_SPEED);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-12,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(99,turn_speed);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  chassis.set_drive_pid(30,120);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-10,DRIVE_SPEED);
  control.set_intake_state(INTAKE);
  chassis.wait_drive();
  chassis.set_turn_pid(240,turn_speed);
  chassis.wait_drive();
  chassis.set_drive_pid(28,DRIVE_SPEED);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-15,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(78,turn_speed);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  chassis.set_drive_pid(28,120);
  chassis.wait_drive();
  // pros::delay(200);
  chassis.set_drive_pid(-20,DRIVE_SPEED);
  chassis.wait_drive();

}

void skill_match(){
  //   chassis.set_pid_constants(&chassis.headingPID,4, 0.000, 13, 0);
  // chassis.set_pid_constants(&chassis.forward_drivePID, 1, 0.001, 2, 150);
  // chassis.set_pid_constants(&chassis.backward_drivePID, 0.5, 0.001, 4, 150);
  // chassis.set_pid_constants(&chassis.turnPID, 4, 0.05,25, 15);
  // chassis.set_pid_constants(&chassis.swingPID, 7, 0.05, 45, 10);
  chassis.set_exit_condition(chassis.drive_exit,30,50, 30, 100, 2000, 100);
  chassis.set_exit_condition(chassis.turn_exit, 30, 3, 30, 7, 500, 100);
  chassis.set_exit_condition(chassis.swing_exit, 30, 3, 30, 7, 500, 100);
  
  constexpr static int turn_speed=100;
  constexpr static int drive_speed=120;
  constexpr static int swing_speed=120;
  chassis.set_arc_turn_pid(90,120,30);
  // chassis.set_swing_pid(ez::LEFT_SWING,90,-120);
  control.set_intake_state(OUTTAKE);
  chassis.wait_drive();

  chassis.set_drive_pid(15,drive_speed);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid(-15,drive_speed);
  chassis.wait_drive();
  
  chassis.set_turn_pid(160,turn_speed);
  chassis.wait_drive();
  chassis.set_drive_pid(-5,35);
  chassis.wait_drive();

  auto start_t=pros::millis();
  // chassis.set_mode(ez::DISABLE);
  // cata_motor_reference.move(120);
  control.set_catapult_state(LAUNCH);
  while(pros::millis()-start_t<2000){//30000ms=30s
    pros::delay(100);
  }
  control.set_catapult_state(RELEASE);
  chassis.set_drive_pid(15,drive_speed);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING,90,120);
  chassis.wait_drive();
  chassis.set_arc_turn_pid(180,120,45);
  chassis.wait_drive();
  chassis.set_drive_pid_with_incline_check(60,120,true,true,10,90);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid_with_incline_check(-70,120,true,true,10,90);

  chassis.wait_drive();
  pros::delay(200);
  // chassis.set_drive_pid(10,drive_speed);
  // chassis.wait_drive();
  chassis.set_turn_pid(90,turn_speed);
  chassis.wait_drive();
  
  chassis.set_drive_pid(-10,drive_speed);
  chassis.wait_drive();

  chassis.set_arc_turn_pid(160,-40,-120);
  chassis.wait_drive();
  chassis.set_drive_pid(-32,40);
  chassis.wait_drive();
  start_t=pros::millis();
  control.set_catapult_state(LAUNCH);
  while(pros::millis()-start_t<2000){//30000ms=30s
    pros::delay(100);
  }
  control.set_catapult_state(RELEASE);

  //第二次
  chassis.set_drive_pid(10,drive_speed);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING,90,120);
  chassis.wait_drive();
  chassis.set_arc_turn_pid(180,120,55);
  chassis.wait_drive();
  chassis.set_drive_pid_with_incline_check(60,120,true,true,10,90);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid_with_incline_check(-20,120,true,true,10,90);
  chassis.wait_drive();



  chassis.set_mode(DISABLE);
  chassis.set_tank(0,0);
  pros::delay(5000);

}

