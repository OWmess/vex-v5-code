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
  chassis.set_exit_condition(chassis.drive_exit,30,50, 30, 100, 500, 500);
  chassis.set_exit_velocity_out(chassis.drive_exit,1);
  chassis.set_exit_condition(chassis.turn_exit, 30, 3, 30, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 30, 3, 30, 7, 500, 500);
  
  constexpr static int turn_speed=100;
  constexpr static int drive_speed=120;
  constexpr static int swing_speed=120;

  chassis.set_arc_turn_pid(90,125,30);
  // chassis.set_swing_pid(ez::LEFT_SWING,90,-120);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  chassis.set_drive_pid(15,125);
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
  chassis.set_arc_turn_pid(180,125,60);
  chassis.wait_drive();
  chassis.set_drive_pid_with_incline_check(70,120,true,true,10,90);
  chassis.wait_until(30);
  control.set_wings_state(ON);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid_with_incline_check(-65,120,true,true,10,90);
  control.set_wings_state(OFF);
  chassis.wait_drive();
  pros::delay(200);
  // chassis.set_drive_pid(10,drive_speed);
  // chassis.wait_drive();
  chassis.set_turn_pid(90,turn_speed);
  chassis.wait_drive();
  
  chassis.set_drive_pid(-20,drive_speed);
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
  chassis.set_drive_pid(15,drive_speed);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING,90,120);
  chassis.wait_drive();
  chassis.set_arc_turn_pid(180,120,65);
  chassis.wait_drive();
  chassis.set_drive_pid_with_incline_check(70,120,true,true,10,90);
  chassis.wait_until(30);
  control.set_wings_state(ON); 
  pros::delay(200);
  chassis.set_drive_pid(-10,drive_speed);
  chassis.wait_drive();

  control.set_wings_state(OFF);
  pros::delay(200);
  chassis.set_turn_pid(270,turn_speed);
  chassis.wait_drive();
  chassis.set_drive_pid(50,50);
  chassis.wait_drive();

  chassis.set_turn_pid(145,turn_speed);
  chassis.wait_drive();

  chassis.set_arc_drive_pid(-55,120,65);
  chassis.wait_drive();

  chassis.set_arc_drive_pid(25,120,65);
  chassis.wait_drive();

  chassis.set_drive_pid_with_incline_check(-20,120,true,true,10,90);
  chassis.wait_drive();
  


  chassis.set_mode(DISABLE);
  chassis.set_tank(0,0);
  pros::delay(5000);

}

void skill_match_classic(){
  auto match_start=pros::millis();
  chassis.set_angle(180);
  chassis.set_exit_condition(chassis.drive_exit,30,50, 30, 100, 1000, 1000);
  chassis.set_exit_velocity_out(chassis.drive_exit,1);
  chassis.set_exit_condition(chassis.turn_exit, 30, 3, 30, 7, 1000, 1000);
  chassis.set_exit_condition(chassis.swing_exit, 30, 3, 30, 7, 1000, 1000);
  
  constexpr static int turn_speed=100;
  constexpr static int drive_speed=120;
  constexpr static int swing_speed=120;
  control.set_intake_state(OUTTAKE);
  
  chassis.set_arc_turn_pid(270,-65,-125);
  // chassis.set_swing_pid(ez::LEFT_SWING,270,-120);
  chassis.wait_drive();
  chassis.set_drive_pid(-12,125);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid(16,drive_speed);
  chassis.wait_drive();
  
  chassis.set_turn_pid(160,turn_speed);
  chassis.wait_drive();
  control.set_catapult_state(LAUNCH);

  chassis.set_drive_pid(-5,35);
  chassis.wait_drive();
  auto gyro_data=chassis.get_gyro();//暂存陀螺仪数据

  chassis.set_mode(ez::DISABLE);//关闭底盘PID控制
  chassis.reset_drive_sensor();//重置底盘编码数据
  // cata_motor_reference.move(120);
  while(pros::millis()-match_start<27000){//30000ms=30s
    pros::delay(10);
    chassis.joy_thresh_opcontrol(0,0);//P控制保持地盘编码为0
  }
  
  chassis.set_angle(gyro_data);//恢复陀螺仪数据

  control.set_catapult_state(RELEASE);
  
  chassis.set_swing_pid(ez::LEFT_SWING,225,swing_speed);
  chassis.wait_drive();

  chassis.set_drive_pid(23,drive_speed);
  chassis.wait_drive();

  chassis.set_turn_pid(0, turn_speed);
  chassis.wait_drive();

  chassis.set_drive_pid(-52,drive_speed);
  chassis.wait_drive();

  // chassis.set_exit_condition(chassis.turn_exit, 30, 5, 30, 10, 500, 500);
  // chassis.set_pid_constants(&chassis.turnPID, 10, 1,25, 20);

  chassis.set_arc_turn_pid(-75,-125,-75);
  chassis.wait_drive();
  // chassis.set_pid_constants(&chassis.turnPID, 4, 0.05,25, 15);
  // chassis.set_exit_condition(chassis.turn_exit, 30, 3, 30, 7, 500, 500);

  chassis.set_mode(DISABLE);
  chassis.set_tank(-120, -120);
  pros::delay(350);
  chassis.set_drive_pid(7,125,false,false);
  chassis.wait_drive();

  chassis.set_drive_pid(-12,125,false,false);
  chassis.wait_drive();



  pros::delay(200);
  chassis.set_drive_pid(5,125,true,false);
  chassis.wait_drive();

  chassis.set_turn_pid(0,50);
  control.set_intake_state(INTAKE);
  chassis.wait_drive();

  chassis.set_drive_pid(40,120);
  chassis.wait_until(15);
  chassis.set_max_speed(50);
  chassis.wait_drive();

  chassis.set_turn_pid(90,50);
  chassis.wait_drive();
  control.set_wings_state(ON);

  chassis.set_drive_pid(13,100);
  chassis.wait_drive();
  chassis.set_swing_pid(LEFT_SWING,180,TURN_SPEED);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);

  chassis.set_drive_pid(50,120);
  chassis.wait_drive();
  control.set_wings_state(OFF);
  chassis.set_swing_pid(LEFT_SWING,0,-TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-15,120);
  chassis.wait_drive();

  chassis.set_drive_pid(37, 45,true);
  chassis.wait_drive();
  chassis.set_turn_pid(90,100);
  chassis.wait_drive();
  control.set_wings_state(ON);

  chassis.set_drive_pid(22, 100,true);
  chassis.wait_drive();
  
  chassis.set_swing_pid(ez::LEFT_SWING, 180, 100);
  chassis.wait_until(120);
  chassis.wait_drive();
  chassis.set_drive_pid(35,120,false);
  chassis.wait_drive();
  chassis.set_drive_pid(-25, 120,true);
  control.set_wings_state(OFF);
  chassis.wait_drive();

  chassis.set_turn_pid(90,100);
  chassis.wait_drive();
  chassis.set_drive_pid(14,60,false);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 180, 50);
  chassis.wait_until(120);
  control.set_wings_state(ON);
  chassis.wait_drive();
  chassis.set_drive_pid(20,100,false);
  chassis.wait_drive();
  control.set_wings_state(OFF);
  chassis.set_turn_pid(90,50);
  chassis.wait_drive();
  chassis.set_drive_pid(25,60,false);
  chassis.wait_drive();

  chassis.set_turn_pid(40,50);
  chassis.wait_drive();
  chassis.set_mode(ez::DISABLE);
  chassis.set_tank(-80,-120);
  pros::delay(2000);
  chassis.set_drive_pid(30,60,false);
  chassis.wait_drive();

}