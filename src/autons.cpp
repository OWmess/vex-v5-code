#include "autons.hpp"
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
  chassis.set_slew_min_power(50, 50);//设置最小启动速度，用于缓加速
  chassis.set_slew_distance(7, 7);//设置缓加速的距离
  ///设置PID参数，第一个参数为PID结构体，后面四个参数分别为P、I、D、积分初始值
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
  pros::delay(500);
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
  chassis.set_drive_pid(-48,120,true);
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
void attack() {
  control.set_catapult_state(MIDDLE);
  control.set_intake_state(INTAKE);
  chassis.set_drive_pid(36,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(15,120);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  pros::delay(200);
  chassis.set_drive_pid(-15,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(INTAKE);
  chassis.set_swing_pid(LEFT_SWING,0,SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(14,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(20,120);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);

  chassis.set_drive_pid(-20,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(INTAKE);
  chassis.set_turn_pid(-85,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(12,DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid(-12,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(23,120);
  chassis.wait_drive();
  control.set_intake_state(OUTTAKE);
  pros::delay(200);
  chassis.set_drive_pid(-10,DRIVE_SPEED);
  control.set_intake_state(INTAKE);
  chassis.wait_drive();
  chassis.set_turn_pid(230,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(27,DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid(-15,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(20,120);
  chassis.wait_drive();

  control.set_intake_state(OUTTAKE);
  pros::delay(200);
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

void skill_match(){
  control.set_catapult_state(MIDDLE);
  control.set_wings_state(ON);
  pros::delay(300);
  control.set_wings_state(OFF);
  control.set_intake_state(OUTTAKE);
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(20, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 90,-SWING_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();


  // auto start_t=pros::millis();
  // auto cata_motor_reference=control.get_catapult_motor();
  // cata_motor_reference.move(120);
  // while(pros::millis()-start_t<3000){
  //   pros::delay(ez::util::DELAY_TIME);
  // }
  // control.set_catapult_state(MIDDLE);


}




void test_pid(){
  chassis.set_pid_logger(true);
  chassis.set_drive_pid(50,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-50,DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(RIGHT_SWING, 90, SWING_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(LEFT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}