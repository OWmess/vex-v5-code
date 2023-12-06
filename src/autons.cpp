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
  chassis.set_slew_min_power(50, 50);//设置最小启动速度，用于缓加速
  chassis.set_slew_distance(7, 7);//设置缓加速的距离
  ///设置PID参数，第一个参数为PID结构体，后面四个参数分别为P、I、D、积分初始值
  chassis.set_pid_constants(&chassis.headingPID,4, 0.000, 13, 0);
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
  control.set_catapult_state(MIDDLE);
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
  chassis.set_drive_pid(8, DRIVE_SPEED,true);
  chassis.wait_drive();
  control.set_armer_state(ON);
  pros::delay(500);
  control.set_intake_state(OUTTAKE);

  chassis.set_swing_pid(ez::LEFT_SWING,45,80);
  chassis.wait_drive();
  pros::delay(300);
  control.set_intake_state(STOP);
  control.set_intake_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.set_drive_pid(12, DRIVE_SPEED,true);
  chassis.wait_drive();
  pros::delay(200);

  chassis.set_drive_pid(-5, DRIVE_SPEED,true);
  chassis.wait_drive();
  pros::delay(200);
  control.set_armer_state(OFF);

  chassis.set_drive_pid(10, DRIVE_SPEED,true);
  chassis.wait_drive();
  control.set_wings_state(ON);
  chassis.set_drive_pid(-5, DRIVE_SPEED,true);
  chassis.wait_drive();
  
  pros::delay(300);
  control.set_wings_state(OFF);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-26,DRIVE_SPEED,true);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  control.pto_arm_mode();
  chassis.set_mode(ez::DISABLE);
  pros::delay(500);
  control.set_catapult_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  control.cata_move(-125);
  pros::delay(1000);

  pros::Task thread([](){
    auto cata_motor=control.get_catapult_motor();
    while(!cata_motor[0].is_over_current()&&!cata_motor[1].is_over_current()){
        control.cata_move(-125);
        pros::delay(10);
    }
    control.cata_brake();
  });

  chassis.set_drive_pid(-50,40,true);
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

void guard_aggressive(){
  constexpr static int turn_speed=120;
  constexpr static int swing_speed=120;
  control.set_catapult_state(INIT_DOWN);
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
  control.set_catapult_state(MIDDLE);

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
  control.set_catapult_state(INIT_MIDDLE);
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
void attack() {
  constexpr static int turn_speed=120;
  control.set_catapult_state(MIDDLE);
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

void conservatively_attack(){
  control.set_catapult_state(MIDDLE);
  chassis.set_drive_pid(39,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake_state(INTAKE);
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
  //   chassis.set_pid_constants(&chassis.headingPID,4, 0.000, 13, 0);
  // chassis.set_pid_constants(&chassis.forward_drivePID, 1, 0.001, 2, 150);
  // chassis.set_pid_constants(&chassis.backward_drivePID, 0.5, 0.001, 4, 150);
  // chassis.set_pid_constants(&chassis.turnPID, 4, 0.05,25, 15);
  // chassis.set_pid_constants(&chassis.swingPID, 7, 0.05, 45, 10);
  chassis.set_exit_condition(chassis.drive_exit,30,50, 30, 100, 2000, 500);
  chassis.set_exit_condition(chassis.turn_exit, 30, 3, 30, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 30, 3, 30, 7, 500, 500);
  
  constexpr static int turn_speed=100;
  constexpr static int drive_speed=120;
  constexpr static int swing_speed=120;
  control.set_wings_state(ON);
  control.set_intake_state(STOP);
  pros::delay(200);
  control.set_wings_state(OFF);
  // chassis.set_drive_pid(5, DRIVE_SPEED,true);
  // chassis.wait_drive();
  chassis.set_turn_pid(65, turn_speed);
  chassis.wait_drive();
  // chassis.set_drive_pid(10, 60);
  // chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING, -18, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(10, 25,false,false);
  chassis.wait_drive();
  // chassis.set_turn_pid(-18, turn_speed);
  // chassis.toggle_auto_drive(false);
  // chassis.set_tank(15, 15);
  auto start_t=pros::millis();

  // auto cata_motor_reference=control.get_catapult_motor();
  // cata_motor_reference.move(120);
  // while(pros::millis()-start_t<3000){//30000ms=30s
  //   cata_motor_reference.move(120);

  //   pros::delay(100);
  // }
  // chassis.toggle_auto_drive(true);
  control.set_catapult_state(MIDDLE);
  // chassis.set_drive_pid(-20, drive_speed);
  // chassis.wait_drive();
  chassis.set_arc_turn_pid(-80, -120, -50);
  chassis.wait_drive();
  chassis.set_drive_pid(-20, drive_speed,false,true);
  chassis.wait_drive();
  chassis.set_arc_turn_pid(0, -55, -125);
  chassis.wait_drive();

  // chassis.set_swing_pid(ez::LEFT_SWING, -60, -swing_speed);
  // chassis.wait_drive();
  // chassis.set_drive_pid(-15, drive_speed);
  // chassis.wait_drive();
  // chassis.set_swing_pid(ez::RIGHT_SWING, -0, -swing_speed);2
  control.set_wings_state(ON);
  chassis.wait_drive();
  chassis.set_drive_pid_with_incline_check(-70, 125,true,true,10.f,90);
  chassis.wait_drive();

  control.set_wings_state(OFF);
  chassis.set_turn_pid(0, turn_speed);
  chassis.wait_drive();
  chassis.set_drive_pid_with_incline_check(60 , 125,true,true,10.f,90);
  chassis.wait_drive();
  // pros::delay(2000);
  chassis.set_turn_pid(-60, turn_speed);
  chassis.wait_drive();
  chassis.set_arc_turn_pid(-18, 120, 60);
  chassis.wait_drive();

  chassis.set_drive_pid(30, drive_speed);
  chassis.wait_until(20);
  chassis.set_max_speed(25);
  chassis.wait_drive();

  start_t=pros::millis();
  // while(pros::millis()-start_t<3000){//30000ms=30s
  //   cata_motor_reference.move(120);

  //   pros::delay(100);
  // }

  control.set_catapult_state(MIDDLE);
  // chassis.set_drive_pid(-20, drive_speed);
  // chassis.wait_drive();
  chassis.set_arc_turn_pid(-85, -120, -65);
  chassis.wait_drive();
  chassis.set_drive_pid(-15,drive_speed);
  chassis.wait_drive();
  chassis.set_arc_turn_pid(0, -45, -125);
  chassis.wait_drive();

  // chassis.set_swing_pid(ez::LEFT_SWING, -60, -swing_speed);
  // chassis.wait_drive();
  // chassis.set_drive_pid(-15, drive_speed);
  // chassis.wait_drive();
  // chassis.set_swing_pid(ez::RIGHT_SWING, -0, -swing_speed);2
  control.set_wings_state(ON);
  chassis.wait_drive();
  chassis.set_drive_pid_with_incline_check(-68, 125,true,true,10.f,90);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid(20,drive_speed);
  chassis.wait_drive();

  // chassis.set_turn_pid(90, TURN_SPEED);
  // chassis.wait_drive();
  // control.set_intake_state(OUTTAKE);
  // chassis.set_drive_pid(12, 120);
  // chassis.wait_drive();

  // pros::delay(200);

  // chassis.set_drive_pid(-15, DRIVE_SPEED,true);
  // chassis.wait_drive();
  // control.set_intake_state(STOP);
  // chassis.set_swing_pid(ez::LEFT_SWING, -21,SWING_SPEED);
  
  // chassis.wait_drive();

  // control.set_catapult_state(MIDDLE);
  // pros::delay(300);
  // chassis.set_turn_pid(20, TURN_SPEED);
  // chassis.wait_drive();

  // chassis.set_drive_pid(-35, DRIVE_SPEED,true);
  // chassis.wait_drive();

  // chassis.set_turn_pid(0, -TURN_SPEED);
  // chassis.wait_drive();

  // chassis.set_drive_pid(-60, DRIVE_SPEED,true);
  // chassis.wait_drive();
  // control.set_intake_state(OUTTAKE);
  // chassis.set_swing_pid(LEFT_SWING, -45,45);
  // chassis.wait_drive();

  // chassis.set_drive_pid(-20,45,true);
  // chassis.wait_drive();

  // chassis.set_swing_pid(LEFT_SWING,-90, 45);
  // chassis.wait_drive();
  // chassis.set_drive_pid(-20,120);
  // chassis.wait_drive();
  // pros::delay(200);
  // chassis.set_drive_pid(15,120);
  // chassis.wait_drive();
  // chassis.set_drive_pid(-20,120);
  // chassis.wait_drive();
  // chassis.set_drive_pid(8,120);
  // chassis.wait_drive();

  // chassis.set_turn_pid(0,TURN_SPEED);
  // chassis.wait_drive();

  // chassis.set_drive_pid(40,45);
  // chassis.wait_drive();

  // chassis.set_swing_pid(LEFT_SWING,90,SWING_SPEED);
  // chassis.wait_drive();
  // control.set_wings_state(ON);

  // chassis.set_drive_pid(25,45);
  // chassis.wait_drive();

  // chassis.set_swing_pid(LEFT_SWING,180,SWING_SPEED);
  // chassis.wait_drive();
  // pros::delay(300);
  // chassis.set_drive_pid(40, 120);
  // chassis.wait_drive();
  // chassis.set_drive_pid(-40, 120);
  // chassis.wait_drive();

  // chassis.set_drive_pid(40, 120);
  // chassis.wait_drive();
  // chassis.set_drive_pid(-40, 120);
  // chassis.wait_drive();
  // chassis.set_turn_pid(150, TURN_SPEED);
  // chassis.wait_drive();
  // chassis.set_drive_pid(40, 120);
  // chassis.wait_drive();
  // chassis.set_drive_pid(-40, 120);
  // chassis.wait_drive();
  // auto start_t=pros::millis();
  // auto cata_motor_reference=control.get_catapult_motor();
  // cata_motor_reference.move(120);
  // while(pros::millis()-start_t<3000){
  //   pros::delay(ez::util::DELAY_TIME);
  // }
  // control.set_catapult_state(MIDDLE);


}




void test_pid(){
  pros::delay(1000);
  control.pto_arm_mode();
  chassis.set_mode(ez::DISABLE);
  pros::delay(500);
  control.cata_move(-120);
  pros::Task thread([](){
    pros::delay(500);
    auto cata_motor=control.get_catapult_motor();
    while(!cata_motor[0].is_over_current()&&!cata_motor[1].is_over_current()){
        control.cata_move(-120);
        pros::delay(10);
    }
    control.cata_brake();
  });
  
  cout<<"222\n";
  pros::delay(20000);
}

void get_sensor_data(){
  pros::lcd::clear();
  chassis.set_active_brake(0);
  while(true){
    double left=chassis.left_sensor();
    double right=chassis.right_sensor();
    double tick_per_inch=chassis.get_tick_per_inch();
    left/=tick_per_inch;
    right/=tick_per_inch;
    double heading=chassis.get_gyro();

    pros::screen::print(pros::E_TEXT_MEDIUM,0,"distance left:%lf ,right:%lf",left,right);
    pros::screen::print(pros::E_TEXT_MEDIUM,1,"heading:%lf",heading);
    auto bt=pros::lcd::read_buttons();
    bt&=0x00000010;
    bt=bt>>1;
    if(bt==1){
      auto t=pros::millis();
      while(bt==1){
        bt=pros::lcd::read_buttons();
        bt&=0x00000010;
        bt=bt>>1;
        pros::delay(10);
      }
      if(pros::millis()-t>1000){
        chassis.imu.set_heading(0);
      }else{
        chassis.reset_drive_sensor();
      }
    }
    pros::delay(10);
  }
}