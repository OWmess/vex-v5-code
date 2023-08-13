#include "main.h"

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
  // chassis.set_pid_constants(&chassis.headingPID,4, 0.000, 13, 0);
  // chassis.set_pid_constants(&chassis.forward_drivePID, 2, 0, 4, 0);
  // chassis.set_pid_constants(&chassis.backward_drivePID, 0.35, 0, 1, 0);
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

void auton_3(){
  chassis.set_drive_pid(25,DRIVE_SPEED,true);
  chassis.wait_drive();
  chassis.set_turn_pid(-90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10,DRIVE_SPEED,true);
  chassis.wait_drive();
  pros::delay(500);


//***由于抛投暂时不可调，所以先注释掉
  // constexpr float catapult_t=30;//抛投的总时间，单位为s
  // auto start_t=pros::millis();
  // while(true){
  //   auto now_t=pros::millis();
  //   if((now_t-start_t)/1000>30){
  //     break;
  //   }
  //   set_catapult(60);//第一个参数为电机方向，第二个参数为电机速度,在control.cpp中修改函数
  //   pros::delay(1000);//每次抛投的延迟，单位为ms
  // }
//**************/
  chassis.set_drive_pid(-20,80,true);
  chassis.wait_drive();
  chassis.set_turn_pid(0,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-20,DRIVE_SPEED,true);
  chassis.wait_drive();
  chassis.set_turn_pid(-90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-105,DRIVE_SPEED,true);//到进攻区的长距离前进（路过提升杆那一段）
  chassis.wait_drive();
  //先打侧面球门
  chassis.set_turn_pid(-135,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-35,DRIVE_SPEED,true);
  chassis.wait_drive();
  chassis.set_turn_pid(-180,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-15,DRIVE_SPEED,true);
  chassis.wait_drive();

  chassis.set_drive_pid(5,80,true);//撞击侧门后回退的距离，可能需要调整
  chassis.wait_drive();
  chassis.set_turn_pid(-270,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-55,DRIVE_SPEED,true);//到中间障碍杆的距离，可能需要调整
  chassis.wait_drive();

  chassis.set_turn_pid(-180,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-45,DRIVE_SPEED,true);//到正面球门的距离
  chassis.wait_drive();

  chassis.set_turn_pid(-90,TURN_SPEED);
  chassis.wait_drive();
  //重复两次撞正面球门
  control.set_wings_state(ON);//true为打开两侧挡板，false为关闭两侧挡板
  pros::delay(500);
  chassis.set_drive_pid(-35,DRIVE_SPEED,true);
  chassis.wait_drive();
  control.set_wings_state(OFF);
  pros::delay(500);
  chassis.set_drive_pid(35,DRIVE_SPEED,true);
  chassis.wait_drive();

  control.set_wings_state(ON);
  pros::delay(500);
  chassis.set_drive_pid(-35,DRIVE_SPEED,true);
  chassis.wait_drive();

  control.set_wings_state(OFF);
  pros::delay(500);

  chassis.set_drive_pid(35,DRIVE_SPEED,true);
  chassis.wait_drive();

}

void test_pid(){
  control.set_catapult_state(DOWN);
  chassis.set_swing_pid(LEFT_SWING,-90,DRIVE_SPEED);
  chassis.wait_drive();

}