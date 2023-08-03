#include "main.h"

/**
 * 设置正常运行时的底盘速度
*/
const int DRIVE_SPEED = 100;
const int TURN_SPEED  = 80;
const int SWING_SPEED = 80;

/**
 * @brief      设置底盘控制相关常数，如PID参数等
 *              该函数在程序初始化时调用，PID参数调整遵循Ziegler-Nichols方法
*/
void default_constants() {
  chassis.set_slew_min_power(50, 50);//设置最小启动速度，用于缓加速
  chassis.set_slew_distance(7, 7);//设置缓加速的距离
  ///设置PID参数，第一个参数为PID结构体，后面四个参数分别为P、I、D、最大输出
  chassis.set_pid_constants(&chassis.headingPID,8, 0.000, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.6, 0, 1, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.35, 0, 1, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 45, 10);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
  chassis.set_pid_constants(&chassis.turnPID_gyro_free, 0.45, 0, 2, 0);
}



/**
* 防守方自动程序
*/
void auton_1(){
  ///init lift
  control.set_lift(100,MIDDLE);
  ///****
  chassis.set_drive_pid(30,DRIVE_SPEED,true);
  chassis.wait_drive();
  chassis.set_turn_pid(45,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-30,60,true);
  chassis.wait_drive();
  control.set_intake(INTAKE,120);//吸球
  control.set_hanger(ON);//放下挂钩
  pros::delay(500);
  chassis.set_drive_pid(16, 60, true);
  chassis.wait_drive();
  ///转弯后收起挂钩
  chassis.set_turn_pid(145, TURN_SPEED);
  chassis.wait_drive();
  control.set_hanger(OFF);
  pros::delay(500);
  chassis.set_drive_pid(25, 60, true);
  // chassis.wait_until(20);
  // chassis.set_max_speed(40);
  chassis.wait_drive();

  chassis.set_turn_pid(180, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-63, DRIVE_SPEED, true);//吸球后冲向球门的距离，关键，小心压线
  chassis.wait_drive();
  chassis.set_turn_pid(270, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-10, DRIVE_SPEED, true);//射门前后退一些
  chassis.wait_drive();
  control.set_intake(OUTTAKE,120);
  chassis.set_drive_pid(25, DRIVE_SPEED, true);//射门
  chassis.wait_drive();
  pros::delay(100);

  chassis.set_drive_pid(-10, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(58, DRIVE_SPEED, true);//从球门回退到起点
  chassis.wait_drive();
  chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-45, DRIVE_SPEED, true);//碰杆
  chassis.wait_drive();

}

/**
* 进攻方自动程序
*/
void auton_2(){
  control.set_lift(100,MIDDLE);
  ///****
  // 直走到球门吐第一个球
  chassis.set_drive_pid(65,80,true);
  chassis.wait_drive();
  chassis.set_turn_pid(90,DRIVE_SPEED);
  chassis.wait_drive();
  control.set_intake(OUTTAKE,100);
  chassis.set_drive_pid(15,80,true);
  chassis.wait_drive();
  chassis.set_drive_pid(-5,DRIVE_SPEED,true);
  chassis.wait_drive();
  //第二个球
  chassis.set_turn_pid(-115,DRIVE_SPEED);//从球门转身到第二个球的角度，可能需要调整
  chassis.wait_drive();
  control.set_intake(INTAKE,100);
  chassis.set_drive_pid(32,80,true);//取第二个球时走的距离，可能需要调整
  chassis.wait_drive();
  pros::delay(100);
  chassis.set_turn_pid(-295,TURN_SPEED);//取到第二个球之后转身射门的角度，可能需要调整
  chassis.wait_drive();
  chassis.set_drive_pid(33,60,true);
  chassis.wait_until(20);//走20in之后吐球
  control.set_intake(OUTTAKE,120);
  chassis.wait_drive();
  pros::delay(100);
  //第三个球
  chassis.set_drive_pid(-18,DRIVE_SPEED,true);//从球门回到第三个球的距离，可能需要调整
  chassis.wait_drive();
  control.set_intake(INTAKE,120);
  chassis.set_turn_pid(-0,TURN_SPEED);//直角转弯直接取第三个球，一般不需要调整
  chassis.wait_drive();
  chassis.set_drive_pid(13,50,true);//取第三个球时走的距离，可能需要调整，小心压线！
  chassis.wait_drive();
  pros::delay(100);
  // chassis.set_drive_pid(-5,DRIVE_SPEED,true);
  // chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  control.set_intake(OUTTAKE,120);
  chassis.set_drive_pid(25,DRIVE_SPEED,true);//第三个球射门
  chassis.wait_drive();

}

void auton_3(){
  chassis.set_drive_pid(25,DRIVE_SPEED,true);
  chassis.wait_drive();
  chassis.set_turn_pid(-90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(20,DRIVE_SPEED,true);
  chassis.wait_drive();
  pros::delay(500);


//***由于抛投暂时不可调，所以先注释掉
  // constexpr float lift_t=30;//抛投的总时间，单位为s
  // auto start_t=pros::millis();
  // while(true){
  //   auto now_t=pros::millis();
  //   if((now_t-start_t)/1000>30){
  //     break;
  //   }
  //   set_lift(60);//第一个参数为电机方向，第二个参数为电机速度,在control.cpp中修改函数
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
  control.set_wings(ON);//true为打开两侧挡板，false为关闭两侧挡板
  pros::delay(500);
  chassis.set_drive_pid(-35,DRIVE_SPEED,true);
  chassis.wait_drive();
  control.set_wings(OFF);
  pros::delay(500);
  chassis.set_drive_pid(35,DRIVE_SPEED,true);
  chassis.wait_drive();

  control.set_wings(ON);
  pros::delay(500);
  chassis.set_drive_pid(-35,DRIVE_SPEED,true);
  chassis.wait_drive();

  control.set_wings(OFF);
  pros::delay(500);

  chassis.set_drive_pid(35,DRIVE_SPEED,true);
  chassis.wait_drive();

}

void test_pid(){
  chassis.set_drive_pid(35,DRIVE_SPEED,true);
  chassis.wait_until(15);
  chassis.set_max_speed(60);
  chassis.wait_drive();
  chassis.set_turn_pid(-90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-35,DRIVE_SPEED,true);
  chassis.wait_drive();
  chassis.set_swing_pid(LEFT_SWING,90,SWING_SPEED);
  chassis.wait_drive();
}