#include "main.h"

// 底盘构造
Drive chassis=Drive(
  // 左侧电机组端口，（负端口将反转电机！）
  {-10, -20, -9}

  // 右侧电机组端口，（负端口将反转电机！）
  ,{11, 3, 2}

  // 陀螺仪端口
  ,4

  // 车轮直径（英寸）
  ,3.25

  // 底盘电机转速(100、200、600RPM)
  ,600

  //外齿轮比（必须是小数）
  //例如。如果您的齿比是 84:36，其中 36t 连接电机，则您的 齿比 将为 2.333。
  //例如。如果您的齿比是 36:60，其中 60t 连接电机，则您的 齿比 将为 0.6。
  ,60.0/36.0

  // 左右两侧轮组的距离
  ,12.0
);

// 上层机构控制器构造
Control control=Control(
  // Intake 电机组端口，（负端口将反转电机！）
  {13, -19}

  // Intake 电机组的RPM,
  //可选项有：
  //pros::E_MOTOR_GEAR_200（200RPM）
  //pros::E_MOTOR_GEAR_100（100RPM）
  //pros::E_MOTOR_GEAR_600（600RPM）
  ,pros::E_MOTOR_GEAR_200

  // Lift电机端口（负端口将反转它！）
  ,-1

  // Lift 电机的RPM,可选项同上
  ,pros::E_MOTOR_GEAR_200

  // Lift的触碰按钮端口
  ,'H'

  // Wings Ports:{left wing port,right wing port} (negative port will reverse it!)
  // 翅膀的电磁阀端口：{左翼端口，右翼端口}（负端口将反转它！）
  ,{-'C', 'D'}

  // Hanger Ports:{hanger_arm,hanger_claw} (negative port will reverse it!)
  //钩子的电磁阀端口：{爪臂的端口,爪子的端口}（负端口将反转它！）
  ,{'A', 'B'}
);



/**
*运行初始化代码。发生在程序刚启动的时候，在所有比赛模式、初始化之前
*推荐将此模式的执行时间保持在几秒钟以内。
*/
void initialize() {
  pros::delay(500);

  //配置底盘参数
  chassis.toggle_modify_curve_with_controller(false); //是否允许使用操纵杆上的按钮（左右键）修改控制器曲线
  chassis.set_active_brake(0.1); // 设置主动制动kP，建议为0.1。
  chassis.set_curve_default(0, 0); //控制器曲线的默认值。如果使用Tank模式，则仅使用第一个参数。（如果您有 SD 卡，请注释掉此行！）
  default_constants(); // 设置PID参数。
  
  // 初始化底盘和自动阶段程序选择器
  ez::as::auton_selector.add_autons({
    Auton("Guard.", auton_1),
    Auton("Attack.", auton_2),
    Auton("1min. ", auton_3),
    Auton("test pid",test_pid),
  });
  chassis.initialize();
  as::initialize();

}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 * 在机器人处于场地管理系统或VEX竞赛开关的禁用状态时才会运行，
 * 当处于启用状态时，此任务将退出，并运行自动/手动控制。
 */
void disabled() {
  // . . .
  printf("disabled\n");
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 * 在连接到场地管理系统或VEX竞赛开关时运行，此任务将在比赛开始后退出。
 */
void competition_initialize() {
  // . . .  
  printf("competition_initialize\n");

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 * 自动阶段运行的代码
 */

void autonomous() {
  chassis.reset_pid_targets(); // 重置所有PID期望为0
  chassis.reset_gyro(); // 重置陀螺仪
  chassis.reset_drive_sensor(); // 重置电机编码器
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // 将所有底盘电机设置为制动模式
  control.set_wings(OFF);
  control.set_hanger(OFF);
  pros::delay(200);
  // auton_1();// 防守方案
  // auton_2();// 攻击方案
  // auton_3();//1分钟全自动方案
  ez::as::auton_selector.call_selected_auton(); // 执行程序选择器所选的自动程序


  
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 * 手控阶段运行的代码，在没有连接到场地控制器时，此函数将在初始化后立即运行。
 */
void opcontrol() {
<<<<<<< HEAD
  chassis.set_drive_pid(100,100);
  chassis.wait_drive();
=======
  pros::Motor lift(-1);
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
  while (true)
  {
    chassis.tank(); // Tank 模式
    //根据按钮状态控制机器人
    if(Controller_Button_State::R1_pressed()){
      control.set_intake(INTAKE,100);
    }
    else if(Controller_Button_State::R2_pressed()){
      control.set_intake(OUTTAKE,100);
    }
    if(Controller_Button_State::L1_pressed()){
      control.set_wings(ON);
    }
    else if(Controller_Button_State::L2_pressed()){
      control.set_wings(OFF);
    }
    if(Controller_Button_State::A_pressed()){
      lift.move(100);
    }
    if(Controller_Button_State::UP_pressed()){
      lift.move(-100);
    }else if(Controller_Button_State::DOWN_pressed()){
      lift.move(100);
    }else{
      lift.brake();
    }
    if(Controller_Button_State::RIGHT_pressed()){
      control.set_lift(80,MIDDLE);
    }
    
    pros::delay(ez::util::DELAY_TIME); // 让代码休眠一下以防止过度占用处理器资源
  }
>>>>>>> main

}
