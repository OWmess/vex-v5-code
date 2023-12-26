#include "main.h"
#include "EZ-Template/util.hpp"
#include "control.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

// 底盘构造
Drive chassis=Drive(
  // 左侧电机组端口，（负端口将反转电机！）
  {1, 2, -3}

  // 右侧电机组端口，（负端口将反转电机！）
  ,{-10, -9, 8}
  

  // 陀螺仪端口
  ,13

  // 车轮直径（英寸）
  ,4.0

  // 底盘电机转速(100、200、600RPM)
  ,600

  //外齿轮比（必须是小数）
  //例如。如果您的齿比是 84:36，其中 36t 连接电机，则您的 齿比 将为 2.333。
  //例如。如果您的齿比是 36:60，其中 60t 连接电机，则您的 齿比 将为 0.6。
  ,72.0/36.0

  // 左右两侧轮组的距离(不使用陀螺仪控制底盘时需要用到该参数(英寸))
  ,12.0
);

/// 上层机构控制器构造,intake、catapult电机默认为hold模式,可通过调用
Control control=Control(
  // Intake 电机组端口，（负端口将反转电机！）
  {4 }

  // Intake 电机组的RPM,
  //可选项有：
  //pros::E_MOTOR_GEAR_200（200RPM）
  //pros::E_MOTOR_GEAR_100（100RPM）
  //pros::E_MOTOR_GEAR_600（600RPM）
  ,pros::E_MOTOR_GEAR_200

  // 投石机电机端口（负端口将反转它！）
  ,{6,-7}

  // 投石机 电机RPM,可选项同上
  ,pros::E_MOTOR_GEAR_100

  // 投石机的角度传感器所在端口,若角度传感器正方向与投石机下压方向相反则为负
  ,5

  // Wings Ports:{left wing port,right wing port} (negative port will reverse it!)
  // 翅膀的电磁阀端口：{左翼端口，右翼端口}（负端口将反转它！）
  ,{'B', 'C'}

  // Hanger Ports: (negative port will reverse it!)
  //钩子的电磁阀端口：（负端口将反转它！）
  ,{'A'}
);




/**
*运行初始化代码。发生在程序刚启动的时候，在所有比赛模式、初始化之前
*推荐将此模式的执行时间保持在几秒钟以内。
*/
void initialize() {
  pros::delay(500);

  //配置底盘参数
  chassis.toggle_modify_curve_with_controller(false); //是否允许使用操纵杆上的按钮（左右键）修改控制器曲线
  chassis.set_active_brake(0.05); // 设置主动制动kP，建议为0.1。
  chassis.set_curve_default(0, 0); //控制器曲线的默认值。如果使用Tank模式，则仅使用第一个参数。（如果您有 SD 卡，请注释掉此行！）
  chassis.set_joystick_threshold(5);//设置摇杆死区的阈值，摇杆的范围在[-127,127]
  chassis.set_tank_min_power(30,30);//设置坦克模式下的最小功率，范围在[0,127],当摇杆输出值小于该值时，底盘将以最小功率运行
  default_constants(); // 设置PID参数。
  
  // 初始化底盘和自动阶段程序选择器
  ez::as::auton_selector.add_autons({
    Auton("Attack.", attack),
    Auton("Guard.", guard_1),
    Auton("skill match",skill_match),
    Auton("guard_aggressive",guard_aggressive),
    Auton("attack_aggressive",attack_aggressive),
    Auton("Conservatively attack. ", conservatively_attack),
    Auton("test_function.", test_pid),

  });
  chassis.initialize();
  as::initialize();
  chassis.set_exit_condition(chassis.drive_exit, 30, 50, 100, 150, 500, 1000);

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
  chassis.set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
  Control_State wings_state=OFF;
  Control_State default_intake_state=INTAKE;//r1按下时，intake的默认状态
  control.set_intake_state(STOP);
  bool launching=false;
  pros::ADIDigitalOut hang('D');
  pros::Distance distance(11);
  while (true){
    chassis.arcade_standard(SPLIT);
    // chassis.tank();
    //根据按钮状态控制机器人
    //intake状态控制
    if(Controller_Button_State::R1_new_press()){//R1按下时，打开或关闭intake
        if(control.get_intake_state()==INTAKE||control.get_intake_state()==OUTTAKE){//如果intake正在运行，则停止
          control.set_intake_state(STOP);
        }else{//如果intake没有运行，则打开
          control.set_intake_state(default_intake_state);
        }
    }else if(Controller_Button_State::R2_pressed()){//R2按下时，翻转intake
      control.set_intake_state(!default_intake_state);
    }else if(control.get_intake_state()!=STOP){//如果intake没有停止，则恢复默认状态
      control.set_intake_state(default_intake_state);
    }

    //大翅膀状态控制
    if(Controller_Button_State::L1_new_press()){//L1按下时，翻转翅膀状态
      wings_state=!wings_state;
      control.set_wings_state(wings_state);
    }else if(Controller_Button_State::L2_new_press()){//L2按下时，关闭翅膀
      wings_state=OFF;
      control.set_wings_state(OFF);
    }

    //被动挂（挡板）控制
    if(Controller_Button_State::UP_new_press()){
      hang.set_value(ON);
    }else if(Controller_Button_State::DOWN_new_press()){
      hang.set_value(OFF);
    }

    //小手臂控制
    if(Controller_Button_State::LEFT_new_press()){
      control.set_armer_state(ON);
    }else if(Controller_Button_State::RIGHT_new_press()){
      control.set_armer_state(OFF);
    }

    //发射架控制
    if(Controller_Button_State::X_new_press()){
      launching=!launching;
      if(launching){
      control.set_catapult_state(KEEP_LAUNCHING);
      }else{
        control.set_catapult_state(RELEASE);
      }
    }
    
    if(Controller_Button_State::B_new_press()){
      std::deque<float> distance_queue;
      bool press=false;
      while(true){
        control.set_catapult_state(READY);
        while(true){
          pros::delay(10);
          distance_queue.push_back(distance.get());
          if(distance_queue.size()>1){
            distance_queue.pop_front();
          }

          bool res=true;
          for_each(distance_queue.begin(),distance_queue.end(),[&res](float &distance){
          if(!(distance<100)){
            res=false;
          }
          });
          if(res){
            while(true){
              control.cata_move(125);
              cout<<"angle:"<<control.cata_rotation->get_angle()<<endl;
              pros::delay(10);
              float angle=control.cata_rotation->get_angle()/100.f;
              if(angle>300||angle<5){
                break;
              }
              if(Controller_Button_State::B_new_press()){
                press=true;
                break;
              }
            }
            break;
          }
          if(Controller_Button_State::B_new_press()){
            press=true;
            break;
          }
        }
        if(press){
          control.set_catapult_state(RELEASE);
          break;
        }
        pros::delay(50);
      }
      pros::delay(200);
    }


    pros::delay(ez::util::DELAY_TIME); // 让代码休眠一下以防止过度占用处理器资源
  }


}
