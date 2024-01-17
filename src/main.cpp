#include "main.h"
#include "EZ-Template/util.hpp"
#include "control.hpp"
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/odom.hpp"

pros::Imu imu(12);
pros::Motor LF(1, pros::E_MOTOR_GEAR_600, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor LM(2, pros::E_MOTOR_GEAR_600, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor LB(3, pros::E_MOTOR_GEAR_600, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor RF(8, pros::E_MOTOR_GEAR_600, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RM(9, pros::E_MOTOR_GEAR_600, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RB(10, pros::E_MOTOR_GEAR_600, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group left_group({LF,LM,LB});
pros::Motor_Group right_group({RF,RM,RB});

// 底盘构造
Drive chassis=Drive(
  // 左侧电机组端口，（负端口将反转电机！）
  left_group

  // 右侧电机组端口，（负端口将反转电机！）
  ,right_group
  
  // 陀螺仪端口
  ,imu

  // 车轮直径（英寸）
  ,4.0

  // 底盘电机转速(100、200、600RPM)
  ,600

  //外齿轮比（必须是小数）
  //例如。如果您的齿比是 84:36，其中 36t 连接电机，则您的 齿比 将为 2.333。
  //例如。如果您的齿比是 36:60，其中 60t 连接电机，则您的 齿比 将为 0.6。
  ,60.0/36.0

  // 左右两侧轮组的距离(不使用陀螺仪控制底盘时需要用到该参数(英寸))
  ,10.5
);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_group, // left motor group
                              &right_group, // right motor group
                              10.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 3.25" omnis
                              300, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis odom(drivetrain, linearController, angularController, sensors);

pros::Motor intake_motor1(4, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup intake_motor_group({intake_motor1});

pros::Motor cata_motor1(6,pros::E_MOTOR_GEAR_200,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata_motor2(7,pros::E_MOTOR_GEAR_100,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup cata_motor_group({cata_motor1,cata_motor2});

pros::Rotation cata_rotation(5,false);
pros::Optical optical(11);
/// 上层机构控制器构造,intake、catapult电机默认为hold模式,可通过调用
Control control=Control(
  // Intake 电机组实例
  intake_motor_group

  // 发射架电机组实例
  ,cata_motor_group

  //角度传感器实例
  ,cata_rotation
  
  //光学传感器实例
  ,optical
  // Wings Ports:{left wing port,right wing port} (negative port will reverse it!)
  // 翅膀的电磁阀端口：{左翼端口，右翼端口}（负端口将反转它！）
  ,{'A', 'B'}

  // Hanger Ports: (negative port will reverse it!)
  //钩子的电磁阀端口：（负端口将反转它！）
  ,{'C'}
); 

pros::Task control_task([](){control.control_task_fn();});

pros::Task catapult_task([](){control.catapult_task_fn();});

pros::Task cata_temp_watchdog_task([](){control.cata_temp_watchdog_fn();});
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
  chassis.set_joystick_threshold(5);//设置摇杆死区的阈值，摇杆的范围在[-127,127]
  chassis.set_tank_min_power(30,30);//设置坦克模式下的最小功率，范围在[0,127],当摇杆输出值小于该值时，底盘将以最小功率运行
  default_constants(); // 设置PID参数。
  
  // 初始化底盘和自动阶段程序选择器
  ez::as::auton_selector.add_autons({
    Auton("skill match",skill_match),
    Auton("Guard.", guard),
    Auton("Attack.", attack),
    Auton("guard_aggressive",guard_aggressive),
    Auton("attack_aggressive",attack_aggressive),

  });
  lemlib::gps=new pros::GPS(13);
  lemlib::gps->initialize_full(0,0,180,1.25*0.0254,-3.25*0.0254);
  lemlib::gps->set_position(0.6,0,0);
  lemlib::kalmanFilterInit();
  chassis.initialize();
  as::initialize();
  odom.calibrate(false);//odom校准s
  
  pros::Task screenTask([=]() {
        while (true) {
            pros::lcd::print(0, "X: %f", odom.getPose().x);
            pros::lcd::print(1, "Y: %f", odom.getPose().y);
            pros::lcd::print(2, "Theta: %f", odom.getPose().theta);
            pros::delay(50);
        }
    });
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
//  chassis.reset_pid_targets(); // 重置所有PID期望为0
//  chassis.reset_gyro(); // 重置陀螺仪
//  chassis.reset_drive_sensor(); // 重置电机编码器
//  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // 将所有底盘电机设置为制动模式
//  ez::as::auton_selector.call_selected_auton(); // 执行程序选择器所选的自动程序
  pros::delay(10000);
  odom.moveToPose(24, 24, 0, 40000,{.lead=0.3,.maxSpeed=40,.minSpeed=40});
  odom.waitUntilDone();
  pros::lcd::print(4, "pure pursuit finished!");
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
  while (true){
    chassis.arcade_standard(SPLIT);
    // chassis.tank();

    pros::delay(ez::util::DELAY_TIME); // 让代码休眠一下以防止过度占用处理器资源
  }


}
