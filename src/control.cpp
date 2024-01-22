#include "control.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

///********************************参数配置*******************************************///
/**
 * @brief 须在主控-角度传感器中设置发射架的最高点为0（SET ZERO）
 *        catapult 在中间和在底部时的角度值，该值可通过主控器直接观测得到，但若角度传感器正方向与投石机下压方向相反，
 *        则需取反后+360
          数值越大，发射架越往下
 *        
*/
#define CATA_READY_POS      65
#define CATA_PERCENTAGE     70

/**
 * @brief 发射架的PID参数
*/
#define CATA_KP             15
#define CATA_KI             0.1
#define CATA_KD             20
#define CATA_START_I        5
///**********************************************************************************///

static inline bool check_task_notify(pros::Task &task,uint32_t delay,bool &flag){
  auto notify=task.notify_take(false, delay);
  if(notify) {
    task.notify();
    flag=true;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    return true;
  }
  return false;
}



Control_State Control::intake_state=STOP;
Catapult_State Control::catapult_state=RELEASE;
Control_State Control::wings_state=OFF;
Control_State Control::armer_state=OFF;


Control::Control(pros::Motor_Group &intake_motors,pros::Motor_Group &catapult_motor,pros::Rotation &catapult_rotation,pros::Optical &opt,const std::vector<int8_t> &wings_ports,
    const std::vector<int8_t> &armer_ports):intake_motors(intake_motors),catapult_motors(catapult_motor),cata_rotation(catapult_rotation),optical(opt){



  for(const auto &wing_port:wings_ports){
    PneumaticsStruct tmp;
    tmp.pneumatics=std::make_shared<pros::ADIDigitalOut>(abs(wing_port),util::is_reversed(wing_port));
    tmp.reversed=util::is_reversed(wing_port);
    wings.push_back(tmp);
  }

  for(const auto &armer_port:armer_ports){
    PneumaticsStruct tmp;
    tmp.pneumatics=std::make_shared<pros::ADIDigitalOut>(abs(armer_port),util::is_reversed(armer_port));
    tmp.reversed=util::is_reversed(armer_port);
    armers.push_back(tmp);
  }


  cata_rotation.set_data_rate(5);
  // 配置catapult的pid参数
  cata_PID={CATA_KP,CATA_KI,CATA_KD,CATA_START_I};
  cata_PID.set_velocity_out(0.1);
  cata_PID.set_exit_condition(20, 1, 50, 1, 2000, 3000);
  
  //配置光学传感器参数
  //LED最高亮度
  optical.set_led_pwm(0);
  //禁用手势检测
  optical.disable_gesture();
}












void Control::set_intake(int speed,Control_State state){
  if(state==STOP){
    intake_motors.brake();
    return;
  }
  speed=(state==INTAKE)?speed:-speed;
  intake_motors.move(speed);
}


void Control::set_catapult(int percentage,Catapult_State state) {
  auto detect_fn=[this,percentage](){
    while(true){
      double hue=optical.get_hue();
      double sat=optical.get_saturation();
      int proximity=optical.get_proximity();
      bool triball_detected=(hue>85&&hue<105)&&(sat>0.3&&sat<0.7)&&proximity>240;
      if(triball_detected){
        cout<<"triball detected\n";
        set_catapult(percentage,READY);
        while(triball_detected) {
          double hue=optical.get_hue();
          double sat=optical.get_saturation();
          int proximity=optical.get_proximity();
          triball_detected=(hue>85&&hue<105)&&(sat>0.3&&sat<0.7)&&proximity>240;
          pros::delay(5);
        }
      }
      pros::delay(10);
    }
  };
  static pros::Task detect_task(detect_fn);
  static bool flag=false;
  if(!flag){
    flag=true;
    detect_task.suspend();
  }
  if(state==RELEASE){
    catapult_motors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    catapult_motors.brake();
    detect_task.suspend();
    optical.set_led_pwm(0);
  }else if(state==READY){
    catapult_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    cata_move_rpm(percentage);
    if(cata_rotation.get_position()/100.f>CATA_READY_POS-15){
      auto start=pros::millis();
      while(cata_rotation.get_position()/100.f>CATA_READY_POS*0.8){
        // if(drive_catapult){
        //   return;
        // }
        if(pros::millis()-start>1000){
          catapult_motors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
          break;
        }
        pros::delay(5);
      }
    }
    while(cata_rotation.get_position()/100.f<CATA_READY_POS){
      // if(drive_catapult){
      //   return;
      // }
      pros::delay(5);
    }
    catapult_motors.brake();
  }else if(state==LAUNCH){
    cata_move_rpm(percentage);
  }else if(state==DETECT){
    cout<<"detect\n";
    optical.set_led_pwm(100);
    detect_task.resume();
  }
}

void Control::set_wings(Control_State state){
  if(state==ON||state==OFF){
    for(const auto &wing:wings){
      wing.pneumatics->set_value(state==ON?!wing.reversed:wing.reversed);
    }
  }else if(state==LEFT_OFF||state==LEFT_ON){
    wings[0].pneumatics->set_value(state==LEFT_ON?!wings[0].reversed:wings[0].reversed);
  }else if(state==RIGHT_OFF||state==RIGHT_ON){
    wings[1].pneumatics->set_value(state==RIGHT_ON?!wings[1].reversed:wings[1].reversed);
  }
}

void Control::set_armer(Control_State state){
  for(const auto &armer:armers){
    armer.pneumatics->set_value(state==ON?!armer.reversed:armer.reversed);
  }
}

void Control::set_catapult_up_pos(double pos){
  catapult_up_pos=pos;
}

void Control::set_catapult_middle_pos(double pos){
  catapult_middle_pos=pos;
}

void Control::set_catapult_down_pos(double pos){
  catapult_down_pos=pos;
}

void Control::catapult_task_fn(){
  while (true) {
    if(drive_catapult){
      drive_catapult=false;
      set_catapult(CATA_PERCENTAGE,catapult_state);
    }
    pros::delay(20);
  }
}

void Control::control_task_fn(){
  while(true){
    if(!pros::competition::is_autonomous()){
      controller_event_handling();
    }
    drive_event_handling();
    pros::delay(20);
  }
}

void Control::controller_event_handling(){
  static Control_State wings_state=OFF;
  static Control_State default_intake_state=INTAKE;//r1按下时，intake的默认状态
  static bool launch=false;
  static pros::ADIDigitalOut hanger_pneumatics('D');//被动挂的电磁阀
  //根据按钮状态控制机器人
  //intake状态控制
  if(Controller_Button_State::R1_new_press()){//R1按下时，打开或关闭intake
      if(control.get_intake_state()!=STOP){//如果intake正在运行，则停止
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
  if(Controller_Button_State::L1_new_press()){//L1按下时，打开翅膀
    wings_state=!wings_state;
    control.set_wings_state(wings_state);
  }else if(Controller_Button_State::L2_new_press()){//L2按下时，关闭翅膀
    wings_state=OFF;
    control.set_wings_state(OFF);
  }

  if(Controller_Button_State::X_new_press()){//X键切换发射架状态
    launch=!launch;
    if(launch)
      control.set_catapult_state(LAUNCH);
    else
      control.set_catapult_state(RELEASE);
  }else if(Controller_Button_State::B_new_press()){//B按下时，关闭armer
    launch=false;
    control.set_catapult_state(RELEASE);
  }else if(Controller_Button_State::A_new_press()){//Y按下时，关闭armer
    control.set_catapult_state(READY);
  }


  if(Controller_Button_State::RIGHT_new_press()){//左右键切换侧挂的开关
    control.set_armer_state(ON);
  }else if(Controller_Button_State::LEFT_new_press()){
    control.set_armer_state(OFF);
  }

  if(Controller_Button_State::UP_new_press()){//上下键切换被动挂的开关
    hanger_pneumatics.set_value(HIGH);
  }else if(Controller_Button_State::DOWN_new_press()){
    hanger_pneumatics.set_value(LOW);
  }
}


void Control::drive_event_handling(){
  if(drive_intake){
    set_intake(intake_speed,intake_state);
    drive_intake=false;
  }
  if(drive_wings){
    set_wings(wings_state);
    drive_wings=false;
  }
  if(drive_armer){
    set_armer(armer_state);
    drive_armer=false;
  }
}

void Control::cata_temp_watchdog_fn(){
  bool flag=false;
  while(true){
    flag=!flag;
    double temperature=std::max(this->catapult_motors[0].get_temperature(),this->catapult_motors[1].get_temperature());
    if(flag){
      master_controller.print(0, 0,"cata temp %lf",temperature);
    }else if(!isinf(temperature)){
      if(temperature>=54.9){
        master_controller.rumble(". . . .");
      }else if(temperature>=49.9){
        master_controller.rumble("- - - -");
      }
    }
    pros::delay(2000);
  }
}