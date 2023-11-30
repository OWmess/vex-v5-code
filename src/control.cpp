#include "control.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.h"

///********************************参数配置*******************************************///
/**
 * @brief 须在主控-角度传感器中设置发射架的最高点为0（SET ZERO）
 *        catapult 在中间和在底部时的角度值，该值可通过主控器直接观测得到，但若角度传感器正方向与投石机下压方向相反，
 *       则需取反后+360
          数值越大，发射架越往下
 *        
*/
#define CATAPULT_UP_POS     10.0
#define CATAPULT_MIDDLE_POS  44.0
#define CATAPULT_DOWN_POS    54.0


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
Catapult_State Control::catapult_state=MIDDLE;
Control_State Control::wings_state=OFF;
Control_State Control::armer_state=OFF;
Control::Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const int8_t &catapult_motor_port,
    pros::motor_gearset_e_t catapult_gearset,const int8_t catapult_rotation_port,const std::vector<int8_t> &wings_ports,
    const std::vector<int8_t> &armer_ports):task([this](){this->control_task();}),catapult_task([this](){this->catapult_task_func();}){
  //创建各个电机、电磁阀对象
  for(auto port:intake_motor_ports){
    pros::Motor temp{static_cast<int8_t>(abs(port)),intake_gearset,util::is_reversed(port)};
    intake_motors.push_back(temp);
    intake_motors.back().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  catapult_motor=std::make_unique<pros::Motor>(abs(catapult_motor_port),catapult_gearset,util::is_reversed(catapult_motor_port));
  catapult_motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cata_rotation=std::make_unique<pros::Rotation>(abs(catapult_rotation_port),ez::util::is_reversed(catapult_rotation_port));

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

  set_catapult_up_pos(CATAPULT_UP_POS);
  set_catapult_middle_pos(CATAPULT_MIDDLE_POS);
  set_catapult_down_pos(CATAPULT_DOWN_POS);
  // 配置catapult的pid参数
  cata_PID={CATA_KP,CATA_KI,CATA_KD,CATA_START_I};
  cata_PID.set_velocity_out(0.1);
  cata_PID.set_exit_condition(20, 1, 50, 1, 2000, 3000);

}


void Control::set_intake(int speed,Control_State state){
  if(state==STOP){
    for(const pros::Motor &intake_motor:intake_motors){
      intake_motor.brake();
    }
    return;
  }
  for(const pros::Motor &intake_motor:intake_motors){
    intake_motor.move(state==INTAKE?speed:-speed);
  }

}


void Control::set_catapult(int speed,Catapult_State state) {
  int cnt=0; 
  double start_t=pros::millis();
  time_out=5000;
  auto cata_to_top_lambda=[this,start_t,speed](float degree){
    this->catapult_motor->move(speed);
    while(pros::millis()-start_t<time_out) {
      ////检测是否有其他task通知catapult运动
      if(check_task_notify(catapult_task,1,cata_exit_condition)){
        this->catapult_motor->brake();
        return;
      }
      if(cata_rotation->get_velocity()<-1){//发射架在复位途中,忽略
        break;
      }
    }
    this->catapult_motor->brake();
    std::deque<float> angle_queue;
    while(true){
      if(check_task_notify(catapult_task,30,cata_exit_condition)){
        this->catapult_motor->brake();
        return;
      }
      float angle=cata_rotation->get_angle()/100.f;
      angle_queue.push_back(angle);
      if(angle_queue.size()>5){
        angle_queue.pop_front();
      }
      // cout<<"angle "<<angle<<endl;
      bool res=true;
      for_each(angle_queue.begin(),angle_queue.end(),[&res](float &angle){
        if(!(angle>355.f||angle<2.f)){
          res=false;
        }
      });
      if(res) break;
    }
  };
  
  auto cata_to_degree_lambda=[this,start_t,speed,state](float degree){
    //PID控制电机
    cata_PID.set_target(degree);
    while(pros::millis()-start_t<time_out){
      //检测是否有其他task通知catapult运动
      if(check_task_notify(catapult_task,10,cata_exit_condition)) {
        this->catapult_motor->brake();
        return;
      }
      auto cata_angle=cata_rotation->get_angle()/100.f;
      // if(cata_rotation->get_velocity()<-1){//发射架在复位途中,忽略
      //   catapult_motor->brake();
      //   continue;
      // }
      //防止因发射架挤压限位形变所导致的数值计算错误 
      if(cata_angle>350.f&&cata_angle<360.f){
        cata_angle=0.f;
      }
      double out=cata_PID.compute(cata_angle);
      // cout<<"PID error "<<cata_PID.error<<"  PID out "<<out<<endl;
      // cout<<"cata angle "<<cata_angle<<endl;
      out=util::clip_num(out, speed, 0);
      catapult_motor->move(out);
      //当pid输出<=0时，则说明已运动到目标位置或已越过目标位置，但由于棘轮的存在无法回到目标位置，此时退出循环
      auto exit_condition=cata_PID.exit_condition(*catapult_motor.get());
      if(exit_condition!=RUNNING||out<=0||cata_angle>=degree){
        std::cout<<"exit condition: "<<exit_to_string(exit_condition)<<std::endl;
        break;
      }
    }
    cata_exit_condition=false;
    catapult_motor->brake();
  };

  if(state==BRAKE){
    catapult_motor->brake();
  }else if(state==DOWN){
    if(cata_rotation->get_angle()/100.f>=catapult_down_pos-5||cata_exit_condition){
      cata_to_top_lambda(catapult_down_pos);
    }
    cata_to_degree_lambda(catapult_down_pos);
  }else if(state==MIDDLE){
    if(cata_rotation->get_angle()/100.f>=catapult_middle_pos-5||cata_exit_condition){
      cata_to_top_lambda(catapult_down_pos);
    }
    cata_to_degree_lambda(catapult_middle_pos);
  }else if(state==UP){
    cata_to_top_lambda(catapult_down_pos);
  }else if(state==INIT_MIDDLE){
    cata_to_degree_lambda(catapult_middle_pos);
  }else if(state==INIT_DOWN){
    cata_to_degree_lambda(catapult_down_pos);
  }

}

void Control::set_wings(Control_State state){
  for(const auto &wing:wings){
    wing.pneumatics->set_value(state==ON?!wing.reversed:wing.reversed);
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

void Control::catapult_task_func(){
  while (true) {
    // block for up to 50ms waiting for a notification and clear the value
    auto t=catapult_task.notify_take(true, 50);
    if(t){
      set_intake(0,STOP);
      set_catapult(catapult_speed,catapult_state);
      set_intake(intake_speed, intake_state);
    }
    // no need to delay here because the call to notify_take blocks
  }

}


void Control::control_task(){
  int cnt=0;
  while(true){
    if(drive_intake){
      set_intake(intake_speed,intake_state);
      drive_intake=false;
    }
    if(drive_catapult){
      //通知发射架运动
      catapult_task.notify();
      drive_catapult=false;
    }
    if(drive_wings){
      set_wings(wings_state);
      drive_wings=false;
    }
    if(drive_armer){
      set_armer(armer_state);

      drive_armer=false;
    }
    auto cata_angle=cata_rotation->get_angle()/100.f;
    if(cata_angle<catapult_middle_pos-5||(cata_angle>350.f&&cata_angle<360.f)){
      set_intake(0, STOP);
    }else{
      set_intake(intake_speed, intake_state);
    }
    double temperature=catapult_motor->get_temperature();
    master.print(0, 0,"cata temp %lf",temperature);
    pros::delay(50);
  }
}

void Control::with_pto(){
  
}
