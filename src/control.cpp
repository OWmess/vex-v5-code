#include "main.h"

/**
 * @brief catapult 在中间和在底部时的角度值，该值可通过主控器直接观测得到，但若角度传感器正方向与投石机下压方向相反，
 *       则需取反后+360
 *        
*/
#define CATAPULT_MIDDLE_POS  360.0-295.0
#define CATAPULT_DOWN_POS    360.0-287.0

Control_State Control::intake_state=INTAKE;
Catapult_State Control::catapult_state=MIDDLE;
Control_State Control::wings_state=OFF;
Control_State Control::hanger_state=OFF;
Control::Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const int8_t &catapult_motor_port,
    pros::motor_gearset_e_t catapult_gearset,const int8_t catapult_rotation_port,const std::vector<int8_t> &wings_ports,
    const int8_t &hanger_port):task([this](){this->control_task();}),catapult_task([this](){this->catapult_task_func();}){
  //创建各个电机、电磁阀对象
  for(auto port:intake_motor_ports){
    pros::Motor temp{static_cast<int8_t>(abs(port)),intake_gearset,util::is_reversed(port)};
    intake_motors.push_back(temp);
    intake_motors.back().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }

  catapult_motor=std::make_shared<pros::Motor>(abs(catapult_motor_port),catapult_gearset,util::is_reversed(catapult_motor_port));
  catapult_motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cata_rotation=std::make_shared<pros::Rotation>(abs(catapult_rotation_port),ez::util::is_reversed(catapult_rotation_port));

  wings_l=std::make_shared<pros::ADIDigitalOut>(abs(wings_ports[0]),util::is_reversed(wings_ports[0]));
  wings_r=std::make_shared<pros::ADIDigitalOut>(abs(wings_ports[1]),util::is_reversed(wings_ports[1]));
  hanger=std::make_shared<pros::ADIDigitalOut>(abs(hanger_port),util::is_reversed(hanger_port));

  wings_reversed[0]=util::is_reversed(wings_ports[0]);
  wings_reversed[1]=util::is_reversed(wings_ports[1]);
  hanger_reversed=util::is_reversed(hanger_port);

  
  set_catapult_middle_pos(CATAPULT_MIDDLE_POS);
  set_catapult_down_pos(CATAPULT_DOWN_POS);
  cata_PID={10,0,25,0};
  cata_PID.set_exit_condition(20, 1, 50, 3, 500, 3000);
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
  auto cata_to_degree_lambda=[this,start_t,speed](float degree){
    catapult_motor->move(speed);
    //走过一段距离防止不运动
    while(pros::millis()-start_t<time_out&&abs(cata_rotation->get_angle()/100.f-degree)<3.5){
      pros::delay(10);
    }

    //PID控制电机
    cata_PID.set_target(degree);
    while(pros::millis()-start_t<time_out){
      double out=cata_PID.compute(cata_rotation->get_angle()/100.f);
      out=util::clip_num(out, speed, 0);
      if(out!=120.f){
        std::cout<<"cata_out "<<out<<std::endl;
      }
      catapult_motor->move(out);
      //当pid输出<=0时，则说明已运动到目标位置或已越过目标位置，但由于棘轮的存在无法回到目标位置，此时退出循环
      if(cata_PID.exit_condition(*catapult_motor.get())!=RUNNING||out<=0){
        break;
      }
      pros::delay(5);
    }
    catapult_motor->brake();
  };
  catapult_motor->move(speed);
  pros::delay(300);
  if(state==BRAKE){
    catapult_motor->brake();
  }else if(state==DOWN){
    cata_to_degree_lambda(catapult_down_pos);
  }else if(state==MIDDLE){
    cata_to_degree_lambda(catapult_middle_pos);
  }else if(state==UP){
    cata_to_degree_lambda(catapult_up_pos);
  }

}

void Control::set_wings(Control_State state){
  wings_l->set_value(state==ON?!wings_reversed[0]:wings_reversed[0]);
  wings_r->set_value(state==ON?!wings_reversed[1]:wings_reversed[1]);
}

void Control::set_hanger(Control_State state){
  hanger->set_value(state==ON?!hanger_reversed:hanger_reversed);
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
  int cnt=0;
  while (true) {
    
    // block for up to 50ms waiting for a notification and clear the value
    if(pros::Task::notify_take(true, 50)){
      set_catapult(catapult_speed,catapult_state);
      std::cout<<"catapult task cnt "<<++cnt<<std::endl;
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
      catapult_task.notify();
      drive_catapult=false;
      std::cout<<"catapult task notify "<<++cnt<<std::endl;
      
    }
    if(drive_wings){
      set_wings(wings_state);
      drive_wings=false;
    }
    if(drive_hanger){
      set_hanger(hanger_state);
      drive_hanger=false;
    }

    pros::delay(50);
  }

}


