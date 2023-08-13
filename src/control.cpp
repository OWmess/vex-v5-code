#include "main.h"

/**
 * @brief 投球机位于顶部和中间位置时的编码器值，编码器0点为底部
*/
#define CATAPULT_UP_POS      500.0 
#define CATAPULT_MIDDLE_POS  1500.0 
#define CATAPULT_DOWN_POS    0.0

Control_State Control::intake_state=INTAKE;
Catapult_State Control::catapult_state=MIDDLE;
Control_State Control::wings_state=OFF;
Control_State Control::hanger_state=OFF;
Control::Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const int8_t &catapult_motor_port,
    pros::motor_gearset_e_t catapult_gearset,const int8_t catapult_press_button_port,const std::vector<int8_t> &wings_ports,
    const int8_t &hanger_port):task([this](){this->control_task();}){
  //创建各个电机、电磁阀对象
  for(auto port:intake_motor_ports){
    pros::Motor temp{static_cast<int8_t>(abs(port)),intake_gearset,util::is_reversed(port)};
    intake_motors.push_back(temp);
    intake_motors.back().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }

  catapult_motor=std::make_shared<pros::Motor>(abs(catapult_motor_port),catapult_gearset,util::is_reversed(catapult_motor_port));
  catapult_motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  catapult_press_button=std::make_shared<pros::ADIDigitalIn>(catapult_press_button_port);

  wings_l=std::make_shared<pros::ADIDigitalOut>(abs(wings_ports[0]),util::is_reversed(wings_ports[0]));
  wings_r=std::make_shared<pros::ADIDigitalOut>(abs(wings_ports[1]),util::is_reversed(wings_ports[1]));
  hanger=std::make_shared<pros::ADIDigitalOut>(abs(hanger_port),util::is_reversed(hanger_port));

  wings_reversed[0]=util::is_reversed(wings_ports[0]);
  wings_reversed[1]=util::is_reversed(wings_ports[1]);
  hanger_reversed=util::is_reversed(hanger_port);

  set_catapult_up_pos(CATAPULT_UP_POS);
  set_catapult_middle_pos(CATAPULT_MIDDLE_POS);
  set_catapult_down_pos(CATAPULT_DOWN_POS);
}


void Control::set_intake(int speed,Control_State state){
  if(state==STOP){
    for(pros::Motor &intake_motor:intake_motors){
      intake_motor.brake();
    }
    return;
  }
  for(pros::Motor &intake_motor:intake_motors){
    intake_motor.move(state==INTAKE?speed:-speed);
  }

}


void Control::set_catapult(int speed,Catapult_State state) {
  int cnt=0;
  double start_t=pros::millis();
  //lambda函数，用于等待弹射机构按键松开
  auto wait_until_not_pressed=[this,start_t](){
    while(catapult_press_button->get_value()){
      if(pros::millis()-start_t>time_out){//超时
        printf("catapult time out\n");
        break;
      }
      pros::delay(1);
    }
  };
  //lambda函数，用于等待弹射机构按键按下
  auto wait_until_pressed=[this,start_t](){
    int cnt=0;
    while(true) {
      if(pros::millis()-start_t>time_out){//超时
        printf("catapult time out\n");
        break;
      }
      if(catapult_press_button->get_value()){
        cnt++;
      }
      if(cnt>=1)
        break;
      pros::delay(1);
    }
  };

  catapult_motor->move(speed);
  if(state==BRAKE){
    catapult_motor->brake();
  }else if(state==DOWN){
    wait_until_not_pressed();
    wait_until_pressed();
    if(catapult_down_pos!=0)
      catapult_motor->move_relative(catapult_down_pos,speed);
    else
      catapult_motor->brake();
  }else if(state==MIDDLE){
    if(catapult_press_button->get_value()){
      wait_until_not_pressed();
      catapult_motor->move_relative(catapult_middle_pos,speed);
    }else{
      wait_until_pressed();
      wait_until_not_pressed();
      catapult_motor->move_relative(catapult_middle_pos,speed);
    }
  }else if(state==UP){
    if(catapult_press_button->get_value()){
      wait_until_not_pressed();
      catapult_motor->move_relative(catapult_up_pos,speed);
    }else{
      wait_until_pressed();
      wait_until_not_pressed();
      catapult_motor->move_relative(catapult_up_pos,speed);
    }
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


void Control::control_task(){
  while(true){
    if(drive_intake){
      set_intake(intake_speed,intake_state);
      drive_intake=false;
    }
    if(drive_catapult){
      set_catapult(catapult_speed,catapult_state);
      drive_catapult=false;
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


