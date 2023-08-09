#include "main.h"

/**
 * @brief 投球机位于顶部和中间位置时的编码器值，编码器0点为底部
*/
#define LIFT_UP_POS      100.0 
#define LIFT_MIDDLE_POS  1600.0 


Control::Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const int8_t &lift_motor_port,
    pros::motor_gearset_e_t lift_gearset,const int8_t lift_press_button_port,const std::vector<int8_t> &wings_ports,
    const std::vector<int8_t> &hanger_ports):task([this](){this->control_task();}){
  
  for(auto port:intake_motor_ports){
    pros::Motor temp{static_cast<int8_t>(abs(port)),intake_gearset,util::is_reversed(port)};
    intake_motors.push_back(temp);
    intake_motors.back().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }

  lift_motor=std::make_shared<pros::Motor>(abs(lift_motor_port),lift_gearset,util::is_reversed(lift_motor_port));
  lift_motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lift_press_button=std::make_shared<pros::ADIDigitalIn>(lift_press_button_port);

  wings_l=std::make_shared<pros::ADIDigitalOut>(abs(wings_ports[0]),util::is_reversed(wings_ports[0]));
  wings_r=std::make_shared<pros::ADIDigitalOut>(abs(wings_ports[1]),util::is_reversed(wings_ports[1]));
  hanger_arm=std::make_shared<pros::ADIDigitalOut>(abs(hanger_ports[0]),util::is_reversed(hanger_ports[0]));
  hanger_claw=std::make_shared<pros::ADIDigitalOut>(abs(hanger_ports[1]),util::is_reversed(hanger_ports[1]));

  wings_reversed[0]=util::is_reversed(wings_ports[0]);
  wings_reversed[1]=util::is_reversed(wings_ports[1]);
  hanger_reversed[0]=util::is_reversed(hanger_ports[0]);
  hanger_reversed[1]=util::is_reversed(hanger_ports[1]);

  set_lift_up_pos(LIFT_UP_POS);
  set_lift_middle_pos(LIFT_MIDDLE_POS);
  
}


void Control::set_intake(Control_State state,int speed){
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


void Control::set_lift(int speed,Lift_State state) {
  lift_motor->move(speed);
  pros::delay(100);
  int cnt=0;
  double start_t=pros::millis();
  while(false) {
    if(pros::millis()-start_t>3000){
      printf("lift time out\n");
      break;
    }

    if(lift_press_button->get_value()){
      cnt++;
      break;
    }
    if(cnt>=2)
      break;
    pros::delay(1);
  }
  lift_motor->tare_position();
  if(state==UP){
    lift_motor->move_relative(lift_up_pos,speed);
  }else if(state==MIDDLE){
    lift_motor->move_relative(lift_middle_pos,speed);
  }
  lift_motor->brake();
  return;

}

void Control::set_wings(Control_State state){
  wings_l->set_value(state==ON?!wings_reversed[0]:wings_reversed[0]);
  wings_r->set_value(state==ON?!wings_reversed[1]:wings_reversed[1]);
}

void Control::set_hanger(Control_State state){
  hanger_arm->set_value(state==ON?!hanger_reversed[0]:hanger_reversed[0]);
  hanger_claw->set_value(state==ON?!hanger_reversed[1]:hanger_reversed[1]);
}

void Control::set_lift_up_pos(double pos){
  lift_up_pos=pos;
}

void Control::set_lift_middle_pos(double pos){
  lift_middle_pos=pos;
}

void Control::control_task(){
  static Control_State last_intake_state=intake_state;
  static Lift_State last_lift_state=lift_state;
  static Control_State last_wings_state=wings_state;
  while(true){
    if(intake_state!=last_intake_state){
      set_intake(intake_state,100);
      last_intake_state=intake_state;
    }
    if(lift_state!=last_lift_state){
      set_lift(100,lift_state);
      last_lift_state=lift_state;
    }
    if(wings_state!=last_wings_state){
      set_wings(wings_state);
      last_wings_state=wings_state;
    }

    pros::delay(ez::util::DELAY_TIME);
  }

}

Control_State Control::reverse_intake(Control_State loggle){
  return loggle==INTAKE?OUTTAKE:INTAKE;
}