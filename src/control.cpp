#include "main.h"


Control::Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const int8_t &lift_motor_port,
    pros::motor_gearset_e_t lift_gearset,const int8_t lift_press_button_port,const std::vector<int8_t> &wings_ports,
    const std::vector<int8_t> &hanger_ports){
  
  for(auto port:intake_motor_ports){
    pros::Motor temp{static_cast<int8_t>(abs(port)),intake_gearset,util::is_reversed(port)};
    intake_motors.push_back(temp);
  }

  lift_motor=std::make_shared<pros::Motor>(abs(lift_motor_port),lift_gearset,util::is_reversed(lift_motor_port));
  lift_press_button=std::make_shared<pros::ADIDigitalIn>(lift_press_button_port);
  wings_l=std::make_shared<pros::ADIDigitalOut>(abs(wings_ports[0]),util::is_reversed(wings_ports[0]));
  wings_r=std::make_shared<pros::ADIDigitalOut>(abs(wings_ports[1]),util::is_reversed(wings_ports[1]));
  hanger_arm=std::make_shared<pros::ADIDigitalOut>(abs(hanger_ports[0]),util::is_reversed(hanger_ports[0]));
  hanger_claw=std::make_shared<pros::ADIDigitalOut>(abs(hanger_ports[1]),util::is_reversed(hanger_ports[1]));

}


/**
 * \param state true for intake, false for outtake
 * \param speed speed of the intakes
*/
void Control::set_intake(Control_State state,int speed){
  if(state==STOP){
    for(auto &intake_motor:intake_motors){
      intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      intake_motor.brake();
    }
    return;
  }
  for(auto &intake_motor:intake_motors){
    intake_motor.move(state==INTAKE?speed:-speed);
  }
}

/**
 * \param state true for lift up, false for lift down
 * \param speed speed of the lift
*/
void Control::set_lift(int speed){
  // static pros::Motor lift_motor(10,pros::E_MOTOR_GEAR_200);
  // static pros::ADIDigitalIn press_buttion('H');
  lift_motor->move(speed);
  pros::delay(100);
  int cnt=0;
  auto start_t=pros::millis();
  while(true) {
    if(pros::millis()-start_t>3000){
      printf("lift time out\n");
      break;
    }
    if(lift_press_button->get_value()){
      printf("press_button.get_value()=%d\n",cnt);
      cnt++;
      break;
    }
    if(cnt>=2)
      break;
    
    pros::delay(1);
  }
  // lift_motor.move(0);
  lift_motor->brake();
  lift_motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  return;

}

void Control::set_wings(Control_State state){
  // static pros::ADIDigitalOut wings1('C');
  // static pros::ADIDigitalOut wings2('D');
  wings_l->set_value(state==ON?true:false);
  wings_r->set_value(state==ON?true:false);
}

void Control::set_hanger(Control_State state){
  // static pros::ADIDigitalOut hanger1('A');
  // static pros::ADIDigitalOut hanger2('B');
  hanger_arm->set_value(state==ON?true:false);
  hanger_claw->set_value(state==ON?true:false);
}
