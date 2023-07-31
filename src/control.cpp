#include "main.h"


/**
 * \param state true for intake, false for outtake
 * \param speed speed of the intakes
*/
void set_intake(bool state,int speed){
  static pros::MotorGroup intake_motors({pros::Motor{9,pros::E_MOTOR_GEAR_200},pros::Motor{-19,pros::E_MOTOR_GEAR_200}});
  intake_motors.move(state?speed:-speed);
}
/**
 * \param state true for lift up, false for lift down
 * \param speed speed of the lift
*/
void set_lift(bool state,int speed){
  static pros::Motor lift_motor(10,pros::E_MOTOR_GEAR_200);
  static pros::ADIDigitalIn press_buttion('H');
  lift_motor.move(state?speed:-speed);
  pros::delay(100);
  int cnt=0;
  auto start_t=pros::millis();
  while(true) {
    if(pros::millis()-start_t>3000){
      printf("lift time out\n");
      break;
    }
    printf("press_buttion.get_value()=%d\n",press_buttion.get_value());
    if(press_buttion.get_value()){
      printf("press_buttion.get_value()=%d\n",cnt);
      cnt++;
      break;
    }
    if(cnt>=2)
      break;
    
    pros::delay(1);
  }
  // lift_motor.move(0);
  lift_motor.brake();
    lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  return;

}

void set_wings(bool state){
  static pros::ADIDigitalOut wings1('C');
  static pros::ADIDigitalOut wings2('D');
  wings1.set_value(!state);
  wings2.set_value(state);
}

void set_hanger(bool state){
  static pros::ADIDigitalOut hanger1('A');
  static pros::ADIDigitalOut hanger2('B');

  hanger1.set_value(state);
  hanger2.set_value(!state);
}

void set_lift_mid(){
  pros::Motor lift(10,pros::E_MOTOR_GEAR_200);
  set_lift(true,100);
  lift.tare_position();
  lift.move_absolute(1500,100);//固定拍子角度
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}