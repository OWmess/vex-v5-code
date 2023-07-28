#include "main.h"


/**
 * \param state true for intake, false for outtake
 * \param speed speed of the intake
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
  while(true) {
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
  static pros::ADIDigitalOut wings('B');
  wings.set_value(state);
}

void set_hanger(bool state){
  static pros::ADIDigitalOut hanger('A');
  hanger.set_value(state);
}