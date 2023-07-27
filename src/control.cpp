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
  pros::delay(25);
  while(!press_buttion.get_value()){
    printf("press_buttion.get_value()=%d\n",press_buttion.get_value());
    pros::delay(ez::util::DELAY_TIME);
  }
  lift_motor.move(0);
    lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_INVALID);
  return;

}

void set_piston(bool state){
  static pros::ADIDigitalOut piston('A');
  piston.set_value(state);
}