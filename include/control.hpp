#pragma once
#include "api.h"
enum Control_State{
    INTAKE,
    OUTTAKE,
    STOP,
    ON,
    OFF,
};
enum Lift_State{
    UP,
    MIDDLE,
    DOWN,
    STOP,
};
class Control {
public:
    Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const int8_t &lift_motor_port,
    pros::motor_gearset_e_t lift_gearset,const int8_t lift_press_button_port,const std::vector<int8_t> &wings_ports,
    const std::vector<int8_t> &hanger_ports);
    /**
     * \param state set the state of the intakes
     * \param speed speed of the intakes
    */
    void set_intake(Control_State state,int speed);
    void set_lift(int speed,Lift_State state=DOWN);
    void set_wings(Control_State state);
    void set_hanger(Control_State state);

    void set_lift_up_pos(double pos);

    void set_lift_middle_pos(double pos);
private:
    std::vector<pros::Motor> intake_motors;
    std::shared_ptr<pros::Motor> lift_motor;
    std::shared_ptr<pros::ADIDigitalOut> wings_l;
    std::shared_ptr<pros::ADIDigitalOut> wings_r;
    std::shared_ptr<pros::ADIDigitalOut> hanger_arm;
    std::shared_ptr<pros::ADIDigitalOut> hanger_claw;
    std::shared_ptr<pros::ADIDigitalIn> lift_press_button;
    double lift_up_pos;
    double lift_middle_pos;
};
