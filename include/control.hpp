#pragma once
#include "api.h"

enum Control_State{
    INTAKE,
    OUTTAKE,
    STOP,
    ON,
    OFF
};
enum Lift_State{
    UP,
    MIDDLE,
    DOWN
};
class Control {
public:
    /**
     * \param intake_motor_ports ports of the intake motors (negative port will reverse it!)
     * \param intake_gearset gearset of the intake motors
     * \param lift_motor_port port of the lift motor (negative port will reverse it!)
     * \param lift_gearset gearset of the lift motor
     * \param lift_press_button_port port of the lift press button
     * \param wings_ports ports of the wings (negative port will reverse it!)
     * \param hanger_ports ports of the hanger (negative port will reverse it!)
    */
    Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const int8_t &lift_motor_port,
    pros::motor_gearset_e_t lift_gearset,const int8_t lift_press_button_port,const std::vector<int8_t> &wings_ports,
    const std::vector<int8_t> &hanger_ports);

    /**
     * \param state set the state of the intakes
     * \param speed speed of the intake motor
    */
    void set_intake(Control_State state,int speed);

    /**
     * \param speed speed of the lift motor
     * \param state set the state of the lift
    */
    void set_lift(int speed,Lift_State state=DOWN);

    /**
     * \param state set the state of the wings
    */
    void set_wings(Control_State state);

    /**
     * \param state set the state of the hanger
    */
    void set_hanger(Control_State state);

    /**
     *  \param 设置升降机在高处的位置
    */
    void set_lift_up_pos(double pos);
    
    /**
     * \param 设置升降机在中间的位置
    */
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
    std::array<bool,2> wings_reversed;
    std::array<bool,2> hanger_reversed;
};
