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
     * \param state 设置intake的模式
     * - INTAKE: 吸取
     * - OUTTAKE: 放出
     * \param speed 设置电机intake的速度
     * - -127~127
    */
    void set_intake(Control_State state,int speed);

    /**
     * \param speed 设置lift电机的速度
     * - -127~127
     * \param state 设置lift的模式，不填时默认为放下
     * - UP: 升起
     * - MIDDLE: 中间
     * - DOWN: 放下
    */
    void set_lift(int speed,Lift_State state=DOWN);

    /**
     * \param state 设置两侧挡板的状态
     * - ON: 打开挡板
     * - OFF: 关闭挡板
    */
    void set_wings(Control_State state);

    /**
     * \param state 设置钩子的状态
     *  -  ON: 放下钩子
     *  -  OFF: 收回钩子
    */
    void set_hanger(Control_State state);

    /**
     *  \param 设置升降机在升起时的位置
    */
    void set_lift_up_pos(double pos);
    
    /**
     * \param 设置升降机在中间时的位置
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

class Controller_Button_State{
public:
  inline static bool A_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_A);}
  inline static bool B_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_B);}
  inline static bool X_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_X);}
  inline static bool Y_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);}
  inline static bool R1_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);}
  inline static bool R2_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);}
  inline static bool L1_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);}
  inline static bool L2_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);}
  inline static bool UP_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);}
  inline static bool DOWN_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);}
  inline static bool LEFT_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);}
  inline static bool RIGHT_pressed(){return master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);}
};