#pragma once
#include "api.h"

enum Control_State{
    INTAKE,
    OUTTAKE,
    STOP,
    ON,
    OFF
};
enum Catapult_State{
    UP,
    MIDDLE,
    DOWN,
    BRAKE
};
class Control {
public:
    /**
     * \param intake_motor_ports ports of the intake motors (negative port will reverse it!)
     * \param intake_gearset gearset of the intake motors
     * \param catapult_motor_port port of the catapult motor (negative port will reverse it!)
     * \param catapult_gearset gearset of the catapult motor
     * \param catapult_press_button_port port of the catapult press button
     * \param wings_ports ports of the wings (negative port will reverse it!)
     * \param hanger_ports ports of the hanger (negative port will reverse it!)
    */
    Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const int8_t &catapult_motor_port,
    pros::motor_gearset_e_t catapult_gearset,const int8_t catapult_press_button_port,const std::vector<int8_t> &wings_ports,
    const int8_t &hanger_ports);

    /**
     * \param 设置投石机在下方的位置
    */
   void set_catapult_down_pos(double pos);

    /**
     *  \param 设置投石机在升起时的位置
    */
    void set_catapult_up_pos(double pos);
    
    /**
     * \param 设置投石机在中间时的位置
    */
    void set_catapult_middle_pos(double pos);

    inline static Control_State reverse_intake(Control_State loggle){
        return loggle==INTAKE?OUTTAKE:INTAKE;
    }
    /**
     * \param state 设置intake的模式
     * - INTAKE: 吸取
     * - OUTTAKE: 放出
    */
    inline void set_intake_state(Control_State state){
        intake_state=state;
        drive_intake=true;
    }
    /**
     * \param state 设置wings的模式
     * - ON: 放出
     * - OFF: 收起
    */
    inline void set_wings_state(Control_State state){
        wings_state=state;
        drive_wings=true;
    }
    /**
     * \param state 设置catapult的模式
     * - UP: 升起
     * - MIDDLE: 中间
     * - DOWN: 放下
    */
    inline void set_catapult_state(Catapult_State state){
        catapult_state=state;
        drive_catapult=true;
    }

    /**
     * \param state 设置hanger的模式
     * - ON: 打开
     * - OFF: 关闭
    */
    inline void set_hanger_state(Control_State state){
        hanger_state=state;
        drive_hanger=true;
    }


    /**
     * \return 返回hanger的当前模式
    */
    inline Control_State get_hanger_state(){
        return hanger_state;
    }

    /**
     * \return 返回intake的当前模式
    */
    inline Control_State get_intake_state(){
        return intake_state;
    }
    /**
     * \return 返回wings的当前模式
    */
    inline Control_State get_wings_state(){
        return wings_state;
    }
    /**
     * \return 返回catapult的当前模式
    */
    inline Catapult_State get_catapult_state(){
        return catapult_state;
    }
private:

    /**
     * \param speed 设置电机intake的速度
     * - -127~127
     * 
     * \param state 设置intake的模式
     * - INTAKE: 吸取
     * - OUTTAKE: 放出
    */
    void set_intake(int speed,Control_State state);

    /**
     * \param speed 设置catapult电机的速度
     * - -127~127
     * \param state 设置catapult的模式，不填时默认为放下
     * - UP: 升起
     * - MIDDLE: 中间
     * - DOWN: 放下
    */
    void set_catapult(int speed,Catapult_State state=DOWN);

    /**
     * \param state 设置两侧挡板的状态
     * - ON: 打开挡板
     * - OFF: 关闭挡板
    */
    void set_wings(Control_State state);

    /**
     * \param state 设置侧面勾爪的状态
     * - ON: 打开勾爪
     * - OFF: 关闭勾爪
    */
    void set_hanger(Control_State state);

    /**
     * \brief 维护上层机构的task
    */
    void control_task();

public:

private:
    std::vector<pros::Motor> intake_motors;
    std::shared_ptr<pros::Motor> catapult_motor;
    std::shared_ptr<pros::ADIDigitalOut> wings_l;
    std::shared_ptr<pros::ADIDigitalOut> wings_r;
    std::shared_ptr<pros::ADIDigitalOut> hanger;
    std::shared_ptr<pros::ADIDigitalIn> catapult_press_button;
    double catapult_up_pos;
    double catapult_middle_pos;
    double catapult_down_pos;
    std::array<bool,2> wings_reversed;
    bool hanger_reversed;
    pros::Task task;

    static Control_State intake_state;
    static Control_State wings_state;
    static Catapult_State catapult_state;
    static Control_State hanger_state;
    bool drive_catapult;
    bool drive_intake;
    bool drive_wings;
    bool drive_hanger;
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