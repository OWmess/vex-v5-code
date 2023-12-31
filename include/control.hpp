#pragma once
#include <memory>
#include "EZ-Template/util.hpp"
#include "api.h"
#include "EZ-Template/api.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

extern Drive chassis;
enum Control_State{
    INTAKE,
    OUTTAKE,
    STOP,
    ON,
    OFF
};

inline Control_State operator!(Control_State state) {
    switch (state) {
        case ON:
            return OFF;
        case OFF:
            return ON;
        case INTAKE:
            return OUTTAKE;
        case OUTTAKE:
            return INTAKE;
        default:
            return state;
    }
}

enum Catapult_State{
    UP,
    MIDDLE,
    DOWN,
    BRAKE,
    INIT_MIDDLE,
    INIT_DOWN

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
     * \param armer_ports ports of the armer (negative port will reverse it!)
    */
    Control(const std::vector<int8_t> &intake_motor_ports,pros::motor_gearset_e_t intake_gearset,const std::vector<int8_t> &catapult_motor_port,
    pros::motor_gearset_e_t catapult_gearset,const int8_t catapult_rotation_port,const std::vector<int8_t> &wings_ports,
    const std::vector<int8_t> &armer_ports);

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
     * \param state 设置armer的模式
     * - ON: 打开
     * - OFF: 关闭
    */
    inline void set_armer_state(Control_State state){
        armer_state=state;
        drive_armer=true;
    }
    /**
     * \param speed 设置intake的速度,默认值为100
     * - -127~127
    */
    inline void set_intake_speed(int speed=100){
        intake_speed=speed;
    }

    /**
     * \param speed 设置catapult的速度,默认值为120
     * - -127~127
    */
    inline void set_catapult_speed(int speed=120){
        catapult_speed=speed;
    }
    /**
     * \return 返回armer的当前模式
    */
    inline Control_State get_armer_state(){
        return armer_state;
    }

    /**
     * \param time 设置投石机的超时时间,默认值为2000ms
    */
    inline void set_time_out(int time=2000){
        time_out=time;
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
    
    /**
     * @brief 设置intake电机的制动模式
     * 
     * @param mode 
     */
    inline void set_intake_brake_mode(const pros::motor_brake_mode_e_t mode) const{
        for(const auto& motor:intake_motors){
            motor.set_brake_mode(mode);
        }
    }

    /**
     * @brief 设置发射架电机的制动模式
     * 
     * @param mode 
     */
    inline void set_catapult_brake_mode(const pros::motor_brake_mode_e_t mode) const{
        for(auto &i:catapult_motor){
            i.set_brake_mode(mode);
        }
    }

    /**
     * @brief 设置发射架的PID参数
     * 
     * @param pid 
     * @param p 
     * @param i 
     * @param d 
     * @param p_start_i 
     */
    inline void set_pid_constants(PID *pid, double p, double i, double d, double p_start_i){
        pid->set_constants(p,i,d,p_start_i);
    }

    /**
     * @brief 获取发射架电机的引用
     * 
     * @return pros::Motor& 
     */
    inline std::vector<pros::Motor> get_catapult_motor(){
        return catapult_motor;
    }

    inline void clean_cata_task_notify(){
        catapult_task.notify_clear();
    }


    inline void cata_move(std::int32_t voltage) const{
        for(const auto &motor:catapult_motor){
            motor.move(voltage);
        }
    }

    inline void cata_brake() const{
        for(const auto &motor:catapult_motor){
            motor.brake();
        }
    }
    
    void pto_chassis_mode(){
        chassis_piston->set_value(LOW);
        // arm_piston->set_value(LOW);
        chassis.pto_toggle(false);
        for(auto &i:chassis.pto_active) {
            cout<<i<<", ";
        }
        set_catapult_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        cout<<"\n";
    }

    void pto_cata_mode(){
        chassis_piston->set_value(HIGH);
        // arm_piston->set_value(LOW);
        chassis.pto_toggle(true);
        for(auto &i:chassis.pto_active){
            cout<<i<<", ";
        }
        cout<<"\n";
        set_catapult_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    }

    void pto_arm_mode(){
        chassis_piston->set_value(HIGH);
        // arm_piston->set_value(HIGH);
        chassis.pto_toggle(true);
        for(auto &i:chassis.pto_active){
            cout<<i<<", ";
        }
        cout<<"\n";
        pros::Task thread([this](){
            cata_move(80);
            pros::delay(300);
            cata_brake();
        });
        set_catapult_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        
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
    void set_armer(Control_State state);

    /**
     * \brief 维护上层机构的task
    */
    void control_task();

    /**
     * \brief 维护投石机的task
    */
    void catapult_task_func();

    void with_pto();


public:
    PID cata_PID;
    std::unique_ptr<pros::ADIDigitalOut> chassis_piston;
    std::unique_ptr<pros::ADIDigitalOut> arm_piston;
    std::unique_ptr<pros::ADIDigitalOut> armlock_piston;
private:
    //气动结构体
    struct PneumaticsStruct{
        std::shared_ptr<pros::ADIDigitalOut> pneumatics;
        bool reversed;

    };
    //电机及电磁阀的智能指针或实例
    std::unique_ptr<pros::Rotation> cata_rotation;
    std::vector<pros::Motor> intake_motors;
    std::vector<pros::Motor> catapult_motor;
    std::vector<PneumaticsStruct> wings;
    std::vector<PneumaticsStruct> armers;
    std::map<std::string,std::unique_ptr<PneumaticsStruct>> pto;
    double catapult_up_pos;
    double catapult_middle_pos;
    double catapult_down_pos;
    bool armer_reversed;
    pros::Task task;
    pros::Task catapult_task;
    bool cata_exit_condition=false;
    bool drive_catapult;
    bool drive_intake;
    bool drive_wings;
    bool drive_armer;
    int intake_speed=120;
    int catapult_speed=120;
    int time_out=2000;
    static Control_State intake_state;
    static Control_State wings_state;
    static Catapult_State catapult_state;
    static Control_State armer_state;
    bool enable_pto=false;
    
    //pto

};


class Controller_Button_State{
public:
  inline static bool A_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);}
  inline static bool B_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);}
  inline static bool X_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);}
  inline static bool Y_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);}
  inline static bool R1_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);}
  inline static bool R2_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2);}
  inline static bool L1_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);}
  inline static bool L2_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);}
  inline static bool UP_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);}
  inline static bool DOWN_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);}
  inline static bool LEFT_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);}
  inline static bool RIGHT_new_press(){return master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);}

    inline static bool A_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);}
  inline static bool B_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);}
  inline static bool X_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);}
  inline static bool Y_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);}
  inline static bool R1_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);}
  inline static bool R2_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);}
  inline static bool L1_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);}
  inline static bool L2_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);}
  inline static bool UP_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);}
  inline static bool DOWN_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);}
  inline static bool LEFT_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);}
  inline static bool RIGHT_pressed(){return master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);}
};