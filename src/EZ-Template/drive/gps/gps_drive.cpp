#include "EZ-Template/drive/gps/gps_drive.hpp"
#include "EZ-Template/util.hpp"
#include "control.hpp"
#include "gps_pid.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

#define HEADING_THRESH 0.1
Gps_Drive::Gps_Drive(Drive &drive_chassis,const std::uint8_t port, double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset):
    gps_sensor(port,xInitial,yInitial,headingInitial,xOffset,yOffset),chassis_reference(drive_chassis),
    gps_task([this](){this->gps_task_func();})
    {
    gps_sensor.set_data_rate(10);

    int n = 6;
    int m = 1;

    double dt = 10.0/1000; // 测量频率

    Eigen::MatrixXd F(n, n); // 状态转移矩阵
    Eigen::MatrixXd H(2, 6); // 观测矩阵
    Eigen::MatrixXd Q(n, n); // 过程噪声协方差
    Eigen::MatrixXd R(2, 2); // 测量噪声协方差
    Eigen::MatrixXd P(n, n); // 估计误差协方差


    /**
     * 状态向量 X(1,6)：[x, x', x'', y, y', y'']
     */
    /**
     *  F:状态转移矩阵
     */
    F << 1, dt, 0.5 * pow(dt, 2), 0, 0, 0,
        0, 1, dt, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, dt, 0.5 * pow(dt, 2),
        0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 1;
    /**
     * H:观测矩阵
     */
    H << 1, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0;

    // 估计过程噪声协方差
    Q << pow(dt, 4) / 4.f, pow(dt, 3) / 2.f, pow(dt, 2) / 2.f, 0, 0, 0,
            pow(dt, 3) / 2.f, pow(dt, 2), dt, 0, 0, 0,
            pow(dt, 2) / 2.f, dt, 1, 0, 0, 0,
            0, 0, 0, pow(dt, 4) / 4.f, pow(dt, 3) / 2.f, pow(dt, 2) / 2.f,
            0, 0, 0, pow(dt, 3) / 2.f, pow(dt, 2), dt,
            0, 0, 0, pow(dt, 2) / 2.f, dt, 1;
    Q=Q*0.35;//乘上加速度方差
    // 测量噪声协方差
    R << 0.005,0,
         0,0.005;
    // 估计误差协方差
    P << .05, .05, .05,0 ,0 ,0 ,
        .05, .05, .05,0 ,0 ,0 ,
        .05, .05, .05,0 ,0 ,0 ,
        0, 0, 0,.05, .05, .05,
        0, 0, 0,.05, .05, .05,
        0, 0, 0,.05, .05, .05;

    kf=KalmanFilter{dt, F, H, Q, R, P};

    // 估计初始状态
    Eigen::VectorXd x0(n);
    double t = 0;
    x0 << xInitial, 0, 0, yInitial, 0, 0;
    kf.init(t, x0);

    auto [p,i,d,start_i] =chassis_reference.turnPID.get_constants();
    // set_pid_contants(&turn_PID, p, i, d, start_i);
    set_pid_contants(&turn_PID, 3, 0.003, 20, 10);
    
    set_pid_contants(&straight_PID, 300,0,600,0);
    set_pid_contants(&heading_PID, 4, 0.000, 10,0);
    turn_PID.set_exit_condition( 100, 1, 500, 3, 50000, 5000);
    straight_PID.set_exit_condition(80, 0.01, 300, 0.02, 50000, 5000);

    task_mode=Task_Mode::MONITOR;
    heading_thresh=HEADING_THRESH;
    // heading_PID.set_exit_condition(80, 0.005, 300, 0.005, 500, 5000);
}

Gps_Drive::~Gps_Drive(){
    gps_task.remove();
}

void Gps_Drive::gps_task_func(){
    const char* path="/usd/heading_data.txt";
    std::ofstream file(path,std::ios::out | std::ios::trunc);
    pros::c::gps_status_s_t status_raw;
    auto start=pros::millis();
    while(true) {
        status_raw=gps_sensor.get_status();
        pros::screen::print(pros::E_TEXT_SMALL,0,"Gps raw: x:%.3lf ,y:%.3lf ,heading:%.1f",status_raw.x,status_raw.y,gps_sensor.get_heading());
        pros::screen::print(pros::E_TEXT_MEDIUM,2,"Gps error: %lf",gps_sensor.get_error());
        Eigen::VectorXd y(2);
        y<<status_raw.x,status_raw.y;
        kf.update(y);
        pos_mutex.take(mutex_timeout);
        ///TODO:暂时不用滤波试试
        position={status_raw.x,status_raw.y};
        // position={kf.state()[0],kf.state()[3]};
        pos_mutex.give();
        file<<"status_raw "<<status_raw.x<<" , "<<status_raw.y<<"\n";
        pros::screen::print(pros::E_TEXT_MEDIUM,1,"KalmanFilter status: x:%.4lf , y:%.4lf,",position.x,position.y);
        heading_angle=gps_sensor.get_heading();
        if(drive_toggle) {
            double l_out=0,r_out=0;
            switch (task_mode) {
                case Task_Mode::MONITOR:
                    task_mode=Task_Mode::TURN;
                case Task_Mode::TURN: 
                    turn_PID.compute(heading_angle);
                    file.open(path, std::ios::out | std::ios::app);
                    file<<"heading "<<heading_angle<<" , "<<turn_PID.output<<"\n";
                    file<<"position "<<position.x <<" , "<<position.y<<"\n";
                    file.close();
                    l_out=util::clip_num(turn_PID.output, max_speed, -max_speed);
                    r_out=util::clip_num(-turn_PID.output, max_speed, -max_speed);
                    pros::screen::print(pros::E_TEXT_MEDIUM,3,"turn_target%.1lf , turn_out%.1lf,",turn_PID.get_target(),turn_PID.output);
                    if(turn_PID.exit_condition(chassis_reference.left_motors[0])!=ez::RUNNING) {
                        pros::screen::print(pros::E_TEXT_SMALL,6,"turn exit: %s",ez::exit_to_string(turn_PID.exit_condition(chassis_reference.left_motors[0])));
                        task_mode=Task_Mode::STRAIGHT;
                    }
                    break;
                case Task_Mode::STRAIGHT:
                    update_heading_target();
                    float straight_out=straight_PID.compute(position.x,position.y);
                    float heading_out=heading_PID.compute(heading_angle);
                    float r=pow(position.x-target.x,2)+pow(position.y-target.y,2);
                    if(r<pow(heading_thresh,2))
                        heading_out=0;
                    double drive_out = util::clip_num(straight_out, max_speed, -max_speed);
                    l_out=straight_out+heading_out;
                    r_out=straight_out-heading_out;
                    pros::screen::print(pros::E_TEXT_MEDIUM,3,"straight_out%.2lf , heading_out%.2lf,",straight_out,heading_out);
                    pros::screen::print(pros::E_TEXT_MEDIUM,4,"heading_target%.2f",heading_PID.get_target());
                    file.open(path, std::ios::out | std::ios::app);
                    file<<"r "<<r<<std::endl;
                    file<<"heading_thresh"<<pow(heading_thresh,2)<<std::endl;
                    file<<"straight_out "<<straight_out<<" , "<<straight_out<<"\n";
                    file<<"position "<<position.x <<" , "<<position.y<<"\n";
                    file.close();
                    if(straight_PID.exit_condition(chassis_reference.left_motors[0])!=ez::RUNNING) {
                        task_mode=Task_Mode::MONITOR;
                        drive_toggle=false;
                        ///TODO: 单位为米时会因为derivative变化过小而退出 
                        pros::screen::print(pros::E_TEXT_MEDIUM,8,"gps drive exit: %s",ez::exit_to_string(straight_PID.exit_condition(chassis_reference.left_motors[0])));
                    }
                    break;
            }
            chassis_reference.set_tank(l_out,r_out);
        }
        pros::Task::delay_until(&start,20);
    }
}



void Gps_Drive::drive_to_position(int speed,double target_x,double target_y){
    max_speed=speed;
    target={target_x,target_y};
    pos_mutex.take(mutex_timeout);
    ///TODO: 直接读数值
    // auto [starting_x,starting_y]=position;
    auto status=gps_sensor.get_status();
    double starting_x=position.x;
    double starting_y=position.y;

    pos_mutex.give();
    double angle = atan((target_x - starting_x) / (target_y - starting_y)) / M_PI * 180;

    while(!Controller_Button_State::A_pressed());
    pros::delay(1000);
    if (target_y - starting_y < 0)
        angle = angle + 180;

    if(angle<0){
        angle+=360;
    }
    turn_PID.set_target(angle);

    while(!Controller_Button_State::A_pressed()){
        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_SMALL,7,"pos x:%.2lf ,y:%.2lf,angle:%.2lf",starting_x,starting_y,angle);
        
        pros::screen::print(pros::E_TEXT_SMALL,8,"target_x,y:%.3lf , %.3lf,init_x,y:%.3lf , %.3lf",target_x,target_y,starting_x,starting_y);
        pros::delay(10);
    }
    straight_PID.initlize(starting_x,starting_y,target_x,target_y);
    drive_toggle=true;

}


void Gps_Drive::update_heading_target(){
    double angle = atan((target.x - position.x) / (target.y - position.y)) / M_PI * 180;
    if (target.y - position.y < 0)
        angle = angle + 180;

    if(angle<0){
        angle+=360;
    }
    heading_PID.set_target(angle);
}

void Gps_Drive::wait_drive(){
    while(drive_toggle){
        pros::delay(10);
    }

}

bool Gps_Drive::check_Gps(){
    return gps_sensor.get_error()>0.5f;
}

void Gps_Drive::gps_imu_turn(double angle){

}