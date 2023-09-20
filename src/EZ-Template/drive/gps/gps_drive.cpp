#include "EZ-Template/drive/gps/gps_drive.hpp"
#include "EZ-Template/util.hpp"
#include "gps_pid.hpp"
#include "main.h"
#include "pros/screen.h"
#include "pros/screen.hpp"

Gps_Drive::Gps_Drive(Drive &drive_chassis,const std::uint8_t port, double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset):
    gps_sensor(port,xInitial,yInitial,headingInitial,xOffset,yOffset),chassis_reference(drive_chassis),
    get_position_task([this](){this->get_position_task_func();})
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
    Q=Q*0.6;//乘上加速度方差
    // 测量噪声协方差
    R << 0.001,0,
         0,0.001;
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

}

Gps_Drive::~Gps_Drive(){
    get_position_task.remove();
}

void Gps_Drive::get_position_task_func(){
    pros::c::gps_status_s_t status;
    auto start=pros::millis();
    while(true){
        status=gps_sensor.get_status();

        pros::screen::print(pros::E_TEXT_MEDIUM,0,"Gps raw status: x:%.4lf , y:%.4lf,",status.x,status.y);
        pros::screen::print(pros::E_TEXT_MEDIUM,2,"Gps error: %lf",gps_sensor.get_error());

        Eigen::VectorXd y(2);
        y<<status.x,status.y;
        kf.update(y);
        auto kf_x=kf.state();
        pros::screen::print(pros::E_TEXT_MEDIUM,1,"KalmanFilter status: x:%.4lf , y:%.4lf,",kf_x[0],kf_x[3]);


        position={status.x,status.y};
        heading_angle=gps_sensor.get_heading();
        if(drive_toggle){
            update_heading_target();
            float straight_out=straight_PID.compute(status.x,status.y);
            float heading_out=heading_PID.compute(heading_angle);

            double drive_out = util::clip_num(straight_out, max_speed, -max_speed);
            chassis_reference.set_tank(straight_out+heading_out,straight_out-heading_out);
            if(straight_PID.exit_condition(chassis_reference.left_motors[0])!=ez::RUNNING){
                drive_toggle=false;
                std::cout<<"Gps exit: "<<ez::exit_to_string(straight_PID.exit_condition(chassis_reference.left_motors[0]))<<"\n";
            }
        }
        pros::Task::delay_until(&start,10);
    }
}

void Gps_Drive::set_pid_contants(double kp,double ki,double kd,double start_i) {
    // Gps_PID.set_constants(kp,ki,kd,start_i);
}

void Gps_Drive::drive_to_position(double target_x,double target_y){
    target={target_x,target_y};
    auto [start_x,start_y]=position;
    straight_PID.initlize(start_x,start_y,target_x,target_y);
    drive_toggle=true;


}


void Gps_Drive::update_heading_target(){
    float distance = hypot(target.x-this->position.x,target.y-this->position.y);
    float angle =asin((target.x-this->position.x)/distance);
    float heading_angle = angle;

    float k=(target.y-this->position.y)/(target.x-this->position.x);
    
    if(k>0){
        if(target.y-this->position.y>0){
            heading_angle=angle;
        }
        else{
            heading_angle=180+angle;
        }
    }else{
        if(target.y-this->position.y>0){
            heading_angle=360-angle;
        }
        else{
            heading_angle=180-angle;
        }
    }
    heading_PID.set_target(heading_angle);
}

void Gps_Drive::wait_drive(){
    while(drive_toggle){
        pros::delay(10);
    }
    
}

bool Gps_Drive::check_Gps(){
    return gps_sensor.get_error()>0.5f;
}