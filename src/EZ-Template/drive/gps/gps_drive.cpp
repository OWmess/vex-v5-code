#include "EZ-Template/drive/gps/gps_drive.hpp"
#include "main.h"

Gps_Drive::Gps_Drive(const std::uint8_t port, double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset):
    gps_sensor(port,xInitial,yInitial,headingInitial,xOffset,yOffset),
    get_position_task([this](){this->get_position_task_func();})
    {
    gps_sensor.set_data_rate(10);

}

Gps_Drive::~Gps_Drive(){
    get_position_task.remove();
}

void Gps_Drive::get_position_task_func(){
    pros::c::gps_status_s_t status;
    auto start=pros::millis();
    while(true){
        status=gps_sensor.get_status();
        position={status.x,status.y};

        update();
        pros::Task::delay_until(&start,10);
    }
}

void Gps_Drive::set_pid_contants(double kp,double ki,double kd,double start_i) {
    // Gps_PID.set_constants(kp,ki,kd,start_i);
}

void Gps_Drive::drive_to_position(double target_x,double target_y){
    target={target_x,target_y};
    straight_PID.set_target(0);

}


void Gps_Drive::update(){

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

    float heading_output=heading_PID.compute(this->heading_angle);



}