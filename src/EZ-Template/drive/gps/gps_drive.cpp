#include "EZ-Template/drive/gps/gps_drive.hpp"
#include "main.h"
#include "pros/gps.h"

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
        pros::Task::delay_until(&start,10);
    }
}

void Gps_Drive::set_pid_contants(double kp,double ki,double kd,double start_i) {
    // Gps_PID.set_constants(kp,ki,kd,start_i);
}

void Gps_Drive::drive_to_position(double target_x,double target_y){
    float distance = hypot(target_x-this->position.x,target_y-this->position.y);
    float angle =acos((target_x-this->position.x)/distance)*180/M_PI;

    // Gps_PID.set_target(distance);

    
}

void turn_to_degree(double deg){

}