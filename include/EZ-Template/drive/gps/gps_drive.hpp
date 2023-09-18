#pragma once

#include "pros/gps.hpp"
#include "pros/rtos.hpp"
#include "EZ-Template/PID.hpp"
#include "EZ-Template/drive/drive.hpp"
struct Gps_Position{
    double x;
    double y;
};

class Gps_Drive{
public:
    Gps_Drive(const std::uint8_t port, double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset);
    ~Gps_Drive();

    void set_pid_contants(double kp,double ki,double kd,double start_i);

    void drive_to_position(double x,double y);

    void turn_to_degree(double deg);


private:


    

    void get_position_task_func();
private:
    PID heading_PID;
    PID straight_PID;
    Gps_Position position;
    pros::Gps gps_sensor;
    pros::Task get_position_task;

    
};