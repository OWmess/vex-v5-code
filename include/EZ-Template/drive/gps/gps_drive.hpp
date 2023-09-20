#pragma once

#include "pros/gps.hpp"
#include "pros/rtos.hpp"
#include "EZ-Template/PID.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "gps_pid.hpp"
#include "EZ-Template/drive/gps/kalman.hpp"

class Gps_Drive{
public:
    Gps_Drive(Drive &drive_chassis,const std::uint8_t port, double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset);
    ~Gps_Drive();

    void set_pid_contants(double kp,double ki,double kd,double start_i);

    void drive_to_position(double x,double y);

    void turn_to_degree(double deg);

    void wait_drive();

    bool check_Gps();
private:


    void update_heading_target();

    void get_position_task_func();
private:
    float heading_angle;
    int max_speed;
    PID heading_PID;
    Gps_PID straight_PID;
    Point2d target;
    Point2d position;
    pros::Gps gps_sensor;
    pros::Task get_position_task;
    Drive &chassis_reference;
    bool drive_toggle;
    KalmanFilter kf;
};