#pragma once
#include "pose.hpp"
#include "pros/gps.hpp"
#include "pros/rtos.hpp"
#include "EZ-Template/PID.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "gps/kalman_filter.hpp"


class Gps_Drive{
    public:
    Gps_Drive(Drive &drive_chassis,const std::uint8_t gps_port);
    Gps_Drive(Drive &drive_chassis,const std::uint8_t gps_port, double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset);

    void initlize_gps(double xInitial, double yInitial, double headingInitial, double xOffset, double yOffset);

    void move_to(float x, float y, float heading, int max_speed,bool forward = true,float chasePower=0,float lead=0.6);
    
    void wait_drive();
    private:
    void initlize_kf();

    void gps_task_fn();

    Pose get_position();

    void set_position(const Pose &position);

    double get_traveled_dist(double tick);

    /**
    * @brief Get the signed curvature of a circle that intersects the first pose and the second pose
    *
    * @note The circle will be tangent to the theta value of the first pose
    * @note The curvature is signed. Positive curvature means the circle is going clockwise, negative means
    * counter-clockwise
    * @note Theta has to be in radians and in standard form. That means 0 is right and increases counter-clockwise
    *
    * @param pose the first pose
    * @param other the second pose
    * @return float curvature
    */
    float getCurvature(Pose pose, Pose other);
    private:
    pros::GPS gps_sensor;
    Drive &drive_chassis;
    KalmanFilter kf;
    pros::Task gps_task;
    Pose position;
    pros::Mutex position_mutex;
    pros::Mutex moving_mutex;
    Drive::Drive_Config chassis_config;
    double tick_per_inch;
};


