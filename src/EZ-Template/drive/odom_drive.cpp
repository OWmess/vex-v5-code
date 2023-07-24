#include "main.h"

Drive &Drive::with_odom(const float &ForwardTracker_center_distance,const float &SidewaysTracker_center_distance){
    this->odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    return *this;
}

void Drive::drive_to_point(double x, double y,int speed, bool slew_on, bool toggle_heading){
    double target=hypot(x-odom.X_position,y-odom.Y_position);
    set_drive_pid(target,speed,slew_on,toggle_heading);

}

void Drive::trun_to_point(double x, double y,int speed){

}