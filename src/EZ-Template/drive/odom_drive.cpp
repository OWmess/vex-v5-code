#include "main.h"

///TODO:: 这样写会导致在Drive被销毁后chassis变为空指针，待debug
Drive &Drive::with_odom(const float &ForwardTracker_center_distance,const float &SidewaysTracker_center_distance){
    return *this;
}

void Drive::drive_to_point(double x, double y,int speed,bool ibackwards , bool slew_on, bool toggle_heading){
    auto [distance,angle] = this->odom.computeDistanceAndAngleToPoint(Odom::Point{x,y});
    printf("distance:%f,angle:%f\n",distance,angle);
    if (ibackwards) {
        distance *= -1;
        angle += 180;
    }
    if(angle>180){
        angle-=360;
    }
    this->set_turn_pid(angle,speed);
    this->wait_drive();
    this->set_drive_pid(distance,speed,slew_on,toggle_heading);
    this->wait_drive();
    this->odom.update_position(Odom::Point{x,y});
}

void Drive::trun_to_point(double x, double y,int speed){
    
}