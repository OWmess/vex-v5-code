#include "main.h"

Drive &Drive::with_odom(const float &ForwardTracker_center_distance,const float &SidewaysTracker_center_distance){
    this->odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    return *this;
}



