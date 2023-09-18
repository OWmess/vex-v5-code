#include "EZ-Template/drive/gps/gps_pid.hpp"

Gps_PID::Gps_PID():PID(){

}

Gps_PID::~Gps_PID(){

}

Gps_PID::Gps_PID(double p, double i, double d, double start_i, std::string name):PID(p,i,d,start_i,name){

}


