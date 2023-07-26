#include "main.h"

void Odom::set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position){
  this->X_position = X_position;
  this->Y_position = Y_position;
}

void Odom::update_position(Point p){
  this->X_position=p.x;
  this->Y_position=p.y;
}

void Odom::initlize(float start_x,float start_y,float moveThreshold,float turnThreshold){
  this->X_position=start_x;
  this->Y_position=start_y;
  this->moveThreshold=moveThreshold;
  this->turnThreshold=turnThreshold;

}

float Odom::get_X_position(){
  return this->X_position;
}

float Odom::get_Y_position(){
  return this->Y_position;
}

Odom::Point Odom::get_position(){
  return Point(this->X_position,this->Y_position);
}

void Odom::set_threshold(float moveThreshold,float turnThreshold){
  this->moveThreshold=moveThreshold;
  this->turnThreshold=turnThreshold;
}
