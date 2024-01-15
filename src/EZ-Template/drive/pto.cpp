/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <algorithm>
#include <vector>

#include "drive.hpp"
#include "main.h"

bool Drive::pto_active_check(pros::Motor check_if_pto) {
  auto does_exist = std::find(pto_active.begin(), pto_active.end(), check_if_pto.get_port());
  if (does_exist != pto_active.end())
    return true;  // Motor is in the list
  return false;   // Motor isn't in the list
}

void Drive::pto_add(std::vector<int> pto_list) {
  for (auto i : pto_list) {
    // Return if the motor is already in the list
    pros::Motor temp(i);
    if (pto_active_check(temp)) return;

    // Return if the first index was used (this motor is used for velocity)
    if (i == left_motors[left_condition_index].get_port() || i == right_motors[right_condition_index].get_port()) {
      printf("You cannot PTO the condition motor!\n");
      return;
    }
    pto_active.push_back(i);
  }
}

void Drive::pto_remove(std::vector<int> pto_list) {
  for (auto i : pto_list) {
    auto does_exist = std::find(pto_active.begin(), pto_active.end(), i);
    // Return if the motor isn't in the list
    if (does_exist == pto_active.end()) continue;
    
    // Find index of motor
    int index = std::distance(pto_active.begin(), does_exist);
    pto_active.erase(pto_active.begin() + index);
    //将电机设置为默认的制动模式和电流限制
    for(index=0;i<left_motors.size();index++){
      if(left_motors[index].get_port()==i){
        break;
      }
    }
    if(index!=left_motors.size()){
      left_motors[index].set_brake_mode(CURRENT_BRAKE); // Set the motor to the brake type of the drive
      left_motors[index].set_current_limit(CURRENT_MA); // Set the motor to the mA of the drive
    }else {
      for(index=0;i<right_motors.size();index++){
        if(right_motors[index].get_port()==i){
          break;
        }
      }
      if(index!=right_motors.size()){
        right_motors[index].set_brake_mode(CURRENT_BRAKE); // Set the motor to the brake type of the drive
        right_motors[index].set_current_limit(CURRENT_MA); // Set the motor to the mA of the drive
      }
    }
  }
}

void Drive::pto_toggle(bool toggle) {
  if (toggle)
    pto_add(this->pto_list);
  else
    pto_remove(this->pto_list);
}

void Drive::with_pto(std::initializer_list<int> list) {
  this->pto_list = {list};
  std::cout<<"pto_list: ";
  for(auto &i:pto_list){
    std::cout<<i<<", ";
    i=abs(i);
  }
  std::cout<<"\n";
  for (auto i : pto_list) {
    int j=0;
    for(j=0;j<left_motors.size();j++){
      if(left_motors[j].get_port()==i){
        break;
      }
    }
    if(j==left_motors.size()){
      for(j=0;j<right_motors.size();j++){
        if(right_motors[j].get_port()==i){
          break;
        }
      }
      if(j==right_motors.size()){
        throw runtime_error("Invalid PTO motor.");
      }
    }
  }
  //pto电机不可用于获取底盘相关的信息（如过载、编码值等）
  //查找并保存非pto电机的索引
  for(int i=0;i<left_motors.size();i++){
    if(std::find(pto_list.begin(),pto_list.end(),left_motors[i].get_port())==pto_list.end()){
      left_condition_index=i;
    }
  }
  std::cout<<"left_condition_index: "<<left_condition_index<<"\n";

  for(int i=0;i<right_motors.size();i++){
    if(std::find(pto_list.begin(),pto_list.end(),right_motors[i].get_port())==pto_list.end()){
      right_condition_index=i;
    }
  }
  std::cout<<"right_condition_index: "<<right_condition_index<<"\n";
}

std::vector<pros::Motor> Drive::get_pto_motors() {
  std::vector<pros::Motor> motors;
  for(auto port:pto_list){
    for(int i=0;i<left_motors.size();i++){
      if(left_motors[i].get_port()==port){
        motors.push_back(left_motors[i]);
        break;
      }
    }
    for(int i=0;i<right_motors.size();i++){
      if(right_motors[i].get_port()==port){
        motors.push_back(right_motors[i]);
        break;
      }
    }
  }
  return motors;
}