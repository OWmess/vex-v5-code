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
    auto it=find_if(left_motors.begin(),left_motors.end(),[i](pros::Motor &motor){
      return motor.get_port()==i;
    });
    if(it!=left_motors.end()){
      it->set_brake_mode(CURRENT_BRAKE); // Set the motor to the brake type of the drive
      it->set_current_limit(CURRENT_MA); // Set the motor to the mA of the drive
    }else {
      it=find_if(right_motors.begin(),right_motors.end(),[i](pros::Motor &motor){
        return motor.get_port()==i;
      });
      if(it!=right_motors.end()){
        it->set_brake_mode(CURRENT_BRAKE); // Set the motor to the brake type of the drive
        it->set_current_limit(CURRENT_MA); // Set the motor to the mA of the drive
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

Drive Drive::with_pto(std::initializer_list<int> list) {
  this->pto_list = {list};
  for(auto &i:pto_list){
    i=abs(i);
  }
  for (auto i : pto_list) {
    auto comp = [i](pros::Motor &motor) {
      return motor.get_port() == i;
    };
    auto it = std::find_if(left_motors.begin(), left_motors.end(), comp);
    if (it == left_motors.end()) {
      it = std::find_if(right_motors.begin(), right_motors.end(), comp);
      if (it == right_motors.end()) {  // 如果PTO电机不在左右电机中，抛出异常
        throw runtime_error("Invalid PTO motor.");
      }
    }
  }
  //pto电机不可用于获取底盘相关的信息（如过载、编码值等）
  //查找并保存非pto电机的索引
  auto it = std::find_if(left_motors.begin(), left_motors.end(), [this](pros::Motor &motor) {
    return std::any_of(pto_list.begin(), pto_list.end(), [&motor](int pto_port) {
      return pto_port != motor.get_port();
    });
  });
  left_condition_index = std::distance(left_motors.begin(), it);
  it = std::find_if(right_motors.begin(), right_motors.end(), [this](pros::Motor &motor) {
    return std::any_of(pto_list.begin(), pto_list.end(), [&motor](int pto_port) {
      return pto_port != motor.get_port();
    });
  });
  right_condition_index = std::distance(right_motors.begin(), it);
  return *this;
}

std::vector<pros::Motor> Drive::get_pto_motors() {
  std::vector<pros::Motor> motors;
  for(auto port:pto_list){
    auto comp=[port](pros::Motor &motor){
      return motor.get_port()==port;
    };
    auto it=find_if(left_motors.begin(),left_motors.end(),comp);
    if(it!=left_motors.end()){
      motors.push_back(*it);
    }else {
      it=find_if(right_motors.begin(),right_motors.end(),comp);
      if(it!=right_motors.end()){
        motors.push_back(*it);
      }
    }
  }
  return motors;
}