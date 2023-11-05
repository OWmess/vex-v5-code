#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "control.hpp"
extern Drive chassis;
extern Control control;
void guard();

void attack();

void conservatively_attack();

void default_constants();

void test_pid();

void skill_match();

void guard_1();

void attack_aggressive();

void guard_aggressive();

void get_sensor_data();