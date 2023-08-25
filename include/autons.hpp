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