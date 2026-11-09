#pragma once
#include <Arduino.h>
void motorsBegin();
void motorsStop();
void motorsDrive(char direction, int pwm, bool timed = true);
void motorsUpdate();
