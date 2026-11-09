#pragma once
#include <Arduino.h>
void actuatorsBegin();
void pumpsOff();
void pumpSelect(char fluid); // 'B' blood, 'O' other; '?' = off
void sprayServoAngle(int degrees);
void spongeServoAngle(int degrees);
