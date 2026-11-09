#pragma once
#include "Arduino.h"
enum class Surface:uint8_t{NONE,GROUND,WALL};
enum class Fluid:uint8_t{NONE,BLOOD,OTHER};
void cleanerBegin();void cleanerAbort();void cleanerUpdate();
bool cleanerSetTarget(Surface,Fluid);bool cleanerStart();bool cleanerBusy();
const char* cleanerStateName();
