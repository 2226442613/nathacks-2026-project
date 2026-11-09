#pragma once
#include <Arduino.h>
enum class Surface : uint8_t { NONE, GROUND, WALL };
enum class Fluid : uint8_t { NONE, BLOOD, OTHER };
void cleanerBegin();
bool cleanerSetTarget(Surface surface, Fluid fluid);
bool cleanerStart();
void cleanerUpdate();
void cleanerAbort();
bool cleanerBusy();
bool cleanerHasTarget();
const char* cleanerStateName();
