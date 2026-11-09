#pragma once
#include <Arduino.h>
constexpr unsigned long COMMUNICATION_WATCHDOG_MS = 1000UL;
void communicationSeen();
void communicationUpdate();
void communicationNewClient();
void communicationLinkLost();
void communicationEmergencyStop();
void communicationResetEmergencyStop();
bool communicationEstopped();
void reportSafety();
void receiveCommandByte(char c, String &line, bool &discard);
