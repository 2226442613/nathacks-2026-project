#pragma once
#include <Arduino.h>
void processCommand(String message);
void sendReply(const String& reply);
void reportStatus();
