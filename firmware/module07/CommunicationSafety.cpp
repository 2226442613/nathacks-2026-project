#include "CommunicationSafety.h"
#include "Cleaner.h"
#include "Protocol.h"

static bool emergencyLatched = false;
static bool watchdogArmed = false;
static bool watchdogExpired = false;
static unsigned long lastCommunication = 0;

void communicationSeen() {
  lastCommunication = millis(); watchdogArmed = true; watchdogExpired = false;
}
void communicationNewClient() { cleanerAbort(); communicationSeen(); }
void communicationLinkLost() { cleanerAbort(); watchdogArmed = false; watchdogExpired = true; }
void communicationEmergencyStop() { emergencyLatched = true; cleanerAbort(); communicationSeen(); }
void communicationResetEmergencyStop() {
  cleanerAbort(); emergencyLatched = false; communicationSeen(); // Never resume old motion/task.
}
bool communicationEstopped() { return emergencyLatched; }
void communicationUpdate() {
  if (watchdogArmed && millis() - lastCommunication > COMMUNICATION_WATCHDOG_MS) {
    cleanerAbort(); watchdogArmed = false; watchdogExpired = true;
    sendReply("EVENT,WATCHDOG_STOP");
  }
}
void reportSafety() {
  sendReply(String("SAFETY,ESTOP=") + (emergencyLatched ? "1" : "0") +
            ",WATCHDOG_MS=1000,WATCHDOG=" + (watchdogExpired ? "EXPIRED" : "OK"));
}
void receiveCommandByte(char c, String &line, bool &discard) {
  if (c == '\n') {
    if (!discard) { line.trim(); if (line.length()) processCommand(line); }
    line = ""; discard = false; return;
  }
  if (discard) return;
  bool afterCR = line.length() && line.charAt(line.length()-1) == '\r';
  if (c == '\r' && !afterCR) { line += c; return; } // Only valid immediately before LF.
  if (afterCR || (unsigned char)c < 32 || (unsigned char)c > 126 || line.length() >= 120) {
    line = ""; discard = true;
    cleanerAbort(); // Protocol faults never leave actuators running.
    sendReply(afterCR || (unsigned char)c < 32 || (unsigned char)c > 126 ? "ERR,ENCODING" : "ERR,LINE_TOO_LONG");
    return;
  }
  line += c;
}
