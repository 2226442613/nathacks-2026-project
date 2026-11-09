#include "Protocol.h"
#include "Cleaner.h"
#include "MotorControl.h"
#include "CommunicationSafety.h"
#include <WiFi.h>
extern WiFiClient pcClient;

void sendReply(const String& reply) {
  Serial.println(reply);
  if (pcClient && pcClient.connected()) pcClient.println(reply);
}

void reportStatus() { sendReply(String("STATUS,") + cleanerStateName()); }

void processCommand(String message) {
  message.trim(); message.toUpperCase();
  if (message == "CAPS") { communicationSeen(); sendReply("CAPS,NATHACK_V1_1,MOVE_PWM_0_255,STOP,ESTOP,RESET_ESTOP,HEARTBEAT,SAFETY,TARGET,ARRIVED"); return; }
  if (message == "SAFETY") { communicationSeen(); reportSafety(); return; }
  if (message == "HEARTBEAT") { communicationSeen(); sendReply("OK,HEARTBEAT"); return; }
  if (message == "ESTOP") { communicationEmergencyStop(); sendReply("OK,ESTOP"); return; }
  if (message == "RESET_ESTOP") { communicationResetEmergencyStop(); sendReply("OK,RESET_ESTOP"); return; }
  if (message == "STOP" || message == "ABORT") {
    communicationSeen(); cleanerAbort(); sendReply("OK,STOP"); return;
  }
  if (message == "STATUS") { communicationSeen(); reportStatus(); return; }
  if (message == "PING") { communicationSeen(); sendReply("PONG"); return; }
  if (communicationEstopped()) { sendReply("ERR,ESTOP_LATCHED"); return; }

  if (message.startsWith("TARGET,")) {
    int comma = message.indexOf(',', 7);
    if (comma < 0 || message.indexOf(',', comma + 1) >= 0) {
      sendReply("ERR,TARGET_FORMAT"); return;
    }
    String surfaceText = message.substring(7, comma);
    String fluidText = message.substring(comma + 1);
    Surface surface = surfaceText == "GROUND" ? Surface::GROUND :
                      surfaceText == "WALL" ? Surface::WALL : Surface::NONE;
    Fluid fluid = fluidText == "BLOOD" ? Fluid::BLOOD :
                  fluidText == "OTHER" ? Fluid::OTHER : Fluid::NONE;
    if (surface == Surface::NONE || fluid == Fluid::NONE) {
      sendReply("ERR,UNKNOWN_TARGET"); return;
    }
    communicationSeen();
    sendReply(cleanerSetTarget(surface, fluid) ? "OK,TARGET" : "ERR,BUSY");
    return;
  }
  if (message == "ARRIVED") {
    communicationSeen();
    sendReply(cleanerStart() ? "OK,ARRIVED" : "ERR,NO_TARGET");
    return;
  }
  if (message.startsWith("MOVE,")) {
    if (cleanerBusy()) { sendReply("ERR,BUSY"); return; }
    int comma = message.indexOf(',', 5);
    if (comma != 6 || message.indexOf(',', comma + 1) >= 0) {
      sendReply("ERR,MOVE_FORMAT"); return;
    }
    char dir = message.charAt(5);
    String speedText = message.substring(comma + 1);
    if (dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R') {
      sendReply("ERR,DIRECTION"); return;
    }
    if (speedText.isEmpty()) { sendReply("ERR,SPEED"); return; }
    for (unsigned int i = 0; i < speedText.length(); i++) {
      if (!isDigit(speedText[i])) { sendReply("ERR,SPEED"); return; }
    }
    // Accumulate with an upper bound; String.toInt overflow must not wrap to a valid PWM.
    long speed = 0;
    for (unsigned int i = 0; i < speedText.length(); i++) {
      speed = speed * 10 + (speedText[i] - '0');
      if (speed > 255) { sendReply("ERR,SPEED_RANGE"); return; }
    }
    if (speed < 0 || speed > 255) { sendReply("ERR,SPEED_RANGE"); return; }
    communicationSeen();
    motorsDrive(dir, (int)speed, true);
    sendReply("OK,MOVE");
    return;
  }
  sendReply("ERR,UNKNOWN_COMMAND");
}
