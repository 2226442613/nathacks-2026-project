#include <Arduino.h>
#include <WiFi.h>
#include "Config.h"
#include "MotorControl.h"
#include "Actuators.h"
#include "Cleaner.h"
#include "Protocol.h"

WiFiServer robotServer(NET_PORT);
WiFiClient pcClient;
String receiveLine;
unsigned long lastWifiAttempt = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("NatHack Robot V1.0");
  motorsBegin();
  actuatorsBegin();
  cleanerBegin();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  robotServer.begin();
  Serial.println("Connecting to Wi-Fi...");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiAttempt > 10000UL) {
      lastWifiAttempt = millis();
      WiFi.reconnect();
    }
  }
  static bool wasConnected = false;
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wasConnected) {
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  }
  wasConnected = connected;

  static bool hadClient = false;
  bool clientAlive = pcClient && pcClient.connected();
  if (hadClient && !clientAlive) {
    cleanerAbort(); // Lost link: stop motors AND pumps and return servos home.
    Serial.println("EVENT,LINK_LOST_ABORTED");
  }
  hadClient = clientAlive;
  if (!clientAlive) {
    if (pcClient) pcClient.stop();
    receiveLine = "";
    if (connected) {
      WiFiClient candidate = robotServer.available();
      if (candidate) {
        pcClient = candidate;
        hadClient = true;
        pcClient.setNoDelay(true);
        sendReply("CONNECTED,NATHACK_V1");
        reportStatus();
      }
    }
  }
  if (pcClient && pcClient.connected()) {
    // Upper limit prevents unbounded RAM consumption from bad input.
    unsigned int byteBudget = 256;
    while (pcClient.available() && byteBudget--) {
      char c = (char)pcClient.read();
      if (c == '\n') {
        receiveLine.trim();
        if (receiveLine.length()) processCommand(receiveLine);
        receiveLine = "";
      } else if (c != '\r') {
        if (receiveLine.length() < 120) receiveLine += c;
        else receiveLine = "";
      }
    }
  }
  // Serial Monitor can issue the same commands during bench testing.
  static String serialLine;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialLine.trim();
      if (serialLine.length()) processCommand(serialLine);
      serialLine = "";
    } else if (c != '\r' && serialLine.length() < 120) serialLine += c;
  }
  motorsUpdate();
  cleanerUpdate();
}
