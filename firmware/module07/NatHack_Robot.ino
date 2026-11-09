#include <Arduino.h>
#include <WiFi.h>
#include "Config.h"
#include "MotorControl.h"
#include "Actuators.h"
#include "Cleaner.h"
#include "Protocol.h"
#include "CommunicationSafety.h"

WiFiServer robotServer(NET_PORT);
WiFiClient pcClient;
String receiveLine;
bool wifiDiscard = false;
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
  communicationUpdate();
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
    communicationLinkLost(); // Preserve existing abort/home behavior, including pumps.
    Serial.println("EVENT,LINK_LOST_ABORTED");
  }
  hadClient = clientAlive;
  if (!clientAlive) {
    if (pcClient) pcClient.stop();
    receiveLine = "";
    wifiDiscard = false;
    if (connected) {
      WiFiClient candidate = robotServer.available();
      if (candidate) {
        pcClient = candidate;
        hadClient = true;
        pcClient.setNoDelay(true);
        communicationNewClient();
        sendReply("CONNECTED,NATHACK_V1");
        reportStatus();
      }
    }
  }
  if (pcClient && pcClient.connected()) {
    // Bounded readers discard an ENTIRE bad/oversized line, never execute its tail.

    unsigned int byteBudget = 256;
    while (pcClient.available() && byteBudget--) {
      receiveCommandByte((char)pcClient.read(), receiveLine, wifiDiscard);
    }
    if (!pcClient.connected()) wifiDiscard = false;
  }
  static String serialLine;
  static bool serialDiscard = false;
  unsigned int serialBudget = 256;
  while (Serial.available() && serialBudget--) {
    receiveCommandByte((char)Serial.read(), serialLine, serialDiscard);
  }
  communicationUpdate();
  motorsUpdate();
  cleanerUpdate();
}
