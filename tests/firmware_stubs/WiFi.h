#pragma once
#include "Arduino.h"
#include <vector>
constexpr int WIFI_STA=1,WL_CONNECTED=3;
extern std::vector<std::string> replies;
struct WiFiClient {
 operator bool()const{return true;}
 bool connected()const{return true;}
 void println(const String& s){replies.push_back(s.value);}
 void stop(){}
 void setNoDelay(bool){}
 int available(){return 0;}
 int read(){return -1;}
};
struct WiFiServer { explicit WiFiServer(int){} void begin(){} WiFiClient available(){return {};}};
struct WiFiStub {
 void mode(int){} void setSleep(bool){} void begin(const char*,const char*){}
 int status(){return WL_CONNECTED;} void reconnect(){} String localIP(){return "loopback-stub";}
};
extern WiFiStub WiFi;
