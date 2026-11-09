#include "Arduino.h"
#include "WiFi.h"
#include "Cleaner.h"
#include "Protocol.h"
#include "CommunicationSafety.h"
#include <iostream>
unsigned long fakeMillis=0;
SerialStub Serial; WiFiStub WiFi; WiFiClient pcClient;
std::vector<std::string> replies;
std::string state="IDLE";
int pwm=0;char direction='-';bool pump=false;
void motorsBegin(){} void motorsStop(){pwm=0;} void motorsUpdate(){} void actuatorsBegin(){}
void cleanerBegin(){} void cleanerUpdate(){}
void cleanerAbort(){pwm=0;pump=false;state="IDLE";}
bool cleanerBusy(){return state!="IDLE"&&state!="APPROACH";}
const char* cleanerStateName(){return state.c_str();}
bool cleanerSetTarget(Surface,Fluid){if(state!="IDLE")return false;state="APPROACH";return true;}
bool cleanerStart(){if(state!="APPROACH")return false;state="SPRAYING";pump=true;sendReply("EVENT,CLEANING_STARTED");return true;}
void motorsDrive(char d,int p,bool){direction=d;pwm=p;}
int main(){
 communicationNewClient();std::string line;
 String buffer;bool discard=false;
 while(std::getline(std::cin,line)){
  if(line.rfind("@TIME,",0)==0){fakeMillis=std::stoul(line.substr(6));communicationUpdate();}
  else if(line=="@DISCONNECT"){communicationLinkLost();}
  else if(line=="@CONNECT"){communicationNewClient();}
  else if(line.rfind("@RAW,",0)==0){for(char c:line.substr(5))receiveCommandByte(c,buffer,discard);receiveCommandByte('\n',buffer,discard);}
  else processCommand(String(line));
  for(auto &r:replies)std::cout<<r<<'\n';replies.clear();
  std::cout<<"@STATE,"<<state<<","<<direction<<","<<pwm<<","<<pump<<'\n';
 }
}
