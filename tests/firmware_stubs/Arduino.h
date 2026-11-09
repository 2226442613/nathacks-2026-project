#pragma once
#include <string>
#include <cstdint>
#include <cctype>
#include <algorithm>
class String {
 public:
  std::string value;
  String(const char* s=""):value(s){}
  String(const std::string& s):value(s){}
  void trim() { auto a=value.find_first_not_of(" \t\r\n"); if(a==std::string::npos){value.clear();return;} value=value.substr(a,value.find_last_not_of(" \t\r\n")-a+1); }
  void toUpperCase(){for(char &c:value)c=std::toupper((unsigned char)c);}
  bool startsWith(const char* s)const{return value.rfind(s,0)==0;}
  int indexOf(char c,int start=0)const{auto p=value.find(c,start);return p==std::string::npos?-1:(int)p;}
  String substring(int a)const{return value.substr(a);}
  String substring(int a,int b)const{return value.substr(a,b-a);}
  char charAt(int p)const{return value.at(p);}
  char operator[](unsigned int i)const{return value[i];}
  bool isEmpty()const{return value.empty();}
  unsigned int length()const{return value.size();}
  String& operator+=(char c){value+=c;return *this;}
};
inline bool operator==(const String&a,const char*b){return a.value==b;}
inline String operator+(const String&a,const String&b){return a.value+b.value;}
inline bool isDigit(char c){return c>='0'&&c<='9';}
extern unsigned long fakeMillis;
inline unsigned long millis(){return fakeMillis;}
inline void delay(int){}
struct SerialStub {
 void begin(int){}
 void println(const String&){}
 void print(const String&){}
 int available(){return 0;}
 int read(){return -1;}
};
extern SerialStub Serial;
