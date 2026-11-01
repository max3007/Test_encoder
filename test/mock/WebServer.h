#pragma once
#include "mock.h"
#include <functional>
#include <string>
extern std::string lastBody;
struct WebServer { WebServer(int){} void on(const char*, std::function<void()>){} void onNotFound(std::function<void()>){}
 void begin(){} void handleClient(){} void send_P(int,const char*,const char*){}
 void send(int,const char*,const char*b){lastBody=b;} void sendHeader(const char*,const char*){} };
