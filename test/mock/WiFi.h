#pragma once
#include "mock.h"
#define WIFI_AP 2
struct FakeWiFi { void mode(int){} bool softAP(const char*,const char*){return true;} const char* softAPIP(){return "192.168.4.1";} };
extern FakeWiFi WiFi;
