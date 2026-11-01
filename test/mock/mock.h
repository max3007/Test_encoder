#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <algorithm>
using std::min; using std::max;
#define IRAM_ATTR
#define DRAM_ATTR
#define PROGMEM
#define PGM_P const char*
#define HIGH 1
#define LOW 0
#define INPUT 1
#define OUTPUT 2
#define INPUT_PULLUP 3
#define CHANGE 3
#define ADC_11db 3
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void)0
#define portEXIT_CRITICAL(x) (void)0
#define portENTER_CRITICAL_ISR(x) (void)0
#define portEXIT_CRITICAL_ISR(x) (void)0
extern uint32_t fakeReg[2];
extern uint64_t fakeTimeUs;
extern void (*isrTab[40])();
inline int digitalRead(int pin){ return pin>=32 ? (fakeReg[1]>>(pin-32))&1 : (fakeReg[0]>>pin)&1; }
inline void digitalWrite(int,int){}
inline void pinMode(int,int){}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int p, void(*f)(), int){ isrTab[p]=f; }
inline uint32_t analogReadMilliVolts(int){ return 1000; }
inline void analogSetPinAttenuation(int,int){}
inline uint32_t micros(){ return (uint32_t)fakeTimeUs; }
inline uint32_t millis(){ return (uint32_t)(fakeTimeUs/1000); }
inline void delay(int){}
struct FakeSerial { void begin(int){} template<class T> void print(T){} template<class T> void println(T){} void println(){}
  void printf(const char*,...){} } ;
extern FakeSerial Serial;
