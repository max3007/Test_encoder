// Simulatore su PC per TesterEncoder.ino: vedi esegui_test.sh
#include "mock.h"
#include <cmath>
#include "WiFi.h"
#include "WebServer.h"
uint32_t fakeReg[2]; uint64_t fakeTimeUs; void (*isrTab[40])(); FakeSerial Serial; FakeWiFi WiFi; std::string lastBody;
#include SKETCH

// Simulatore: passi da 0.1 us, interrupt con latenza (ingresso Lin, uscita Lout), flag GPIO "latched".
struct Sim {
  double t = 0;                // us
  int a = 1, b = 1;
  bool pend[2] = {false,false};
  double busyUntil = 0, execAt = -1; int execPin = -1;
  double Lin = 0.0, Lout = 0.0;
  long edgesA = 0, edgesB = 0; // fronti di salita reali
  void setReg(){ fakeReg[1] = (a<<2)|(b<<3); }
  void step(int na, int nb){
    if (na != a){ if(na) edgesA++; a = na; pend[0] = true; }
    if (nb != b){ if(nb) edgesB++; b = nb; pend[1] = true; }
    setReg();
    if (execAt >= 0 && t >= execAt){ fakeTimeUs = (uint64_t)t; isrTab[execPin==0?PIN_A:PIN_B](); execAt = -1; }
    if (execAt < 0 && t >= busyUntil && (pend[0]||pend[1])){
      execPin = pend[0]?0:1; pend[execPin] = false;
      execAt = t + Lin; busyUntil = t + Lin + Lout;
    }
  }
};
Sim S;
double nextWin = 500000;
void finestra(){ if (S.t >= nextWin){ fakeTimeUs=(uint64_t)S.t; calcolaFinestra(500); nextWin += 500000; } }
// segnale in quadratura: lag = ritardo di B rispetto ad A in gradi
void run(double durUs, double f, double lagDeg, double duty, double off=0.0){
  double end = S.t + durUs;
  for (; S.t < end; S.t += 0.1){
    double ph = S.t*1e-6*f + off;
    double fa = ph - floor(ph), pb = ph - lagDeg/360.0, fb = pb - floor(pb);
    S.step(fa < duty, fb < duty); finestra();
  }
}
void hold(double durUs){ double end = S.t + durUs; for (; S.t < end; S.t += 0.1){ S.step(S.a,S.b); finestra(); } }
void stampa(const char* tit){
  hold(0.5);
  printf("%-44s fA=%8.2f fB=%8.2f dA=%6.1f dB=%6.1f fase=%6.1f totA=%lu/%ld totB=%lu/%ld err=%lu pos=%ld senso=%s\n",
    tit, freqA, freqB, dutyA, dutyB, faseGradi, (unsigned long)totAMostrati, S.edgesA,
    (unsigned long)totBMostrati, S.edgesB, (unsigned long)errMostrati, (long)posMostrata, senso);
}
void reset(){ S.edgesA = S.edgesB = 0; azzera(); }
int main(){
  S.setReg(); fakeTimeUs = 0; setup();
  S.Lin = 0; S.Lout = 0;
  reset(); run(1000000, 1000, 90, 0.5, 0.01);   stampa("1 kHz, B in ritardo 90, duty 50");
  reset(); run(1000000, 1000, 270, 0.5, 0.01);  stampa("1 kHz, B in ritardo 270, duty 50");
  reset(); run(1000000, 250, 90, 0.3, 0.01);    stampa("250 Hz, duty 30");
  reset(); run(1000000, 3.3, 90, 0.5, 0.01);    stampa("3,3 Hz (bassa velocita')");
  // inversione del moto e azzeramento con encoder fermo
  run(1000000, 1000, 90, 0.5, 0.01); hold(0.5); reset(); hold(499999); stampa("1a finestra dopo azzera(), encoder fermo");
  // sosta lunga con A alto, poi ripartenza
  run(600000, 1000, 90, 0.5, 0.2); hold(3000000); run(400000, 1000, 90, 0.5, 0.2); hold(100000);
  stampa("ripartenza dopo sosta 3 s con A alto");
  // fase vicina a 0 gradi (A e B quasi coincidenti, jitter +-2 gradi)
  reset(); for (int i=0;i<1000;i++){ run(1000, 1000, (i%2)?2:-2, 0.5, 0.01); } stampa("fase ~0 (jitter +-2)");
  // latenza interrupt realistica, 5 kHz e 15 kHz
  S.Lin = 2.0; S.Lout = 1.5;
  reset(); run(1000000, 5000, 90, 0.5, 0.01);   stampa("5 kHz, latenza ISR 3,5 us");
  reset(); run(1000000, 15000, 90, 0.5, 0.01);  stampa("15 kHz, latenza ISR 3,5 us");
  // A e B quasi simultanei (sfasamento 3 gradi a 2 kHz = 4 us)
  reset(); run(1000000, 2000, 1, 0.5, 0.01);    stampa("2 kHz, B quasi insieme ad A (1,4 us)");
  inviaDati(); printf("JSON: %s\n", lastBody.c_str());
}
