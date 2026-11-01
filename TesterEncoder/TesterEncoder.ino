/*
  TESTER ENCODER - ESP32 DevKit
  ------------------------------------------------------------
  Encoder alimentato a 5 V (pin VIN), segnali A e B tramite partitori.
  Collegamenti (vedi schema_tester_encoder.svg):
    A  -> GPIO 34      LED A -> GPIO 25 (330 ohm)
    B  -> GPIO 35      LED B -> GPIO 26 (330 ohm)
    Voltage Sensor S -> GPIO 36 (VP)   [facoltativo]
    Pulsante reset   -> GPIO 32 e GND  [facoltativo]

  Uso:
    1. Alimenta l'ESP32 dalla USB-C.
    2. Sul telefono collegati alla rete Wi-Fi "TesterEncoder" (password 12345678).
    3. Apri il browser all'indirizzo http://192.168.4.1
  Le stesse misure escono anche sul Monitor seriale (115200 baud).

  Nessuna libreria esterna: servono solo le schede "esp32" di Espressif
  (compatibile con il core 2.x e 3.x).

  Limiti:
    - I segnali sono letti con interrupt: oltre circa 20-30 kHz per canale
      l'ESP32 non riesce a seguire tutti i fronti e compaiono "errori".
    - Con il Voltage Sensor (divide per 5) e l'ADC a 11 dB la tensione
      massima misurabile e' circa 15 V.
*/

#include <WiFi.h>
#include <WebServer.h>
#include "soc/gpio_reg.h"
#include "esp_timer.h"

// ================== IMPOSTAZIONI ==================
const char* WIFI_NOME     = "TesterEncoder";
const char* WIFI_PASSWORD = "12345678";     // minimo 8 caratteri

const bool  USA_SENSORE_TENSIONE = true;    // false se non monti il Voltage Sensor
const float FATTORE_PARTITORE    = 5.0;     // il Voltage Sensor 0-25 V divide per 5
const bool  USA_PULSANTE         = true;    // false se non monti il pulsante di reset

// Impulsi per giro dell'encoder (PPR, canale A). 0 = non noto: i giri/min non vengono mostrati.
const uint32_t IMPULSI_GIRO = 0;

const int PIN_A        = 34;
const int PIN_B        = 35;
const int PIN_LED_A    = 25;
const int PIN_LED_B    = 26;
const int PIN_TENSIONE = 36;   // VP
const int PIN_PULSANTE = 32;

const uint32_t FINESTRA_MS      = 500;      // ogni quanto si aggiornano le misure
const uint32_t DURATA_MAX_US    = 2000000;  // impulsi/periodi piu' lunghi sono scartati (encoder fermo)
const uint32_t ANTIRIMBALZO_MS  = 50;       // antirimbalzo del pulsante
// ==================================================

WebServer server(80);
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Tabella quadratura: indice = (stato precedente << 2) | stato attuale, stato = (A << 1) | B
//  +1 / -1 = un passo avanti / indietro, 0 = nessun cambio, 2 = transizione impossibile (errore)
// DRAM_ATTR: la tabella deve stare in RAM perche' viene letta dentro l'interrupt.
static const int8_t DRAM_ATTR QT[16] = { 0, 1, -1, 2,  -1, 0, 2, 1,  1, 2, 0, -1,  2, -1, 1, 0 };

// --- stato di un canale, aggiornato negli interrupt (accesso solo dentro la sezione critica) ---
struct Canale {
  uint8_t  liv;            // ultimo livello visto
  bool     salitaValida;   // tSalita contiene un fronte di salita reale
  uint32_t tSalita;        // istante dell'ultimo fronte di salita (us)
  uint32_t periodo;        // ultimo periodo valido (us), 0 = non disponibile
  uint32_t tot;            // impulsi totali (fronti di salita)
  // accumulatori della finestra corrente
  uint32_t fin;            // fronti di salita
  uint32_t sommaAlto, nAlto;        // durata dei livelli alti completi
  uint32_t sommaPeriodo, nPeriodo;  // durata dei periodi completi
};

static Canale chA, chB;
static uint8_t  statoPrec = 0;
static int32_t  posizione = 0;
static int32_t  passiFin = 0;          // passi nella finestra corrente (per senso e velocita')
static uint32_t errori = 0;
static uint32_t faseRif10 = 0;         // primo campione di fase della finestra (decimi di grado)
static int32_t  sommaFase10 = 0;       // scarti dal riferimento, per una media "circolare"
static uint32_t nFase = 0;

// --- risultati calcolati ogni finestra (usati solo dal loop) ---
float    freqA = 0, freqB = 0;
float    dutyA = -1, dutyB = -1;
float    faseGradi = -1;
float    tensione = -1;
float    passiSec = 0;
float    giriMin = -1;
int32_t  posMostrata = 0;
uint32_t errMostrati = 0, totAMostrati = 0, totBMostrati = 0;
const char* senso = "fermo";

// Legge A e B direttamente dai registri GPIO: piu' veloce di digitalRead,
// utilizzabile dall'interrupt e con i due livelli campionati nello stesso istante.
static inline uint8_t IRAM_ATTR livelloPin(int pin, uint32_t in0, uint32_t in1) {
  return (pin >= 32) ? ((in1 >> (pin - 32)) & 1) : ((in0 >> pin) & 1);
}

static inline void IRAM_ATTR leggiAB(uint8_t &a, uint8_t &b) {
  uint32_t in0 = REG_READ(GPIO_IN_REG);
#ifdef GPIO_IN1_REG
  uint32_t in1 = REG_READ(GPIO_IN1_REG);
#else
  uint32_t in1 = 0;
#endif
  a = livelloPin(PIN_A, in0, in1);
  b = livelloPin(PIN_B, in0, in1);
}

static void IRAM_ATTR aggiornaQuadratura(uint8_t a, uint8_t b) {
  uint8_t stato = (a << 1) | b;
  int8_t d = QT[(statoPrec << 2) | stato];
  if (d == 2) errori++;
  else { posizione += d; passiFin += d; }
  statoPrec = stato;
}

// Gestisce un eventuale fronte su un canale. Ritorna true se e' un fronte di salita.
static bool IRAM_ATTR gestisciFronte(Canale &c, uint8_t liv, uint32_t ora) {
  if (liv == c.liv) return false;
  c.liv = liv;
  if (liv) {                                       // fronte di salita
    if (c.salitaValida) {
      uint32_t p = ora - c.tSalita;
      if (p > 0 && p <= DURATA_MAX_US) {
        c.periodo = p;
        c.sommaPeriodo += p; c.nPeriodo++;
      } else {
        c.periodo = 0;                             // ripartenza dopo una sosta
      }
    }
    c.tSalita = ora;
    c.salitaValida = true;
    c.tot++; c.fin++;
    return true;
  }
  if (c.salitaValida) {                            // fronte di discesa
    uint32_t h = ora - c.tSalita;
    if (h <= DURATA_MAX_US) { c.sommaAlto += h; c.nAlto++; }
  }
  return false;
}

// Unico gestore per entrambi gli interrupt: campiona A e B insieme e processa
// i fronti di tutti e due i canali, cosi' nessun fronte viene "consumato" dal
// gestore dell'altro canale senza essere contato.
static void IRAM_ATTR gestisciEncoder() {
  uint32_t ora = (uint32_t)esp_timer_get_time();
  uint8_t a, b;
  leggiAB(a, b);
  portENTER_CRITICAL_ISR(&mux);
  aggiornaQuadratura(a, b);
  gestisciFronte(chA, a, ora);
  if (gestisciFronte(chB, b, ora) && chA.salitaValida && chA.periodo) {
    uint32_t dt = ora - chA.tSalita;               // sfasamento di B rispetto ad A
    if (dt < chA.periodo) {
      uint32_t f10 = (uint32_t)((uint64_t)dt * 3600ULL / chA.periodo);   // 0..3599
      if (nFase == 0) faseRif10 = f10;
      int32_t d = (int32_t)f10 - (int32_t)faseRif10;
      if (d > 1800) d -= 3600; else if (d < -1800) d += 3600;
      sommaFase10 += d;
      nFase++;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isrA() { gestisciEncoder(); }
void IRAM_ATTR isrB() { gestisciEncoder(); }

void azzera() {
  portENTER_CRITICAL(&mux);
  posizione = 0; errori = 0; chA.tot = 0; chB.tot = 0;
  portEXIT_CRITICAL(&mux);
  posMostrata = 0; errMostrati = 0; totAMostrati = 0; totBMostrati = 0;
}

float leggiTensione() {
  if (!USA_SENSORE_TENSIONE) return -1;
  uint32_t somma = 0;
  for (int i = 0; i < 16; i++) somma += analogReadMilliVolts(PIN_TENSIONE);
  return (somma / 16.0f) / 1000.0f * FATTORE_PARTITORE;
}

// Frequenza: dalla media dei periodi se disponibile (precisa anche a bassa velocita'),
// altrimenti dal conteggio degli impulsi nella finestra.
static float calcolaFreq(const Canale &c, uint32_t dtMs) {
  if (c.nPeriodo > 0 && c.sommaPeriodo > 0) return c.nPeriodo * 1e6f / c.sommaPeriodo;
  return c.fin * 1000.0f / dtMs;
}

// Duty: durata media del livello alto / periodo medio.
static float calcolaDuty(const Canale &c) {
  if (c.nAlto == 0 || c.nPeriodo == 0 || c.sommaPeriodo == 0) return -1;
  float d = ((float)c.sommaAlto / c.nAlto) / ((float)c.sommaPeriodo / c.nPeriodo) * 100.0f;
  return constrain(d, 0.0f, 100.0f);
}

static void azzeraFinestra(Canale &c) {
  c.fin = 0; c.sommaAlto = 0; c.nAlto = 0; c.sommaPeriodo = 0; c.nPeriodo = 0;
}

void calcolaFinestra(uint32_t dtMs) {
  Canale a, b;
  int32_t passi, sF;
  uint32_t nF, rif;

  portENTER_CRITICAL(&mux);
  a = chA; b = chB;
  azzeraFinestra(chA); azzeraFinestra(chB);
  passi = passiFin; passiFin = 0;
  sF = sommaFase10; nF = nFase; rif = faseRif10;
  sommaFase10 = 0; nFase = 0;
  posMostrata  = posizione;
  errMostrati  = errori;
  portEXIT_CRITICAL(&mux);
  totAMostrati = a.tot;
  totBMostrati = b.tot;

  freqA = calcolaFreq(a, dtMs);
  freqB = calcolaFreq(b, dtMs);
  dutyA = calcolaDuty(a);
  dutyB = calcolaDuty(b);

  if (nF > 0) {
    float f = rif / 10.0f + (sF / (float)nF) / 10.0f;
    if (f < 0) f += 360.0f; else if (f >= 360.0f) f -= 360.0f;
    faseGradi = f;
  } else {
    faseGradi = -1;
  }

  passiSec = passi * 1000.0f / dtMs;
  giriMin  = (IMPULSI_GIRO > 0) ? freqA * 60.0f / IMPULSI_GIRO : -1;

  if (passi > 0)      senso = "avanti (+)";
  else if (passi < 0) senso = "indietro (-)";
  else                senso = "fermo";

  tensione = leggiTensione();
}

// ------------------ PAGINA WEB ------------------
const char PAGINA[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="it"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tester encoder</title>
<style>
body{font-family:Arial,sans-serif;margin:0;padding:12px;background:#f2f4f5;color:#111}
h1{font-size:20px;margin:4px 0 12px}
#st{font-size:12px;font-weight:normal;margin-left:8px}
.g{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.c{background:#fff;border-radius:10px;padding:10px;box-shadow:0 1px 3px #0002}
.c.w{grid-column:1/3}
.l{font-size:12px;color:#555}
.v{font-size:24px;font-weight:bold;margin-top:2px}
.ok{color:#1a7f37}.ko{color:#c00000}
button{width:100%;margin-top:12px;padding:14px;font-size:18px;border:0;border-radius:10px;background:#1f4e9c;color:#fff}
.n{font-size:12px;color:#555;margin-top:10px;line-height:1.5}
</style></head><body>
<h1>Tester encoder<span id="st"></span></h1>
<div class="g">
 <div class="c w"><div class="l">Tensione encoder</div><div class="v" id="v">--</div></div>
 <div class="c"><div class="l">Livello A</div><div class="v" id="la">--</div></div>
 <div class="c"><div class="l">Livello B</div><div class="v" id="lb">--</div></div>
 <div class="c"><div class="l">Impulsi A</div><div class="v" id="ia">--</div></div>
 <div class="c"><div class="l">Impulsi B</div><div class="v" id="ib">--</div></div>
 <div class="c"><div class="l">Frequenza A</div><div class="v" id="fa">--</div></div>
 <div class="c"><div class="l">Frequenza B</div><div class="v" id="fb">--</div></div>
 <div class="c"><div class="l">Duty A</div><div class="v" id="da">--</div></div>
 <div class="c"><div class="l">Duty B</div><div class="v" id="db">--</div></div>
 <div class="c"><div class="l">Sfasamento A-B</div><div class="v" id="ph">--</div></div>
 <div class="c"><div class="l">Errori</div><div class="v" id="er">--</div></div>
 <div class="c"><div class="l">Posizione (passi)</div><div class="v" id="po">--</div></div>
 <div class="c"><div class="l">Senso</div><div class="v" id="se">--</div></div>
 <div class="c"><div class="l">Velocit&agrave; (passi/s)</div><div class="v" id="vp">--</div></div>
 <div class="c"><div class="l">Giri/min</div><div class="v" id="rpm">--</div></div>
</div>
<button onclick="fetch('/reset').then(agg)">Azzera contatori</button>
<div class="n">Valori attesi: tensione 4,75-5,25 V &middot; duty 40-60% &middot; sfasamento circa 90&deg; (o 270&deg; nell'altro senso) &middot; errori 0.<br>
Encoder a un canale (TAU): conta solo A; B, sfasamento e posizione non servono.<br>
Giri/min: imposta IMPULSI_GIRO nello sketch.</div>
<script>
function s(id,t,c){var e=document.getElementById(id);e.textContent=t;e.className='v'+(c===undefined?'':(c?' ok':' ko'));}
function f(x){return x.toFixed(1).replace('.',',');}
var occ=false,ko=0;
function agg(){if(occ)return;occ=true;
 fetch('/dati',{cache:'no-store'}).then(r=>r.json()).then(d=>{
 ko=0;document.getElementById('st').textContent='';
 if(d.v<0)s('v','non montato');else s('v',f(d.v)+' V',d.v>=4.75&&d.v<=5.25);
 s('la',d.la?'ALTO':'BASSO');s('lb',d.lb?'ALTO':'BASSO');
 s('ia',d.ia);s('ib',d.ib);
 s('fa',f(d.fa)+' Hz');s('fb',f(d.fb)+' Hz');
 if(d.da<0)s('da','--');else s('da',f(d.da)+' %',d.da>=40&&d.da<=60);
 if(d.db<0)s('db','--');else s('db',f(d.db)+' %',d.db>=40&&d.db<=60);
 if(d.ph<0)s('ph','--');else s('ph',f(d.ph)+'°',(d.ph>=70&&d.ph<=110)||(d.ph>=250&&d.ph<=290));
 s('er',d.er,d.er==0);s('po',d.po);s('se',d.se);
 s('vp',f(d.vp));s('rpm',d.rpm<0?'--':f(d.rpm));
}).catch(()=>{if(++ko>=3){var e=document.getElementById('st');e.textContent='connessione persa';e.className='ko';}})
 .finally(()=>{occ=false;});}
setInterval(agg,500);agg();
</script></body></html>
)HTML";

void paginaPrincipale() { server.send_P(200, "text/html", PAGINA); }

void inviaDati() {
  char json[400];
  snprintf(json, sizeof(json),
    "{\"v\":%.2f,\"la\":%d,\"lb\":%d,\"ia\":%lu,\"ib\":%lu,\"fa\":%.1f,\"fb\":%.1f,"
    "\"da\":%.1f,\"db\":%.1f,\"ph\":%.1f,\"er\":%lu,\"po\":%ld,\"se\":\"%s\","
    "\"vp\":%.1f,\"rpm\":%.1f}",
    tensione, digitalRead(PIN_A), digitalRead(PIN_B),
    (unsigned long)totAMostrati, (unsigned long)totBMostrati, freqA, freqB,
    dutyA, dutyB, faseGradi, (unsigned long)errMostrati, (long)posMostrata, senso,
    passiSec, giriMin);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void richiestaReset() { azzera(); server.send(200, "text/plain", "ok"); }

// Pulsante fisico: reset sul fronte di pressione, con antirimbalzo.
void controllaPulsante() {
  if (!USA_PULSANTE) return;
  static bool statoStabile = HIGH, letturaPrec = HIGH;
  static uint32_t tCambio = 0;
  bool lettura = digitalRead(PIN_PULSANTE);
  if (lettura != letturaPrec) { letturaPrec = lettura; tCambio = millis(); }
  if (lettura != statoStabile && millis() - tCambio >= ANTIRIMBALZO_MS) {
    statoStabile = lettura;
    if (statoStabile == LOW) {
      azzera();
      Serial.println("Contatori azzerati.");
    }
  }
}

// ------------------ SETUP / LOOP ------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_A, INPUT);          // GPIO 34/35: solo ingresso, pull-up esterni
  pinMode(PIN_B, INPUT);
  pinMode(PIN_LED_A, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  if (USA_PULSANTE) pinMode(PIN_PULSANTE, INPUT_PULLUP);
  if (USA_SENSORE_TENSIONE) analogSetPinAttenuation(PIN_TENSIONE, ADC_11db);

  uint8_t a, b;
  leggiAB(a, b);
  chA.liv = a;
  chB.liv = b;
  statoPrec = (a << 1) | b;
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), isrB, CHANGE);

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(WIFI_NOME, WIFI_PASSWORD)) {
    Serial.println("ERRORE: impossibile avviare il Wi-Fi (password di almeno 8 caratteri?)");
  }
  server.on("/", paginaPrincipale);
  server.on("/dati", inviaDati);
  server.on("/reset", richiestaReset);
  server.onNotFound([]() { server.send(404, "text/plain", "non trovato"); });
  server.begin();

  Serial.println();
  Serial.println("Tester encoder pronto.");
  Serial.print("Wi-Fi: "); Serial.print(WIFI_NOME);
  Serial.print("  -  pagina: http://"); Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  // LED che seguono i segnali A e B
  digitalWrite(PIN_LED_A, digitalRead(PIN_A));
  digitalWrite(PIN_LED_B, digitalRead(PIN_B));

  controllaPulsante();

  // Calcolo misure ogni FINESTRA_MS
  static uint32_t tFin = millis();
  uint32_t ora = millis();
  if (ora - tFin >= FINESTRA_MS) {
    uint32_t dt = ora - tFin;
    tFin = ora;
    calcolaFinestra(dt);

    Serial.printf("V=%.2f  A:%lu imp %.1f Hz duty %.1f  |  B:%lu imp %.1f Hz duty %.1f  |  fase %.1f  pos %ld  %.1f passi/s  err %lu  %s\n",
      tensione, (unsigned long)totAMostrati, freqA, dutyA,
      (unsigned long)totBMostrati, freqB, dutyB,
      faseGradi, (long)posMostrata, passiSec, (unsigned long)errMostrati, senso);
  }
  delay(1);   // lascia respirare gli altri task (Wi-Fi, watchdog)
}
