# Tester encoder – ESP32

Sketch Arduino per ESP32 DevKit che verifica un encoder incrementale (canali A/B)
e mostra le misure su una pagina web (Wi-Fi "TesterEncoder", http://192.168.4.1)
e sul Monitor seriale. Collegamenti e uso sono descritti in testa a
`TesterEncoder/TesterEncoder.ino`.

Su Android 17 o successivo il browser deve avere il permesso di accesso alla rete
locale (Impostazioni > App > Chrome > Autorizzazioni). Senza, Chrome non apre la
pagina e mostra `ERR_TOO_MANY_RETRIES`, anche se la rete Wi-Fi e' collegata.

## Misure
Tensione di alimentazione, livelli A/B, impulsi, frequenza, duty, sfasamento A-B,
errori di quadratura, posizione, senso di rotazione, velocità (passi/s) e
giri/min (impostando `IMPULSI_GIRO`).

## Test sul PC
`sh test/esegui_test.sh` compila lo sketch con g++ usando finte API Arduino
(`test/mock/`) e simula segnali in quadratura, comprese pause, bassa velocità
e latenza degli interrupt. Non sostituisce la prova sulla scheda.

## Build con PlatformIO
```sh
pio run                 # compila (env esp32dev)
pio run -t upload       # carica sulla scheda
pio device monitor      # Monitor seriale a 115200 baud
```
`platformio.ini` compila `pio/main.cpp`, che include `TesterEncoder/TesterEncoder.ino`:
lo sketch resta unico e apribile anche con Arduino IDE. Il wrapper serve perche' il
convertitore `.ino` di PlatformIO si confonde con la pagina HTML nella raw string.
Nel wrapper manca la generazione automatica dei prototipi: nello sketch ogni funzione
va definita prima di essere usata.
