# Tester encoder – ESP32

Sketch Arduino per ESP32 DevKit che verifica un encoder incrementale (canali A/B)
e mostra le misure su una pagina web (Wi-Fi "TesterEncoder", http://192.168.4.1)
e sul Monitor seriale. Collegamenti e uso sono descritti in testa a
`TesterEncoder/TesterEncoder.ino`.

## Misure
Tensione di alimentazione, livelli A/B, impulsi, frequenza, duty, sfasamento A-B,
errori di quadratura, posizione, senso di rotazione, velocità (passi/s) e
giri/min (impostando `IMPULSI_GIRO`).

## Test sul PC
`sh test/esegui_test.sh` compila lo sketch con g++ usando finte API Arduino
(`test/mock/`) e simula segnali in quadratura, comprese pause, bassa velocità
e latenza degli interrupt. Non sostituisce la prova sulla scheda.
