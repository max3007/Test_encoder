#!/bin/sh
# Compila lo sketch sul PC con finte API Arduino (cartella mock/) e simula i segnali
# dell'encoder: frequenza, duty, sfasamento, senso, latenza degli interrupt.
# Uso: sh test/esegui_test.sh [percorso/sketch.ino]
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
SKETCH=${1:-$DIR/../TesterEncoder/TesterEncoder.ino}
g++ -std=gnu++17 -O2 -Wall -Wextra -Wno-unused-parameter -I"$DIR/mock" -include mock.h \
    -DSKETCH="\"$SKETCH\"" -o /tmp/simulatore_encoder "$DIR/simulatore.cpp"
/tmp/simulatore_encoder
