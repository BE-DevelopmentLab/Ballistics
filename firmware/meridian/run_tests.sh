#!/bin/sh
# Meridian: every test that runs without hardware. Needs g++ and python3.
# Meridian reuses Solstice's solver and packets from ../core (the Solstice folder), unchanged.
set -e
cd "$(dirname "$0")/tests"
INC="-I../core -I../../core"
g++ -std=c++17 -O1 -Wall -Wextra $INC -DBALLISTICS_REAL=double test_core.cpp -o t_core
g++ -std=c++17 -O1 -Wall $INC -Istubs -DBALLISTICS_REAL=float test_firmware.cpp -o t_fw
g++ -std=c++17 -O1 -Wall $INC -DBALLISTICS_REAL=float proto_tool.cpp -o proto_tool
fail=0
run() { name="$1"; shift; echo "== $name =="; if "$@" > last.log 2>&1; then tail -1 last.log; else grep FAIL last.log; tail -1 last.log; fail=1; fi; }
run "packets, laser, GPS, angles, screen" ./t_core
run "firmware on the PC (stub hardware)" ./t_fw
run "protocol C++ <-> Python client" python3 test_protocol.py
rm -f last.log
exit $fail
