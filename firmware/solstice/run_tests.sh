#!/bin/sh
# Runs every test that works without hardware.
set -e
cd "$(dirname "$0")/tests"
g++ -std=c++17 -O2 -Wall -Wextra -I../core -DBALLISTICS_REAL=double test_solver.cpp -o t_double
g++ -std=c++17 -O2 -Wall -Wextra -I../core -DBALLISTICS_REAL=float  test_solver.cpp -o t_float
g++ -std=c++17 -O2 -Wall -Wextra -I../core -DBALLISTICS_REAL=float  proto_tool.cpp -o proto_tool
g++ -std=c++17 -O2 -Wall -Wextra -I../core -DBALLISTICS_REAL=double hit_ref.cpp -o hit_ref
g++ -std=c++17 -O2 -Wall -Wextra -I../core -DBALLISTICS_REAL=double solver_ref.cpp -o solver_ref
g++ -std=c++17 -O2 -Wall -Wextra -I../core -DBALLISTICS_REAL=float  solver_ref.cpp -o solver_ref_f
echo "== solver (double) =="; ./t_double | tail -1
echo "== solver (float) ==";  ./t_float  | tail -1
echo "== protocol C++ <-> JS =="; node test_protocol.js | tail -1
echo "== phone page logic ==";    node test_page.js     | tail -1
echo "== phone solver copy ==";  node test_solver_js.js | tail -1
echo "== shot log and truing =="; node test_shotlog.js | tail -1
echo "== group analyzer ==";      node test_group.js | tail -1
echo "== stage planner ==";       node test_stage.js | tail -1
echo "== hit probability ==";    node test_hitprob.js  | tail -1
echo "== install/offline files =="; node test_pwa.js | tail -1
g++ -std=c++17 -O1 -I../core -Istubs -DBALLISTICS_REAL=float test_firmware.cpp -o t_fw
echo "== firmware on host (stub hardware) =="; ./t_fw | tail -1; ./t_fw nocfg | tail -1
g++ -std=c++17 -O1 -Wall -D_USE_MATH_DEFINES test_wind.cpp -o t_wind
echo "== wind pulse detector =="; ./t_wind | tail -1
