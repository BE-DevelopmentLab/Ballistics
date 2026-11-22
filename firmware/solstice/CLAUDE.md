# Solstice: ballistic calculator project brief

## Names (decided 2026-11-11)

- **Celestial** is the overall project. **Solstice** is the V1 handheld (this build; earlier called "KB", "Kestrel Ballistic"
  or "BallisticCalc"). **Meridian** is the V2 rifle-mounted unit.
- The board advertises over Bluetooth as `Solstice` (`BLE_NAME` in `main.cpp`). The app title, header and manifest say Solstice.
  The app finds the board by its service UUID, not by name, so the name can change without breaking Connect.
- Left unchanged on purpose: the GitHub repo and Pages URL (`Ballistics`), the `ballistics-vN` cache prefix, the
  `BC_UUID_*` constants and the service UUID, and this folder's name.

A DIY handheld ballistic computer (Foretrex 701 style) with Kestrel-style weather sensing.
An ESP32-S3 board solves the shot. An Android phone app (installable PWA over Web Bluetooth) is the screen.

## Current state: read this first

- **Firmware first ran on hardware 2026-11-10** (bring-up steps 1 to 3 done). It builds and flashes cleanly, and
  the BME280 reads sane values over serial (22.3 C, 55 % RH, 998 hPa station pressure indoors). The firmware
  prints an `env ...` status line every 5 s so a late-attached monitor still sees sensor state.
  Phone connected over BLE 2026-11-10: env readings update live, config and request round trips give solutions
  (about 34 ms per solve on the board, no restarts). The range card works too (bring-up step 4 done). Wind step 5 done 2026-11-16 with a DRV5055 analog sensor on A5 (see Hardware); step 6 (calibration) is still open.
  Closing a .NET SerialPort on the board's USB port can trigger the core's esptool reset pattern and reboot it
  into download mode, which drops the phone. Use `pio device monitor` or leave the port alone during phone tests.
  NimBLE-Arduino 2.x does not advertise the name unless `adv->setName()` is called (now added). On Android 12+
  Chrome needs the Nearby devices permission or no device chooser shows.
- Flashing quirk: after upload, the "Hard resetting via RTS" leaves the S3 in ROM download mode (USB 303A:1001,
  new COM port). Get it running with `esptool.exe --chip esp32s3 --port <that COM> --after watchdog-reset chip-id`
  (esptool is in `~/.platformio/packages/tool-esptool`) or tap RESET. If the app's COM port says "device not
  functioning", hold BOOT, tap RESET, and upload again.
- The solver, protocol and phone page are tested on a normal computer only (`./run_tests.sh`).
  Run it after any change to `core/`, `web/` or the firmware logic. It must stay green.
- **The G1/G7 drag tables in `core/drag_tables.h` are unverified.** Do not trust the solver's output
  for real shooting until they are checked against published standard tables.
- `WIND_MPS_PER_HZ` (0.50) in `main.cpp` is a placeholder. It must be recalibrated once the real
  impeller's pulses per revolution are known.

## Hardware

- Adafruit Feather ESP32-S3 (PlatformIO board: `adafruit_feather_esp32s3`).
- BME280 on I2C, address 0x77 (0x76 if the jumper is cut): temperature, humidity, pressure.
- BNO085 (heading, pitch, cant): NOT part of version 1. Tilt and heading are meaningless on a handheld that is not on the rifle, so the BNO085 is reserved for the rifle-mounted unit in version 2. Version 1 runs without it.
- Wind: a Kestrel replacement impeller read by a DRV5055A1QLPG linear Hall sensor (60 mV/mT at 3.3 V, output
  centred ~1.65 V, no pull-up) on Feather A5 (`WIND_ADC_PIN`, GPIO 8, ADC1). Powered from the BME280 VIN with a
  0.1 uF bypass capacitor; the sensor is mounted flat against the side of the impeller housing (edge-on gave half the signal). Bench 2026-11-16: swing 64-71 mV spinning, 7-13 mV at rest. Detector thresholds: 30 mV swing gate, 12 mV hysteresis band, 5-sample smoothing. The firmware samples it at 2 kHz
  with an esp_timer and `core/wind_pulse.h` turns the wave into pulses (2 per revolution; tests in
  `tests/test_wind.cpp`). `WIND_ANALOG 0` in `main.cpp` switches back to the original DRV5033AJ Hall switch on A0
  (GPIO 18, 10K pull-up, falling edges, pulses closer than 1500 us ignored), which proved too insensitive for the
  impeller's magnet (2026-11-12). Flashed and bench tested 2026-11-16: pulses count while spinning and stay flat at rest. The serial `env` line
  now prints `pulses=` (running count) and `swing=` (wave peak-to-peak in mV; under 30 mV counts as no signal).
  `WIND_MPS_PER_HZ` is still a placeholder.
- **I2C power:** the Feather ESP32-S3 may need its STEMMA/NeoPixel I2C power pin (GPIO 7, `NEOPIXEL_I2C_POWER`)
  driven HIGH before the BME280 responds. `main.cpp` now drives it HIGH in `setup()` before `Wire.begin()`,
  and `tests/test_firmware.cpp` checks that it is HIGH when the BME280 starts.
- The DigiKey and Amazon parts may not all have arrived. Ask which are wired before assuming a sensor exists.

## Layout

- `core/ballistics.h`: point-mass RK4 solver with G1/G7 drag, spin drift (Miller stability + Litz),
  and Coriolis. Coriolis is applied only beyond 1000 yd (914 m).
- `core/protocol.h`: BLE packet layouts. **Must match `web/protocol.js` byte for byte.**
- `firmware/`: PlatformIO project (NimBLE-Arduino, Adafruit BME280, Adafruit BNO08x).
- `web/`: the phone app. `index.html`, `protocol.js`, `hitprob.js` (hit chance maths), `solver.js` (JS copy of the solver), `shotlog.js` (shot log, truing, chronograph stats), `groupcalc.js` (group maths and hole finder), `stage.js` (stage planner); none of these has page code, `sw.js` (offline cache), manifest, icons.
- `tests/`: host-side tests with stub hardware headers in `tests/stubs/`.

## BLE protocol

- Version 2. App and firmware must both be v2, so bump both together if the layout changes.
- Request/card flag bit5 (0x20) = apply aerodynamic jump. It used a reserved bit, so the layout and version
  did not change; an older board just ignores it.
- Request flag bit6 (0x40) = sleep: a one-shot command from the app's Sleep button (a plain valid request with only that bit set).
  The board answers by switching off the I2C rail (GPIO 7) and going into deep sleep; BOOT (GPIO 0) or RESET wakes it. It used a
  reserved bit, so no layout or version change; older boards ignore it. A sleeping board still charges.
- Service `b4a10000-1d5b-4c0e-9a3f-5e7d2c0a1b00`. Characteristics (suffix 0001 to 0005): config,
  request, solution, env, card.
- Env byte 15 = battery percent (0..100), valid only when env flags bit2 (0x04) is set; env flags bit3 (0x08) = charging. It used a reserved byte, so no layout or
  version change; an older app ignores it, and older firmware leaves bit2 clear so the app shows no battery instead of 0 %.
- Sizes: config 48 B, request 28 B, card 32 B, solution 20 B, env 16 B.
- Solstice keeps advertising after a connection (`ServerCb::onConnect` restarts it while fewer than
  `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` (3) are connected), so the phone and Meridian can both link to it; Meridian
  pulls Solstice's wind as a Bluetooth client. Added 2026-11-15 with Brad's OK; not yet flashed or tried on the board.

## Phone app hosting

- Live at https://be-developmentlab.github.io/Ballistics/ (GitHub Pages, repo BE-DevelopmentLab/Ballistics, branch `main`).
- The repo root holds the contents of `web/`.
- **When any web file changes, bump `CACHE` in `sw.js`** (currently `ballistics-v20`), or phones keep
  serving the old copy. After Pages rebuilds, the app must be opened twice on the phone to update.

## First-boot bring-up plan

1. `pio run`. Fix compile errors. Do not change `core/` behavior to make it compile without running the tests.
2. `pio run -t upload`. If the board is not detected, hold BOOT, tap RESET, then retry.
3. `pio device monitor` (115200). Confirm it boots, finds the BME280, and prints sane readings. The BNO085 is expected to be NOT FOUND in version 1 (firmware already falls back to manual slope and direction).
4. Connect from the phone app (Chrome on Android, HTTPS). Check that the config, request and card
   round trips work and the env readings update.
5. Wind: check that pulses are counted and that the reading falls to zero when the impeller stops.
6. Report pulses per revolution so `WIND_MPS_PER_HZ` can be set.

## Version 1 scope (current)

Build only the single handheld unit: ballistic computer with the wind impeller built in. Do not start any rifle-mounted or separate wind-unit work.

- In scope: firmware bring-up on the Feather ESP32-S3, BME280, the Hall wind sensor and impeller (all in one handheld), and the phone app. Slope and shooting direction are entered manually in the app.
- Out of scope until version 2: the BNO085 (heading, tilt, cant), the laser rangefinder, the radar chronograph, the rear display, any rifle-mounted unit, and splitting wind into a separate Bluetooth unit.
- The owner is checking the G1/G7 drag tables against published tables and will supply real load data.
  When reference cases arrive, add them to `tests/` as automatic checks and keep `./run_tests.sh` green.
- Do not treat the solver as validated until it matches the owner's reference values and range-tested impacts.

- Done (2026-11-10): slope and shooting direction default to manual entry. The sensor options are shown
  but disabled ("version 2"), older saved settings are switched to manual on load, and the app never sends
  the IMU flags while `HAS_ORIENTATION_SENSOR` in `index.html` is false. `CACHE` is now `ballistics-v4`.
- Done (2026-11-10): moving target lead calculator in the app (Shot > Moving target). Lead = target speed x
  (time of flight + shooter delay ms), converted to MIL/MOA with the solution's range, shown as a lead line plus
  total hold with wind, and as a LEAD column on the range card. It runs on the phone from the board's TOF, so the
  BLE protocol and firmware did not change. Tests are in `tests/test_page.js` ("lead:"). Pushed to GitHub, `CACHE` is `ballistics-v5`.

## Solver upgrades (AB-style physics, from published methods)

Applied Ballistics' solver code and custom drag models are proprietary: do not copy them. Use the published methods (McCoy, *Modern Exterior Ballistics*; Litz, *Applied Ballistics for Long Range Shooting*).

Status (2026-11-10): items 1 to 3 are done in `core/` with tests. Truing is in the solver only: the phone app
has no way to run it yet (that needs a new BLE message, so ask before adding it). Item 4 waits on the owner's data.

1. Spin drift: `core/ballistics.h` already has a Litz spin drift (`spin_drift_m`). Confirm it uses the Litz form (drift grows with stability and with time of flight to roughly the 1.83 power) and that stability comes from the Miller twist rule (needs bullet length, diameter, weight, twist, and an atmosphere correction). Add a unit test.
2. Aerodynamic jump (Litz): add the vertical correction caused by crosswind, driven by stability and crosswind speed. Make it a switch like `spin_drift`, include it in the solution, and add a unit test.
3. Truing: add muzzle-velocity truing from one observed drop at a mid range, then a drop scale factor (or BC adjustment) from an observed drop past about 600 yd. Store the trued values in the rifle/load profile. Add tests with synthetic data (truing a known trajectory must recover the true MV and BC).
4. Reference checks: when the owner supplies chronograph MV and observed drops, run the same inputs through the free Applied Ballistics online calculator and add both as test cases in `tests/`.

Any new config fields change the BLE config characteristic. Ask before changing the protocol version, and update the app and firmware together.

## Owner's load (reference load for tests)

- Barnes Precision Match .223 Rem, 77 gr Match Burner OTM BT (Barnes SKU 32018)
- Bullet: G1 0.349 (0.352 for the bare bullet), G7 0.176 (use G7), diameter 0.224 in, length 0.990 in, minimum twist 1:8
- Factory velocity 2670 to 2680 fps (test barrel length not stated). The owner shoots a 16 in barrel, so the real MV will be lower: chronograph value to come.
- Still needed from the owner: barrel twist, scope height over bore, zero range, chronograph MV, observed drops.

## Working rules

- Prefer small changes, and run `./run_tests.sh` before saying anything works.
- Never claim a sensor or the solver is correct from compile success alone. Say what was actually
  observed on the board.
- Ask before pushing to GitHub or changing the protocol version.
- Done (2026-11-10): app layout reworked. Compact top bar (title, status pill, Connect). Demo mode removed.
  Simple / Advanced tabs: Simple shows saved profile (picker only), solution, shot (range, wind speed, wind
  direction, units) and live conditions; Advanced shows everything. Wind direction is a drag compass in degrees
  (wind FROM, relative to the line of fire, 0 = head, 90 = from the right), sent in the existing
  wind-clock-degrees field, so no protocol change. Old saved clock values convert on load. `CACHE` is `ballistics-v6`.
- Done (2026-11-10): temperature is always °F (live readout and zero temperature) whatever Units says. Rifle and
  load fields are always imperial (fps, in, °F, inHg, grains, in); only zero range follows Units. Older metric
  saves are converted once on load (`rifleImp` flag in saved settings). `CACHE` is `ballistics-v7`.
- Done (2026-11-11): battery bar in the app's top bar (percent, amber at 30% or less, red at 15% or less, hidden until
  connected). The board has a MAX17048 fuel gauge (I2C 0x36, shared with the BME280; library `adafruit/Adafruit MAX1704X`)
  that gives the real state of charge and voltage, sent in env byte 15. If the gauge is not found, the firmware falls
  back to the VBAT pin (`BATT_PIN` GPIO 12 = A13, 1/2 divider, **unverified**) and `protocol::battery_percent_from_mv`
  (LiPo curve, an estimate). The serial `env` line prints `batt=<mV> <pct>% (gauge|pin)` and boot prints whether the
  MAX17048 was found. Built, flashed and seen on the board 2026-11-11: `env` line shows `batt=4066 mV 85% (gauge)`, so the
MAX17048 is found and in use; BME280 still reads 22.4 C, 54.6 %, 998.1 hPa. `CACHE` is `ballistics-v9`.
- Done (2026-11-11): charging bolt on the app's battery icon. The MAX17048 charge-rate register is flat (-4.2 %/h) on this
  board even while charging, so the firmware's `ChargeDetector` (main.cpp) works it out from the cell-voltage trend instead:
  one point per 30 s over the last 4.5 min, on when the voltage rises (+3 mV over 4.5 min, +4 mV over 2 min, or a +20 mV step
  in 30 s), off when it falls, and a full cell (4.15 V or more, not falling) on USB counts as charging. It is a few minutes
  late to switch. The thresholds are estimates: tune them on the board. The serial `env` line still prints `rate=` for diagnosis.
  Env flags bit3 (0x08) carries it. `CACHE` is `ballistics-v11`.
- Done (2026-11-11): Sleep button at the very bottom of the app (Power section, both tabs, two taps to confirm) until a real
  power switch is fitted. Firmware deep sleep with ext0 wake on GPIO 0. Not yet compiled with pio or tried on the board
  (the `esp_sleep` / `gpio_hold_en` calls are the likely first compile issues; also measure the sleeping current).
  `CACHE` is `ballistics-v11`.
- Done (2026-11-11): fixed a stale-app bug. GitHub Pages sends max-age=600, so the service worker's addAll could store the browser's cached OLD index.html under the NEW cache name (phone kept showing old text after an update). sw.js now installs with new Request(f, { cache: 'reload' }), and tests/test_pwa.js checks it. CACHE is ballistics-v13.
- Done (2026-11-11): renamed V1 to Solstice (app title, header, manifest, BLE advertised name). tests/test_pwa.js and
  tests/test_firmware.cpp check the name and that Connect filters on the service UUID. CACHE is ballistics-v14.
- Done (2026-11-15): hit chance calculator (Future idea 1), phone side only, no protocol or firmware change. `web/hitprob.js`
  holds the maths with no page code so the spotter unit can reuse it. It takes the board's solution (range, TOF, velocity at
  target) plus MV, zero range and sight height, works out how far the impact moves per unit error (wind: lag-time rule, exact;
  range: slope of a drop curve fitted to TOF and remaining velocity; MV: drop x 1/MV^2 x 1.15), then a seeded Monte Carlo
  (20,000 shots) counts hits on a round or rectangular target. Inputs (1 SD): group size MOA + shots per group (ES to sigma:
  3 shots 2.41, 5 shots 3.06, 10 shots 3.81), velocity SD fps, wind call error, range error. `tests/hit_ref.cpp` prints the
  real solver's sensitivities and `tests/test_hitprob.js` checks hitprob.js against them (wind within 2%, range within 7%,
  MV within 15%) and the Monte Carlo against exact formulas. UI: Advanced > Hit chance, and a one-line result in Solution on
  both tabs. `CACHE` is `ballistics-v15`.
- Done (2026-11-15): Load development tab (third tab; Brad asked for it by name). Shows the saved profile picker plus:
  Load (powder, charge weight gr, notes; saved with each logged shot and load test), Shot log (range, elevation and windage
  used, impact high/low and right/left; logs the board's live conditions, or the zero conditions when not connected; CSV
  export; per-shot on/off for truing), True the load, Group from a photo, and Load tests (chronograph string + group,
  compared in a table). Storage keys: `bc.shots.v1`, `bc.loadtests.v1`.
  Truing runs on the phone (Brad chose this over a new BLE message): `web/solver.js` is a line-for-line JS port of
  `core/ballistics.h` + `drag_tables.h`, **keep it in step with any core/ change**. `tests/solver_ref.cpp` +
  `tests/test_solver_js.js` check the two agree (double: 0.002 mrad, float build like the board: 0.02 mrad) and that JS
  truing matches `true_muzzle_velocity` / `true_load`. Shots 300-600 yd true MV, past 600 yd true BC, each in its own
  logged conditions; Apply writes MV/BC into the form, re-saves the active profile and sends it if connected.
  Group from a photo: Take photo (file input with capture="environment", opens the camera) or Choose photo; tap two points
  a known distance apart for scale, Find holes (`GroupCalc.findHoles`: dark round spots about one bullet wide, from the
  profile's bullet diameter, else .224), tap to add/remove, Aim point for the offset. Gives ES, mean radius, size and
  offset in in and MOA; Use in hit chance copies the size. `tests/test_group.js` checks the maths and the finder on a
  drawn target. `CACHE` is `ballistics-v16`. Hit chance stays in Advanced.
- Done (2026-11-15, Brad's changes): truing now uses every shot from 300 yd out (Brad: BC from 300-600 yd, not only past 600).
  `ShotLog.trueFromLog` is a least-squares fit with a +/- (1 SD). "Muzzle velocity is: From a chronograph" fits the BC only
  (firm: +/-0.1 mil impact errors move BC about 0.003); "Estimated" fits MV and BC together when the shots span 150 yd or
  more (rough from 300-600 yd: the same errors move BC about 0.02, so the app warns), else MV only. App wording says
  "Solstice", not "the board" (test checks). Group from a photo: drag circles, scale points and aim point (pointer events;
  a press that moves under 8 px is a tap), Clear marks, and Remove photo. `CACHE` is `ballistics-v17`.
- Done (2026-11-15): Stage tab (fourth tab; Future idea 7 without GPS/heading/laser). Shows the profile picker, Stage planner
  and Live conditions. Targets are typed in (name, range, round or rectangle size, shots). Each is solved on the phone with
  `solver.js` (Solstice's live conditions, or the zero conditions when not connected; wind from the Shot section) and gets a
  hit chance from `hitprob.js` using Advanced > Hit chance settings. Run order: as entered (arrows), near to far, far to
  near, best odds first; each card shows UP/wind, TOF, the dial change from the previous target, and flags targets below
  the set hit chance. Hit/Miss buttons record results; the summary compares hits with what the plan expected and names
  targets that went much worse. Stages save by name (`bc.stages.v1`; an unsaved one is kept as a draft). Refreshes every
  15 s while the tab is open. Tests: `tests/test_stage.js` and the "stage" checks in `tests/test_page.js`. `CACHE` is `ballistics-v18`.
- Done (2026-11-15): Bullet section on the Load development tab (Brad): bullet name, weight (gr), size (diameter in), type,
  sectional density (worked out: gr / 7000 / dia^2, read-only), BC and model. Saved with the form, with each logged shot
  (`bullet` field, CSV columns bullet..bc_model) and each load test (shown under the powder). Copy from profile / Use in
  profile; Use in profile only copies weight and diameter when bullet length and twist are already filled (spin-drift data
  is all or nothing). `CACHE` is `ballistics-v19`.
- Done (2026-11-22): BC uncertainty in hit chance (Brad). New input "BC uncertainty (%)" (default 2). `HitProb.bcSensitivity` solves
  the phone's solver copy at BC +/-1% in the current conditions (checked against the C++ solver within 3% by `tests/hit_ref.cpp`),
  and the BC spread is added to the vertical spread like MV spread. It also applies on the Stage tab. Hit chance stays for the one
  current target only (no range chart); the note gives the target size in MOA/MIL and says when it is smaller than the likely
  spread. Tests check the target size decides it (a 2 MOA rifle on a 0.5 MOA plate is under 15%). `CACHE` is `ballistics-v20`.
