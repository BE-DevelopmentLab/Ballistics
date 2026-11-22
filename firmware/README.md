# Celestial firmware

- `solstice/`: Solstice (V1 handheld) as on the laptop's `ballistic-calc/` folder, 2026-11-22: `core/`, `firmware/`
  (PlatformIO project, analog DRV5055 wind on A5), `tests/`, `run_tests.sh` and the project brief `CLAUDE.md`.
  The phone app is the repo root; the page tests expect it at `../web`, so to run `run_tests.sh` here, copy the
  repo root's app files into `firmware/solstice/web/` first (the laptop folder already has them there).
- `core/`: the solver and Bluetooth packets Meridian builds against (`ballistics.h`, `drag_tables.h`, `protocol.h`,
  `wind_pulse.h`). Same as `solstice/core/`; keep the two in step.
- `meridian/`: firmware for Meridian, the rifle-mounted unit. Start with `meridian/CLAUDE.md`.
