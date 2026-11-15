# Meridian: rifle-mounted unit (Celestial V2) firmware brief

Meridian is the rifle-mounted unit ("RMU") of the Celestial system. It measures range, atmosphere, rifle angles and
position, sends them to the camera spotter and the phone over Bluetooth, and shows the dial/hold the spotter sends
back. Solstice is the V1 handheld; the spotter is a Jetson Orin Nano with a camera. Say "Solstice" and "Meridian",
not "the board".

## Current state: read this first

- **Written 2026-11-15 before any Meridian parts arrived. Nothing here has run on hardware.** It has not been built
  with PlatformIO either (the cloud session could not download the toolchain). First job on the laptop: `pio run`.
- `./run_tests.sh` builds and runs the PC tests (136 checks: packets, laser frames, GPS sentences, rifle angles,
  screen text, the whole firmware loop against stub hardware, and the Python client against the C++ packets).
  It must stay green after any change.
- **The laser protocol is an assumption.** No ERDI document for the LRF0306C was found. The driver uses the
  protocol from ERDI's LRF0310C manual (1535 nm sister model): 115200 8N1, frames `EE 16 len 03 cmd params sum`.
  Get the LRF0306C manual from ERDI with the order and check it against `core/rangefinder.h`. The laser lives behind
  the `lrf::RangeFinder` interface, so a different protocol only changes that file.
- Compass, level and laser need real calibration once the parts are on the rifle (see Bring-up).
- Meridian reuses Solstice's solver and packets **unchanged** from `../core` (`ballistics.h`, `drag_tables.h`,
  `protocol.h`). The drag-table caveat from Solstice applies here too.

## Hardware (recommended RMU list from the cost doc)

| Part | Connection | Pins / address |
| --- | --- | --- |
| Adafruit Feather ESP32-S3, 8 MB flash, no PSRAM (MINI-1U module: **needs an antenna on the u.FL jack**) | | PlatformIO board `adafruit_feather_esp32s3_nopsram` |
| ERDI LRF0306C laser, 1535 nm | Serial1, 3.3 V logic | module TX -> Feather RX (GPIO 38), module RX -> Feather TX (GPIO 39), POWER_ON -> D6 (GPIO 6) |
| BME280 | STEMMA QT / I2C | 0x77 |
| BNO085 | STEMMA QT / I2C | 0x4A |
| PA1010D mini GPS | STEMMA QT / I2C | 0x10 |
| 1.3" 240x240 TFT (ST7789) | SPI | SCK 36, MOSI 35, CS D10 (GPIO 10), DC D9 (GPIO 9), RST to RESET, backlight D5 (GPIO 5) |
| RANGE button | to GND | D11 (GPIO 11): press = range, hold 1 s = continuous on/off |
| MODE button | to GND | D12 (GPIO 12): press = next page, hold 3 s = sleep; press again to wake |
| Battery gauge (on the Feather) | I2C | MAX17048 0x36, or LC709203F 0x0B on older boards (both handled) |

- GPIO 7 (`I2C_POWER`) switches the STEMMA QT power and must be HIGH before any I2C part answers (done in `setup()`).
- **Laser power:** do NOT run the laser from the Feather's 3.3 V pin. The cost doc lists 3 to 5 V; the LRF0310C sister
  module peaks at several watts. Feed it from BAT (or USB 5 V) through the power switch, and confirm the LRF0306C's
  supply range and peak current with ERDI first. Its serial pins are 3.3 V, which matches the Feather.
- If the laser's POWER_ON is not wired, tie it high and set `LASER_EN_PIN` to -1.
- All pins are in the "Hardware settings" block at the top of `firmware/src/main.cpp`.

## Bluetooth

Meridian advertises as `Meridian`. Up to 4 links at once (phone, spotter, and Meridian's own link to Solstice).

1. **Solstice service** (`b4a10000-...`), byte for byte the same as Solstice. Today's phone app connects to Meridian
   and works unchanged: profile, live solution, range card, env readings, battery, Sleep button. Meridian answers
   like Solstice, with its BNO085 behind the IMU slope/heading flags and Solstice's wind behind the sensor-wind flag.
   Request flag bit7 (`0x80`, was reserved) = use Meridian's laser range instead of the typed range. The current app
   never sets it and Solstice ignores it, so nothing changes for either.
2. **Meridian service** (`b4a20000-...`, `core/meridian_protocol.h`), for the spotter:
   - SHOT (notify 5 Hz and at once on each new range): range, range age, pitch, cant, bearing, temperature,
     pressure, RH, battery, range sequence, compass calibration.
   - POSITION (notify 1 Hz): GPS lat/lon/alt, fix, satellites, HDOP, UTC, declination in use.
   - CORRECTION (write): elevation and windage (mrad), range it was solved for, time of flight, lead, hit chance,
     target number, wind used, sequence. Meridian shows it for 2 minutes (amber after 30 s, or if it was solved for
     a range more than 5 m from the latest laser range).
   - COMMAND (write): range, continuous, level zero, declination, aim bearing, mount, compass calibration start/save,
     units, laser offset, first/last target, Solstice link on/off, sleep.
   - MESSAGE (write): up to 20 characters for the bottom line ("SEND IT").
   - STATUS (notify 1 Hz and after each command): calibration values, last command result, links, laser state.
3. **Wind from Solstice:** Meridian scans for a device named `Solstice` with the Solstice service and subscribes to
   its env packet. Solstice keeps advertising while a link is open (change approved by Brad 2026-11-15, in the
   laptop's Solstice `main.cpp`), so the phone and Meridian can both connect to it. Solstice firmware older than
   that stops advertising once the phone connects, and then Meridian can't find it.

`tools/meridian_client.py` (Python, `pip install bleak`) talks to Meridian from the laptop or the Jetson:
`scan`, `watch`, `range`, `correct 5.0 -1.2 --range 612`, `message "SEND IT"`, and every command. The spotter
software can import its `pack_correction()` / `unpack_shot()` functions.

## Screen

Page 1 (shooting): source (SPOTTER / MERIDIAN / NO SOLUTION) and units, battery and links, elevation and windage in
big text, range (or RANGING / NO TARGET / LASER FAULT), lead or hit chance, a cant bar (red past 1 degree), pitch and
bearing, the spotter's message. Page 2 (setup): sensors, GPS, laser, calibration. With no spotter, Meridian solves
for itself once the phone app has sent the rifle profile: laser range, BNO085 angles, GPS latitude, Solstice wind,
and the wind direction and spin/Coriolis/jump switches from the app's last request.

## Bring-up plan (when the parts arrive)

1. `cd meridian/firmware && pio run`. Fix compile errors without changing `core/` behaviour, then `./run_tests.sh`.
2. Flash with only the Feather and the screen connected. Check the start-up text, then that the phone app connects
   to "Meridian" and shows its BME280 readings.
3. Add the BME280, BNO085 and GPS. Serial monitor prints what was found and an `env` line every 5 s.
4. BNO085: raise the muzzle and check pitch goes positive; roll right and check cant goes positive; point roughly
   north and east. If pitch is backwards, the world frame is Z-down (see `core/orientation.h`). If the sensor is not
   flat with +Y toward the muzzle, send the right `mount` code.
5. Compass: away from steel, `magcal-start`, move the rifle in slow figure-8s until the calibration reads 3/3,
   `magcal-save`. Then set `declination` for the area, or aim at a known bearing and send `aim <bearing>`.
   Expect the steel of the rifle and scope to bend the compass: calibrate with Meridian on the rifle.
6. Level: with the scope's bubble level, send `level`.
7. Laser (once the LRF0306C manual is in hand): check the self-check reply in the serial log, range known distances
   (a 100 m and a 300+ m target), and set `laser-offset` if there is a constant error. Try first vs last target.
8. Solstice link: flash the updated Solstice, connect the phone to Solstice, then check Meridian's screen shows WIND too.
9. Spotter: `python tools/meridian_client.py correct 5.0 -1.2 --range <laser range>` and check the screen.

## Working rules

- Run `./run_tests.sh` before saying anything works, and never claim a part works from a compile alone.
- Ask Brad before changing anything in Solstice (`../core`, `../firmware`, `../web`) or the live app.
- Meridian protocol changes: update `core/meridian_protocol.h`, `tools/meridian_client.py` and the tests together,
  and bump `meridian::kVersion` if a layout changes.
