#!/usr/bin/env python3
"""Meridian Bluetooth client for the laptop or the Jetson spotter.

The packet code here mirrors core/meridian_protocol.h byte for byte (tests/test_protocol_py.sh checks it).
The spotter software can import this file and use pack_correction() / unpack_shot() etc.

Bench use (needs `pip install bleak`):
  python meridian_client.py scan                         list Meridian units nearby
  python meridian_client.py watch                        print shot, position and status packets
  python meridian_client.py range                        fire the laser once and print the result
  python meridian_client.py correct 5.0 -1.2 --range 612 --hit 70
                                                         send a spotter correction (mil, + = up / right)
  python meridian_client.py message "SEND IT"
  python meridian_client.py level | clear-level | clear-heading | magcal-start | magcal-save | sleep
  python meridian_client.py declination 11.5 | aim 270 | mount 10 | units moa | laser-offset 0.3 |
                            target first|last | continuous on|off | solstice on|off
"""
import argparse
import asyncio
import struct
import sys

SERVICE = "b4a20000-1d5b-4c0e-9a3f-5e7d2c0a1b00"
SHOT = "b4a20001-1d5b-4c0e-9a3f-5e7d2c0a1b00"
POSITION = "b4a20002-1d5b-4c0e-9a3f-5e7d2c0a1b00"
CORRECTION = "b4a20003-1d5b-4c0e-9a3f-5e7d2c0a1b00"
COMMAND = "b4a20004-1d5b-4c0e-9a3f-5e7d2c0a1b00"
MESSAGE = "b4a20005-1d5b-4c0e-9a3f-5e7d2c0a1b00"
STATUS = "b4a20006-1d5b-4c0e-9a3f-5e7d2c0a1b00"
VERSION = 1

# SHOT flags
RANGE_VALID, IMU_OK, BME_OK, GPS_FIX, TRUE_NORTH, RANGING, CHARGING, LASER_FAULT = (1 << i for i in range(8))
# CORRECTION flags
CORR_VALID, CORR_MISS_ADJUSTED, CORR_HAS_LEAD, CORR_CLEAR = 0x01, 0x02, 0x04, 0x80
# COMMAND codes
CMD = dict(range=1, continuous=2, level=3, clear_level=4, declination=5, aim=6, mount=7, magcal_start=8,
           magcal_save=9, units=10, laser_offset=11, target=12, solstice=13, sleep=14, clear_heading=15)


def _clamp(v, lo, hi):
    return max(lo, min(hi, int(round(v))))


def unpack_shot(b):
    (flags, batt, rng, age, pitch, cant, brg, temp, press, rh, seq, cal) = struct.unpack("<BBHHhhHhHHBB", bytes(b))
    acc = cal >> 2
    return dict(flags=flags, battery_pct=None if batt == 255 else batt,
                range_m=rng / 10 if flags & RANGE_VALID else None, range_age_s=None if age == 65535 else age / 10,
                pitch_deg=pitch / 100, cant_deg=cant / 100, bearing_deg=brg / 100, true_north=bool(flags & TRUE_NORTH),
                temp_c=temp / 100, pressure_hpa=press / 10, rh_pct=rh / 100, range_seq=seq,
                mag_cal=cal & 3, heading_acc_deg=None if acc == 63 else acc)


def unpack_position(b):
    lat, lon, alt, fix, sats, hdop, utc, decl = struct.unpack("<iihBBHIh", bytes(b))
    return dict(lat_deg=lat / 1e7, lon_deg=lon / 1e7, alt_m=alt, fix=fix, sats=sats, hdop=hdop / 100,
                utc=utc, declination_deg=decl / 100)


def unpack_status(b):
    (ver, flags, mount, units, p0, c0, hoff, decl, loff, tmode, res, seq, centrals, laser) = struct.unpack("<BBBBhhhhhBBHBB", bytes(b))
    return dict(version=ver, flags=flags, mount=mount, units="MOA" if units else "mil", pitch_zero=p0 / 100,
                cant_zero=c0 / 100, heading_offset=hoff / 100, declination=decl / 100, laser_offset_m=loff / 100,
                target_mode="last" if tmode == 2 else "first", last_result=res, last_corr_seq=seq,
                centrals=centrals, laser=["absent", "idle", "busy", "continuous", "fault"][laser] if laser < 5 else laser)


def pack_correction(elev_mrad, wind_mrad, range_m=0.0, tof_s=0.0, lead_mrad=None, hit_pct=None, target=0,
                    wind_mps=0.0, wind_from_deg=0.0, seq=0, miss_adjusted=False, valid=True, clear=False):
    flags = (CORR_VALID if valid else 0) | (CORR_MISS_ADJUSTED if miss_adjusted else 0) | \
            (CORR_HAS_LEAD if lead_mrad is not None else 0) | (CORR_CLEAR if clear else 0)
    return struct.pack("<BBhhHHhBBHhH", VERSION, flags,
                       _clamp(elev_mrad * 100, -32768, 32767), _clamp(wind_mrad * 100, -32768, 32767),
                       _clamp(range_m * 10, 0, 65535), _clamp(tof_s * 1000, 0, 65535),
                       _clamp((lead_mrad or 0) * 100, -32768, 32767), 255 if hit_pct is None else _clamp(hit_pct, 0, 100),
                       target, _clamp(wind_mps * 100, 0, 65535), _clamp(wind_from_deg, -32768, 32767), seq & 0xFFFF)


def pack_command(name, value=None):
    code = CMD[name]
    if name in ("declination", "aim", "laser_offset"):
        return struct.pack("<Bf", code, float(value))
    if name in ("continuous", "solstice", "units", "target", "mount"):
        return struct.pack("<BB", code, int(value))
    return struct.pack("<B", code)


def pack_message(text):
    data = text.encode("ascii")
    if len(data) > 20 or any(c < 0x20 or c > 0x7E for c in data):
        raise ValueError("message must be up to 20 printable ASCII characters")
    return data


# ---- Bluetooth (bleak) --------------------------------------------------------------------------------
async def _find(name):
    from bleak import BleakScanner
    devs = await BleakScanner.discover(timeout=5.0, service_uuids=None)
    found = [d for d in devs if (d.name or "") == name]
    if not found:
        sys.exit(f"No device named {name} found. Is it on and not already connected to two devices?")
    return found[0]


async def main_async(a):
    from bleak import BleakClient, BleakScanner
    if a.cmd == "scan":
        for d in await BleakScanner.discover(timeout=5.0):
            if d.name in ("Meridian", "Solstice"):
                print(d.address, d.name)
        return
    dev = await _find(a.name)
    async with BleakClient(dev) as c:
        if a.cmd == "watch":
            await c.start_notify(SHOT, lambda _, b: print("shot", unpack_shot(b)))
            await c.start_notify(POSITION, lambda _, b: print("position", unpack_position(b)))
            await c.start_notify(STATUS, lambda _, b: print("status", unpack_status(b)))
            await asyncio.sleep(a.seconds)
            return
        if a.cmd == "range":
            first = unpack_shot(await c.read_gatt_char(SHOT))["range_seq"]
            done = asyncio.Event()

            def on_shot(_, b):
                s = unpack_shot(b)
                if s["range_seq"] != first:
                    print(f"range {s['range_m']} m  pitch {s['pitch_deg']:+.2f}  cant {s['cant_deg']:+.2f}  bearing {s['bearing_deg']:.1f}")
                    done.set()
            await c.start_notify(SHOT, on_shot)
            await c.write_gatt_char(COMMAND, pack_command("range"), response=True)
            try:
                await asyncio.wait_for(done.wait(), 3.0)
            except asyncio.TimeoutError:
                print("no new range (no target, laser fault, or laser not answering); status:",
                      unpack_status(await c.read_gatt_char(STATUS)))
            return
        if a.cmd == "correct":
            await c.write_gatt_char(CORRECTION, pack_correction(a.elev, a.wind, range_m=a.range, hit_pct=a.hit, seq=a.seq), response=True)
        elif a.cmd == "message":
            await c.write_gatt_char(MESSAGE, pack_message(a.text), response=True)
        else:
            name = a.cmd.replace("-", "_")
            value = a.value
            if name in ("continuous", "solstice"):
                value = 1 if value == "on" else 0
            elif name == "units":
                value = 1 if value.lower() == "moa" else 0
            elif name == "target":
                value = 2 if value == "last" else 1
            await c.write_gatt_char(COMMAND, pack_command(name, value), response=True)
        await asyncio.sleep(0.3)
        print("status", unpack_status(await c.read_gatt_char(STATUS)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", default="Meridian")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    w = sub.add_parser("watch"); w.add_argument("--seconds", type=float, default=30)
    sub.add_parser("range")
    c = sub.add_parser("correct"); c.add_argument("elev", type=float); c.add_argument("wind", type=float)
    c.add_argument("--range", type=float, default=0); c.add_argument("--hit", type=int); c.add_argument("--seq", type=int, default=1)
    m = sub.add_parser("message"); m.add_argument("text")
    for n in ("level", "clear-level", "clear-heading", "magcal-start", "magcal-save", "sleep"):
        sub.add_parser(n)
    for n in ("declination", "aim", "mount", "units", "laser-offset", "target", "continuous", "solstice"):
        sub.add_parser(n).add_argument("value")
    a = p.parse_args()
    if a.cmd in ("mount",):
        a.value = int(a.value)
    asyncio.run(main_async(a))


if __name__ == "__main__":
    main()
