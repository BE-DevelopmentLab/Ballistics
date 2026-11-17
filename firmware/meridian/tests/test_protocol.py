#!/usr/bin/env python3
"""Checks tools/meridian_client.py against core/meridian_protocol.h byte for byte, via tests/proto_tool."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import meridian_client as mc  # noqa: E402

TOOL = os.path.join(HERE, "proto_tool")
fails = passes = 0


def check(ok, name):
    global fails, passes
    print(("[PASS] " if ok else "[FAIL] ") + name)
    if ok:
        passes += 1
    else:
        fails += 1


def run(*args):
    return subprocess.run([TOOL, *args], capture_output=True, text=True, check=True).stdout.strip()


enc = dict(line.split() for line in run("encode").splitlines())
s = mc.unpack_shot(bytes.fromhex(enc["shot"]))
check(s["range_m"] == 612.3 and s["range_age_s"] == 4.2 and s["battery_pct"] == 77, "shot: range, age, battery")
check(s["pitch_deg"] == -3.21 and s["cant_deg"] == 1.57 and s["bearing_deg"] == 271.43 and s["true_north"], "shot: angles")
check(s["temp_c"] == -4.25 and s["pressure_hpa"] == 845.6 and s["rh_pct"] == 33.3, "shot: atmosphere")
check(s["range_seq"] == 200 and s["mag_cal"] == 3 and s["heading_acc_deg"] == 4, "shot: seq, compass")
p = mc.unpack_position(bytes.fromhex(enc["position"]))
check(abs(p["lat_deg"] - 45.1234567) < 1e-9 and abs(p["lon_deg"] + 111.7654321) < 1e-9, "position: lat/lon")
check(p["alt_m"] == 1523 and p["fix"] == 1 and p["sats"] == 9 and p["hdop"] == 0.87 and p["utc"] == 1794772800
      and p["declination_deg"] == 11.25, "position: other fields")
t = mc.unpack_status(bytes.fromhex(enc["status"]))
check(t["version"] == mc.VERSION and t["mount"] == 10 and t["units"] == "MOA" and t["pitch_zero"] == 0.81 and t["cant_zero"] == -1.3,
      "status: version, mount, units, level zero")
check(t["heading_offset"] == 93.5 and t["declination"] == 11.5 and t["laser_offset_m"] == 0.3 and t["target_mode"] == "last"
      and t["last_corr_seq"] == 42 and t["centrals"] == 2 and t["laser"] == "idle", "status: other fields")

g = mc.unpack_target(bytes.fromhex(enc["target"]))
check(g["slot"] == "B" and g["saved"] and g["true_north"] and g["gps"] and g["range_m"] == 845.6 and g["incline_deg"] == 3.05
      and g["bearing_deg"] == 271.43 and g["lat_deg"] == 45.5 and g["lon_deg"] == -111.75 and g["utc"] == 1794772800, "target: every field")
check(run("command", mc.pack_command("save_target", "C").hex()) == "16 0.000 2", "command: save target C")
check(run("command", mc.pack_command("save_target", "next").hex()) == "16 0.000 255", "command: save target next")
check(run("command", mc.pack_command("clear_targets", "all").hex()) == "17 0.000 255", "command: clear all targets")
check(run("command", mc.pack_command("send_targets").hex()) == "18 0.000 0", "command: send targets")
check(run("command", mc.pack_command("autosleep", 15).hex()) == "19 0.000 15", "command: autosleep 15")
c = mc.pack_correction(7.43, -1.27, range_m=803.5, tof_s=1.234, lead_mrad=2.5, hit_pct=64, target=3, wind_mps=4.47,
                       wind_from_deg=270, seq=513, miss_adjusted=True)
check(len(c) == 20, "correction: 20 bytes")
check(run("correction", c.hex()) == "7 7.43 -1.27 803.5 1.234 2.50 64 3 4.47 270 513", "correction: C++ decodes Python's packet")
check(run("correction", mc.pack_correction(1, 1, hit_pct=None).hex()).split()[6] == "255", "correction: no hit chance = 255")
check(run("command", mc.pack_command("declination", -12.5).hex()) == "5 -12.500 0", "command: declination")
check(run("command", mc.pack_command("aim", 270).hex()) == "6 270.000 0", "command: aim bearing")
check(run("command", mc.pack_command("target", 2).hex()) == "12 0.000 2", "command: target mode")
check(run("command", mc.pack_command("range").hex()) == "1 0.000 0", "command: range")
check(mc.pack_message("SEND IT") == b"SEND IT", "message: ascii")
try:
    mc.pack_message("x" * 21)
    check(False, "message: too long rejected")
except ValueError:
    check(True, "message: too long rejected")
print(f"protocol C++ <-> Python: {passes} passed, {fails} failed")
sys.exit(1 if fails else 0)
