#!/usr/bin/env python3
"""gz-hitl-bridge smoke test: acts as a mock FC.
Prerequisites: gz running headless with x500 spawned, bridge running with
--udp 127.0.0.1:24580 --local-port 24540.
Usage: python3 hitl_smoke_test.py
"""
import math
import time
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:127.0.0.1:24580')
print("waiting for bridge heartbeat...")
conn.wait_heartbeat(timeout=10)

# HIL_SENSOR fields_updated bits for XMAG|YMAG|ZMAG (see mavlink common.xml HIL_SENSOR_UPDATED_FLAGS)
MAG_BITS = 0x01C0

# 1) HIL_SENSOR reception rate (5s, >=150Hz); also capture zacc and the last mag sample that
#    actually carried a mag update (fields_updated only includes MAG bits on the gz frames that
#    triggered a magnetometer callback, since HIL_SENSOR is otherwise built off the IMU callback).
t0, n, zacc, mag = time.time(), 0, None, None
while time.time() - t0 < 5.0:
    m = conn.recv_match(type='HIL_SENSOR', blocking=True, timeout=1.0)
    if m:
        n += 1
        zacc = m.zacc
        if m.fields_updated & MAG_BITS == MAG_BITS:
            mag = (m.xmag, m.ymag, m.zmag)
rate = n / 5.0
assert rate >= 150, f"HIL_SENSOR rate {rate:.0f}Hz < 150Hz"
print(f"PASS rate={rate:.0f}Hz")

# 2) Zero-motion gravity sign (FRD: zacc ~ -9.8)
assert zacc is not None and -11.0 < zacc < -8.5, f"zacc={zacc} (wrong FRD gravity sign?)"
print(f"PASS zacc={zacc:.2f}")

# 3) Magnetometer magnitude sanity. Earth's field is roughly 0.25-0.65 gauss at the surface; the
#    wider 0.05-1.5 gauss band tolerates whatever magnetic-field strength the gz world happens to
#    configure, while still catching a gross unit/scale bug (e.g. the earlier x1e4 tesla->gauss
#    conversion applied on top of gz's already-gauss output, which would land ~1e4 outside this band).
assert mag is not None, "no HIL_SENSOR message carried a magnetometer update"
mag_norm = math.sqrt(sum(c * c for c in mag))
assert 0.05 < mag_norm < 1.5, f"mag magnitude {mag_norm:.4f} gauss outside expected Earth-field range"
print(f"PASS mag={mag_norm:.3f}gauss")

# 4) HIL_GPS sanity
g = conn.recv_match(type='HIL_GPS', blocking=True, timeout=3.0)
assert g and g.fix_type == 3 and g.lat != 0, "HIL_GPS invalid"
print(f"PASS gps lat={g.lat/1e7:.5f}")

# 5) Actuator ramp send (2s, armed, 0.7) -- verified manually afterwards via
#    `gz topic -e -t /x500_0/command/motor_speed` (expect ~700 rad/s)
t0 = time.time()
while time.time() - t0 < 2.0:
    conn.mav.hil_actuator_controls_send(
        int(time.time() * 1e6), [0.7] * 16, 128 | 4, 0)
    time.sleep(0.004)
print("sent actuator ramp — verify: gz topic -e -t /x500_0/command/motor_speed (~700 rad/s)")
print("CHECKS 1-4 PASS (check 5: verify motor speed manually via gz topic)")
