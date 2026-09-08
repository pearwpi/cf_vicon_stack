#!/usr/bin/env python3
"""READ-ONLY IMU probe: connects, logs the accelerometer, disconnects.

Sends no setpoints, spins no motors, injects nothing.

Gravity is an absolute vertical reference. If Vicon says the body is tilted and
the IMU says level, the tilt is baked into the Vicon rigid-body template -- the
same act that bakes in a tilt also bakes in a YAW offset, which is what rotates
the position loop and makes it spiral.

    python3 imu_tilt.py [uri] [n_samples]
"""
import math
import os
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from cf_core import tilt_deg_of, vicon_capture, wrap180, yaw_deg_of

TOPIC = os.environ.get("VICON_TOPIC", "/vicon/crazyflie1/crazyflie1")
URI = sys.argv[1] if len(sys.argv) > 1 else "radio://0/80/2M/E7E7E7E701"
N = int(sys.argv[2]) if len(sys.argv) > 2 else 300


def sd(vals, m):
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


cflib.crtp.init_drivers(enable_debug_driver=False)
print(f"[imu] scanning for {URI} ...")
found = cflib.crtp.scan_interfaces()
print(f"[imu] scan saw: {[f[0] for f in found] if found else 'nothing'}")

acc = []
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
    print("[imu] connected. NO motors, NO setpoints -- read only.")
    lg = LogConfig(name="acc", period_in_ms=20)
    for v in ("acc.x", "acc.y", "acc.z"):
        lg.add_variable(v, "float")
    with SyncLogger(scf, lg) as logger:
        for _, data, _ in logger:
            acc.append((data["acc.x"], data["acc.y"], data["acc.z"]))
            if len(acc) >= N:
                break
print(f"[imu] disconnected. {len(acc)} samples.")

n = len(acc)
ax, ay, az = (sum(s[i] for s in acc) / n for i in (0, 1, 2))
mag = math.sqrt(ax * ax + ay * ay + az * az)

print("\n=== RAW ACCELEROMETER (body frame, g) ===")
for name, v, m in (("acc.x", [s[0] for s in acc], ax),
                   ("acc.y", [s[1] for s in acc], ay),
                   ("acc.z", [s[2] for s in acc], az)):
    print(f"  {name} {m:+.4f}  (sd {sd(v, m):.4f})")
print(f"  |a|   {mag:.4f} g   (should be ~1.00 at rest)")
if abs(mag - 1.0) > 0.15:
    print("  WARNING: |a| far from 1 g -- drone was moving, or accel uncalibrated.")

# Convention-free: total tilt from vertical, no roll/pitch sign convention used.
tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(az) / mag))))
print("\n=== TILT FROM VERTICAL (convention-free) ===")
print(f"  IMU says total tilt   : {tilt:.2f} deg   "
      f"(azimuth in body frame {math.degrees(math.atan2(ay, ax)):+.1f} deg)")

# Read Vicon live rather than against a baked-in number: a hard-coded baseline
# goes stale the moment the rigid body is edited.
vicon_tilt = vicon_yaw = None
try:
    rows = vicon_capture(TOPIC, 3.0, quiet=True)
    if rows:
        quats = [r[5:9] for r in rows]
        yaws = [yaw_deg_of(*q) for q in quats]
        ref = sorted(yaws)[len(yaws) // 2]
        # Drop frames far from the modal yaw so one solver flip cannot drag the mean.
        keep = [q for q, y in zip(quats, yaws) if abs(wrap180(y - ref)) < 20.0]
        vicon_tilt = sum(tilt_deg_of(*q) for q in keep) / len(keep)
        vicon_yaw = ref
        dropped = len(quats) - len(keep)
        print(f"  VICON says total tilt : {vicon_tilt:.2f} deg   "
              f"(yaw {vicon_yaw:+.2f}, {len(keep)} frames"
              f"{f', {dropped} flipped dropped' if dropped else ''})")
        print(f"  disagreement          : {abs(tilt - vicon_tilt):.2f} deg")
    else:
        print("  VICON: no frames received -- is the rigid body tracked?")
except Exception as exc:
    print(f"  VICON: could not read live tilt ({exc})")

print("\n=== VERDICT ===")
if vicon_tilt is None:
    print("  No live Vicon comparison; only the IMU figure above is valid.")
elif abs(tilt - vicon_tilt) < 1.5:
    print(f"  IMU and Vicon AGREE to {abs(tilt-vicon_tilt):.2f} deg at this heading.")
    print("  Not proof on its own: a template tilt and a world tilt can cancel at")
    print("  one heading. Confirm with tilt_origin_check.py, which turns the drone.")
elif tilt < 2.0:
    print(f"  IMU says LEVEL ({tilt:.2f} deg); Vicon says {vicon_tilt:.2f} deg.")
    print("  -> That tilt is BAKED INTO the Vicon rigid body, not physical.")
    print("  -> Expect a constant YAW offset from the same cause. Quantify both")
    print("     with tilt_origin_check.py.")
else:
    print(f"  IMU ({tilt:.2f}) and Vicon ({vicon_tilt:.2f}) disagree and neither is")
    print("  level. The drone is not sitting flat, so this cannot be cleanly")
    print("  attributed. Re-run on a known-flat surface.")
