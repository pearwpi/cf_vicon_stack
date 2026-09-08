#!/usr/bin/env python3
"""Apply the post-2026-08-28 fixes. Run from the cf_vicon_stack root:

    python3 apply_fixes.py

Idempotent, all-or-nothing: every anchor is checked before anything is written,
and originals are backed up to ./_backup_<timestamp>/ .

  1. cf_core.occluded_sentinel()  -- the occluded-frame detector
  2. teleop + ROS driver          -- drop sentinel frames so the staleness
                                     watchdog can act on occlusion
  3. teleop docstring             -- --max-z is not a flag
  4. tests                        -- prove the guard, in the suite
"""
import ast
import os
import shutil
import subprocess
import sys
import time

FILES = ["cf_core.py", "crazyflie_vicon_teleop.py", "test_crazyflie_ros.py",
         "src/crazyflie_ros/crazyflie_ros/crazyflie_server.py"]

for f in FILES:
    if not os.path.exists(f):
        sys.exit(f"not found: {f}\nRun this from the cf_vicon_stack root.")

src = {f: open(f).read() for f in FILES}
edits = []          # (file, old, new, label, marker-present-only-after-apply)


# ---------------------------------------------------------------- 1. cf_core
CORE_ANCHOR = "@dataclass\nclass Pose:"
CORE_NEW = '''# The Vicon SDK returns translation [0,0,0] and quaternion [1,0,0,0] for a
# segment it cannot see. vicon_receiver never reads the Occluded flag, so the
# topic keeps publishing those AT FULL RATE and perfectly FRESH -- meaning no
# staleness watchdog can see an occlusion, and the position controller flies
# toward the Vicon origin at full authority. The quaternion even has norm 1, so
# the zero-norm guard passes it.
#
# Exact equality is deliberate. These are the SDK's literal sentinel values,
# not a measurement, and a real pose is never bit-exactly zero in all three
# axes with a bit-exact identity-about-X quaternion. Both halves are required
# so a drone genuinely sitting near the origin is never rejected.
def occluded_sentinel(x, y, z, qx, qy, qz, qw):
    """True if this frame is the SDK's occluded-segment sentinel."""
    return (x == 0.0 and y == 0.0 and z == 0.0
            and qx == 1.0 and qy == 0.0 and qz == 0.0 and qw == 0.0)


@dataclass
class Pose:'''
edits.append(("cf_core.py", CORE_ANCHOR, CORE_NEW, "cf_core.occluded_sentinel()",
              "def occluded_sentinel("))


# ----------------------------------------------------------------- 2. teleop
T_IMP_OLD = "    flip_check, prearm_check, apply_leash, clamp_alt,"
T_IMP_NEW = ("    flip_check, prearm_check, apply_leash, clamp_alt,\n"
             "    occluded_sentinel,")
edits.append(("crazyflie_vicon_teleop.py", T_IMP_OLD, T_IMP_NEW, "teleop import",
              "\n    occluded_sentinel,"))

T_INIT_OLD = "        self.total_yaw_rejects = 0"
T_INIT_NEW = ("        self.total_yaw_rejects = 0\n"
              "        self.total_occluded = 0\n"
              "        self._occl_warned = False")
edits.append(("crazyflie_vicon_teleop.py", T_INIT_OLD, T_INIT_NEW, "teleop counters",
              "self.total_occluded = 0"))

GUARD_COMMENT = '''        # An occluded segment arrives looking perfectly healthy: full rate,
        # fresh timestamp, unit-norm quaternion. Dropping it here is what lets
        # MOCAP_STALE_S and MOCAP_DEAD_S notice that the body is not tracked.
'''
T_CB_OLD = "        # Unit sanity: a Crazyflie flying in a room is never 100 m from origin."
T_CB_NEW = GUARD_COMMENT + '''        if occluded_sentinel(p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw):
            self.total_occluded += 1
            if not self._occl_warned:
                self._occl_warned = True
                self.get_logger().error(
                    "OCCLUDED: position exactly (0,0,0) with quaternion "
                    "exactly (1,0,0,0) is the Vicon SDK's not-tracked "
                    "sentinel, not a measurement. Dropping these frames so "
                    "the mocap watchdog can act. Fix marker visibility.")
            return

        # Unit sanity: a Crazyflie flying in a room is never 100 m from origin.'''
edits.append(("crazyflie_vicon_teleop.py", T_CB_OLD, T_CB_NEW, "teleop occlusion guard",
              "OCCLUDED: position exactly (0,0,0)"))

T_DOC_OLD = "props on, netted, --max-z 0.8."
T_DOC_NEW = "props on, netted, --volume 1.5 1.5 0.8."
edits.append(("crazyflie_vicon_teleop.py", T_DOC_OLD, T_DOC_NEW, "teleop --max-z docstring",
              "--volume 1.5 1.5 0.8."))


# ------------------------------------------------------------- 3. ROS driver
S_CB_OLD = "        # Unit sanity: a Crazyflie flying in a room is never 100 m from origin."
S_CB_NEW = '''        # Same occluded-segment guard as the standalone script; see
        # cf_core.occluded_sentinel for why exact equality is correct here.
        if C.occluded_sentinel(p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw):
            self._occluded += 1
            if not self._occl_warned:
                self._occl_warned = True
                self.get_logger().error(
                    "OCCLUDED: the Vicon SDK's not-tracked sentinel. Frames "
                    "dropped so the mocap watchdog can act.")
            return

        # Unit sanity: a Crazyflie flying in a room is never 100 m from origin.'''
edits.append(("src/crazyflie_ros/crazyflie_ros/crazyflie_server.py",
              S_CB_OLD, S_CB_NEW, "driver occlusion guard",
              "OCCLUDED: the Vicon SDK's not-tracked sentinel"))

S_INIT_OLD = "        self._yaw_rejects = 0"
S_INIT_NEW = ("        self._yaw_rejects = 0\n"
              "        self._occluded = 0\n"
              "        self._occl_warned = False")
edits.append(("src/crazyflie_ros/crazyflie_ros/crazyflie_server.py",
              S_INIT_OLD, S_INIT_NEW, "driver counters",
              "self._occluded = 0"))


# ------------------------------------------------------------------ 4. tests
TEST_ANCHOR = 'print("\\n" + "=" * 62)\nprint(f"{PASS} passed, {FAIL} failed")'
TEST_NEW = '''print("\\n[K] occluded frames must not reach the control loop")
# The SDK's not-tracked sentinel arrives at full rate, fresh, with a unit-norm
# quaternion, so every existing guard passes it and the drone would be
# commanded toward the Vicon origin. These pin the fix.
check("sentinel is detected", C.occluded_sentinel(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
check("a normal pose is not", not C.occluded_sentinel(0.9, -0.1, 0.5, 0.0, 0.0, 0.0, 1.0))
check("a drone genuinely AT the origin is not rejected",
      not C.occluded_sentinel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      "position alone must never be enough")
check("position near but not at zero is not rejected",
      not C.occluded_sentinel(1e-9, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
check("the sentinel quaternion has unit norm, so the zero-norm guard misses it",
      abs(C.quat_norm(1.0, 0.0, 0.0, 0.0) - 1.0) < 1e-12)
check("...and reads as exactly 180 deg tilt",
      abs(C.tilt_deg_of(1.0, 0.0, 0.0, 0.0) - 180.0) < 1e-9)


class _OccMsg:
    def __init__(s, x, y, z, q, stamp=1000.0):
        s.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y, z=z),
            orientation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))
        s.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=int(stamp),
                                        nanosec=int((stamp % 1) * 1e9)))


_src = T.ViconSource.__new__(T.ViconSource)
_src._init_state(240.0)
_src.get_logger = lambda: types.SimpleNamespace(
    info=lambda *a: None, warn=lambda *a: None,
    error=lambda *a: None, fatal=lambda *a: None)

_src.position_callback(_OccMsg(0.9, -0.1, 0.5, (0.0, 0.0, 0.0, 1.0)))
_good = _src.latest()
check("a good frame is accepted", _good is not None and abs(_good.x - 0.9) < 1e-9)

_src.position_callback(_OccMsg(0.0, 0.0, 0.0, (1.0, 0.0, 0.0, 0.0)))
_after = _src.latest()
check("an occluded frame is DROPPED, not stored",
      _after is not None and abs(_after.x - 0.9) < 1e-9,
      f"latest became {(_after.x, _after.y, _after.z) if _after else None}")
check("...and is counted", getattr(_src, "total_occluded", 0) == 1,
      f"count {getattr(_src, 'total_occluded', None)}")

''' + TEST_ANCHOR
edits.append(("test_crazyflie_ros.py", TEST_ANCHOR, TEST_NEW, "occlusion tests",
              "[K] occluded frames must not reach the control loop"))


# ------------------------------------------------------------------- apply
print("checking anchors...")
already = 0
todo = []
for f, old, new, label, marker in edits:
    # The replacement text re-appends the anchor, so the anchor is STILL there
    # after a successful apply. Detecting "already applied" by the anchor would
    # re-apply on every run and duplicate the guard. Use a marker that only
    # exists afterwards.
    if marker in src[f]:
        print(f"  SKIP  {label} (already applied)"); already += 1; continue
    todo.append((f, old, new, label))
    if old not in src[f]:
        sys.exit(f"\nANCHOR NOT FOUND for '{label}' in {f}:\n    {old[:80]}\n"
                 "Nothing has been written. Send this message to Claude.")
    if src[f].count(old) != 1:
        sys.exit(f"\nANCHOR NOT UNIQUE for '{label}' in {f} "
                 f"({src[f].count(old)} matches). Nothing written.")
    print(f"  ok    {label}")

if not todo:
    sys.exit("\nAll fixes already applied; nothing to do.")

stamp = time.strftime("%Y%m%d-%H%M%S")
bak = f"_backup_{stamp}"
os.makedirs(bak, exist_ok=True)
for f in FILES:
    d = os.path.join(bak, f.replace("/", "__"))
    shutil.copy2(f, d)
print(f"\nbacked up to {bak}/")

out = dict(src)
for f, old, new, label in todo:
    out[f] = out[f].replace(old, new, 1)
for f, text in out.items():
    ast.parse(text)                      # never write something that will not parse
    open(f, "w").write(text)
    print(f"  wrote {f}")

print("\nrunning the suites...")
fail = False
for t in ("test_frame_check.py", "test_tilt_origin_check.py",
          "test_flip_guard.py", "test_crazyflie_ros.py"):
    r = subprocess.run([sys.executable, t], capture_output=True, text=True)
    n = r.stdout.count("\n  PASS")
    if r.returncode == 0:
        print(f"  {t:<28} {n} passed")
    else:
        fail = True
        print(f"  {t:<28} FAILED")
        print("\n".join(r.stdout.strip().splitlines()[-15:]))

if fail:
    print(f"\nSuites failed. Restore with:  cp {bak}/* . 2>/dev/null; "
          f"see {bak}/ for the originals.")
    sys.exit(1)
print("\nAll green. Occlusion guard is in and pinned by tests.")
