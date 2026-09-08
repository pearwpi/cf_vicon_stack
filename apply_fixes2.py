#!/usr/bin/env python3
"""Second fix batch. Run from the cf_vicon_stack root:  python3 apply_fixes2.py

1. --step METRES        a step-response mode that logs, so loop dead time and
                        the loop-fit yaw error can finally be measured
2. post-mortem z        stop reporting a landed drone as "z out of bounds"
3. frame_check          refuse to average two slides taken in different mocap
                        solver branches, instead of inventing a "mirrored axis"
4. tests                pin all three

All-or-nothing: anchors are checked before anything is written, originals go to
_backup_<timestamp>/, and nothing is written that does not parse.
"""
import ast
import os
import shutil
import subprocess
import sys
import time

FILES = ["crazyflie_vicon_teleop.py", "frame_check.py", "test_crazyflie_ros.py"]
for f in FILES:
    if not os.path.exists(f):
        sys.exit(f"not found: {f}\nRun from the cf_vicon_stack root.")
src = {f: open(f).read() for f in FILES}
edits = []

T = "crazyflie_vicon_teleop.py"

# ---- 1a. the CLI flag -------------------------------------------------------
A_OLD = '''    ap.add_argument("--diagnose", type=float, metavar="SECONDS", default=0.0,
                    help="auto: take off, hold, record SECONDS of hover to "
                         "hover_log.csv, land, then analyse and report why it "
                         "drifts")'''
A_NEW = A_OLD + '''
    ap.add_argument("--step", type=float, metavar="METRES", default=0.0,
                    help="with --diagnose: move the hold point this far along "
                         "world +X at 1/3 of the capture and back at 2/3. The "
                         "delay between the setpoint moving and the drone "
                         "moving is the loop dead time -- the only way to "
                         "measure real end-to-end latency. 0.30 is a good "
                         "value; keep --volume wide enough to contain it")'''
edits.append((T, A_OLD, A_NEW, "--step flag", 'ap.add_argument("--step"'))

# ---- 1b. state --------------------------------------------------------------
S_OLD = """        self._diag_started = False
        self._diag_t0: float | None = None"""
S_NEW = """        self._diag_started = False
        self._diag_t0: float | None = None
        self._diag_step_n = 0
        self._diag_hold_x0: float | None = None"""
edits.append((T, S_OLD, S_NEW, "step state", "_diag_step_n"))

# ---- 1c. drive the steps ----------------------------------------------------
D_OLD = """            if self._diag_t0 is None:
                self._diag_t0 = now
                print("[diagnose] settled -- recording")
            self.record_sample(now)
            if now - self._diag_t0 >= secs:"""
D_NEW = """            if self._diag_t0 is None:
                self._diag_t0 = now
                self._diag_step_n = 0
                self._diag_hold_x0 = self.hold_x
                print("[diagnose] settled -- recording")
            self._maybe_step(now - self._diag_t0)
            self.record_sample(now)
            if now - self._diag_t0 >= secs:"""
edits.append((T, D_OLD, D_NEW, "step driver", "self._maybe_step("))

# ---- 1d. the step itself ----------------------------------------------------
M_OLD = "    def record_sample(self, now: float):"
M_NEW = '''    def _maybe_step(self, elapsed: float):
        """Move the hold point out at 1/3 of the capture and back at 2/3.

        WHY THE HOLD POINT AND NOT sp_x. Moving hold_x keeps the setpoint on
        the ordinary _drive_to_hold path, so the geofence clamp, the leash and
        the supervisor all still apply. A diagnostic that bypasses the safety
        layer measures a configuration you never actually fly.

        The move is therefore a rate-limited ramp at POS_SPEED_MS, not an
        instantaneous jump. That is correct for real hardware and it does not
        harm the measurement: dead time is read from the ONSET, the delay
        between the setpoint starting to move and the position starting to
        move. Time-to-settle would mix in controller dynamics; onset does not.
        """
        d = float(getattr(self.args, "step", 0.0) or 0.0)
        if d == 0.0 or self._diag_hold_x0 is None:
            return
        secs = float(getattr(self.args, "diagnose", 0.0) or 0.0)
        if secs <= 0.0:
            return

        if self._diag_step_n == 0 and elapsed >= secs / 3.0:
            target = self._diag_hold_x0 + d
            # Refuse rather than let the geofence silently clamp it: a clipped
            # step is a step of unknown size, which is worthless for timing.
            if not (self.bounds.xmin <= target <= self.bounds.xmax):
                print(f"\\n[diagnose] STEP SKIPPED -- target x={target:+.2f} m is "
                      f"outside {self.bounds.describe()}. Use a smaller --step "
                      f"or a wider --volume.")
                self._diag_step_n = 2
                return
            self._diag_step_n = 1
            self.hold_x = target
            print(f"\\n[diagnose] STEP +{d:.2f} m along world +X  "
                  f"(hold_x {self._diag_hold_x0:+.2f} -> {self.hold_x:+.2f})")
        elif self._diag_step_n == 1 and elapsed >= 2.0 * secs / 3.0:
            self._diag_step_n = 2
            self.hold_x = self._diag_hold_x0
            print(f"\\n[diagnose] STEP back to {self.hold_x:+.2f} m")

    def record_sample(self, now: float):'''
edits.append((T, M_OLD, M_NEW, "_maybe_step", "def _maybe_step"))

# ---- 2. post-mortem z -------------------------------------------------------
P_OLD = """            for name, lo, hi, val in (("x", self.bounds.xmin, self.bounds.xmax, p.x),
                                      ("y", self.bounds.ymin, self.bounds.ymax, p.y),
                                      ("z", self.bounds.zmin, self.bounds.zmax, p.z)):
                if val < lo or val > hi:
                    over = (lo - val) if val < lo else (val - hi)
                    print(f"    -> {name} out of bounds by {over:+.2f} m")"""
P_NEW = """            # z deliberately has NO lower bound here, matching
            # Bounds.outside(): zmin floors the COMMANDED altitude, not the
            # measured one, so a drone resting on the floor is inside the
            # volume. Checking z against zmin made every landed post-mortem
            # report a violation that the supervisor does not agree exists.
            for name, lo, hi, val in (("x", self.bounds.xmin, self.bounds.xmax, p.x),
                                      ("y", self.bounds.ymin, self.bounds.ymax, p.y),
                                      ("z", None, self.bounds.zmax, p.z)):
                below = lo is not None and val < lo
                if below or val > hi:
                    over = (lo - val) if below else (val - hi)
                    print(f"    -> {name} out of bounds by {over:+.2f} m")"""
edits.append((T, P_OLD, P_NEW, "post-mortem z", "z deliberately has NO lower bound"))

# ---- 3. frame_check cross-branch guard --------------------------------------
F = "frame_check.py"
F_CONST_OLD = "MIN_YAW_SPAN_FOR_FIT_DEG = 90.0"
F_CONST_NEW = """MIN_YAW_SPAN_FOR_FIT_DEG = 90.0
# The drone is not rotated between the two slides, so the yaw it REPORTS must
# agree between them. More than this and the mocap solver changed solution
# between the slides. Well under MIN_FLIP_DEG (45) so it catches a flip, well
# over the few degrees of sloppy repositioning by hand.
SLIDE_YAW_AGREEMENT_DEG = 20.0"""
edits.append((F, F_CONST_OLD, F_CONST_NEW, "frame_check constant",
              "SLIDE_YAW_AGREEMENT_DEG"))

F_OLD = '''    usable = [r for r in (nose, left) if r and "yaw_error_deg" in r]'''
F_NEW = '''    # CROSS-BRANCH GUARD. Nothing rotates the drone between the two slides, so
    # the yaw it reports must be the same in both. If it is not, the solver
    # picked a different solution between them and the two measurements live in
    # different branches. Averaging them yields a confident number describing
    # neither, and their disagreement then looks like a mirrored axis.
    #
    # This is not hypothetical: on 2026-08-28 a run reported yaw -92.5 deg
    # during the nose slide and +0.6 deg during the left slide, and this
    # function averaged +95.3 and -2.8 into a "CONSTANT YAW OFFSET of +46 deg"
    # and diagnosed a "MIRRORED or swapped axis". Both were artefacts. The
    # evidence to reject the run was already in hand.
    branch_gap = None
    if (nose and left and "mean_yaw_deg" in nose and "mean_yaw_deg" in left):
        branch_gap = abs(wrap180(nose["mean_yaw_deg"] - left["mean_yaw_deg"]))
        L.append(f"  branch check   : reported yaw differs by {branch_gap:.1f} deg "
                 f"between the two slides")

    usable = [r for r in (nose, left) if r and "yaw_error_deg" in r]

    if branch_gap is not None and branch_gap > SLIDE_YAW_AGREEMENT_DEG:
        problems.append((
            "PRIMARY",
            f"CONTAMINATED RUN -- reported yaw differs by {branch_gap:.0f} deg "
            f"between the two slides on a drone that was never rotated. The "
            f"mocap solver changed solution partway through.",
            "The slides are in different solution branches, so neither the "
            "offset nor any disagreement between them means anything. Fix the "
            "marker ambiguity first (see the flips line above), then repeat. "
            "Do not act on any number from this run."))
        usable = []          # refuse to derive an offset from mixed branches'''
edits.append((F, F_OLD, F_NEW, "frame_check cross-branch guard", "CROSS-BRANCH GUARD"))

# ---- 4. tests ---------------------------------------------------------------
TA = 'print("\\n" + "=" * 62)\nprint(f"{PASS} passed, {FAIL} failed")'
TN = '''print("\\n[L] --step moves the hold point, and refuses to leave the volume")
_te = T.Teleop.__new__(T.Teleop)
_te.args = types.SimpleNamespace(diagnose=30.0, step=0.30)
_te.bounds = C.Bounds(-1.0, 1.0, -1.0, 1.0, 0.05, 1.2)
_te.hold_x = 0.0
_te._diag_hold_x0 = 0.0
_te._diag_step_n = 0

_te._maybe_step(1.0)
check("no step before 1/3 of the capture", _te.hold_x == 0.0, f"{_te.hold_x}")
_te._maybe_step(10.1)
check("steps out at 1/3", abs(_te.hold_x - 0.30) < 1e-9, f"{_te.hold_x}")
_te._maybe_step(12.0)
check("does not step twice", abs(_te.hold_x - 0.30) < 1e-9, f"{_te.hold_x}")
_te._maybe_step(20.1)
check("steps back at 2/3", abs(_te.hold_x - 0.0) < 1e-9, f"{_te.hold_x}")

# A step that would leave the geofence must be refused, not silently clamped:
# a clipped step is a step of unknown size and useless for timing.
_te2 = T.Teleop.__new__(T.Teleop)
_te2.args = types.SimpleNamespace(diagnose=30.0, step=5.0)
_te2.bounds = C.Bounds(-1.0, 1.0, -1.0, 1.0, 0.05, 1.2)
_te2.hold_x = 0.0
_te2._diag_hold_x0 = 0.0
_te2._diag_step_n = 0
_te2._maybe_step(10.1)
check("an out-of-volume step is refused, not clamped", _te2.hold_x == 0.0,
      f"{_te2.hold_x}")
check("...and it does not retry later", (_te2._maybe_step(20.1) or True)
      and _te2.hold_x == 0.0)

_te3 = T.Teleop.__new__(T.Teleop)
_te3.args = types.SimpleNamespace(diagnose=30.0, step=0.0)
_te3.bounds = C.Bounds(-1.0, 1.0, -1.0, 1.0, 0.05, 1.2)
_te3.hold_x = 0.0
_te3._diag_hold_x0 = 0.0
_te3._diag_step_n = 0
_te3._maybe_step(25.0)
check("--step 0 is a plain hover capture, unchanged", _te3.hold_x == 0.0)

print("\\n[M] --frame-test must refuse a run that crossed solver branches")
import frame_check as FC                                      # noqa: E402
_nose = {"distance_m": 0.28, "travel_heading_deg": 2.8, "mean_yaw_deg": -92.5,
         "yaw_wobble_deg": 3.0, "dz_m": 0.0, "yaw_error_deg": 95.3, "ok": False}
_left = {"distance_m": 0.32, "travel_heading_deg": 87.9, "mean_yaw_deg": 0.6,
         "yaw_wobble_deg": 3.0, "dz_m": 0.0, "yaw_error_deg": -2.8, "ok": True}
_out = FC.report(_nose, _left, 0.465, None, 15602, 0.6)
check("the real 2026-08-28 run is now flagged CONTAMINATED", "CONTAMINATED" in _out)
check("...and no longer invents a +46 deg offset",
      "CONSTANT YAW OFFSET" not in _out)
check("...and no longer claims a mirrored axis", "MIRRORED" not in _out)
check("...and still reports the ambiguity that caused it",
      "rotationally ambiguous" in _out)

_ok_n = dict(_nose); _ok_n["mean_yaw_deg"] = -1.0; _ok_n["yaw_error_deg"] = -2.0
_ok_l = dict(_left); _ok_l["mean_yaw_deg"] = -1.4; _ok_l["yaw_error_deg"] = -2.8
_out2 = FC.report(_ok_n, _ok_l, 0.465, None, 0, 0.4)
check("a clean single-branch run still passes", "PASS --" in _out2, _out2[-200:])

''' + TA
edits.append(("test_crazyflie_ros.py", TA, TN, "step + frame_check tests",
              "[L] --step moves the hold point"))

# ---- apply ------------------------------------------------------------------
print("checking anchors...")
todo = []
for f, old, new, label, marker in edits:
    if marker in src[f]:
        print(f"  SKIP  {label} (already applied)"); continue
    if old not in src[f]:
        sys.exit(f"\nANCHOR NOT FOUND for '{label}' in {f}:\n{old[:120]}\n"
                 "Nothing written. Send this to Claude.")
    if src[f].count(old) != 1:
        sys.exit(f"\nANCHOR NOT UNIQUE for '{label}' in {f}. Nothing written.")
    print(f"  ok    {label}")
    todo.append((f, old, new))

if not todo:
    sys.exit("\nAlready applied; nothing to do.")

bak = "_backup_" + time.strftime("%Y%m%d-%H%M%S")
os.makedirs(bak, exist_ok=True)
for f in FILES:
    shutil.copy2(f, os.path.join(bak, f.replace("/", "__")))
print(f"\nbacked up to {bak}/")

out = dict(src)
for f, old, new in todo:
    out[f] = out[f].replace(old, new, 1)
for f, text in out.items():
    ast.parse(text)
    open(f, "w").write(text)
    print(f"  wrote {f}")

print("\nrunning the suites...")
bad = False
for t in ("test_frame_check.py", "test_tilt_origin_check.py",
          "test_flip_guard.py", "test_crazyflie_ros.py"):
    r = subprocess.run([sys.executable, t], capture_output=True, text=True)
    if r.returncode == 0:
        print(f"  {t:<28} {r.stdout.count(chr(10) + '  PASS')} passed")
    else:
        bad = True
        print(f"  {t:<28} FAILED")
        print("\n".join(r.stdout.strip().splitlines()[-15:]))
if bad:
    print(f"\nFailed. Restore from {bak}/")
    sys.exit(1)
print("\nAll green.")
