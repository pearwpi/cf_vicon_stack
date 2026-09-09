#!/usr/bin/env python3
"""Validation for tilt_origin_check.py.

Builds synthetic rigid bodies with a KNOWN injected defect and checks the tool
names the right one. An unvalidated diagnostic is worse than none: it points
confidently at the wrong subsystem.

Construction (both cases place the drone physically LEVEL, only the reported
quaternion differs):

  Case A -- template misaligned by fixed M (template <- body):
      body level in world  =>  R_wb = Rz(psi)
      R_wt = R_wb @ M^T = Rz(psi) @ M^T
      then u = R_wt^T z = M @ Rz(psi)^T z = M z   -- CONSTANT

  Case B -- Vicon world tilted by fixed T (vicon <- true):
      body level in TRUE world  =>  R_true = Rz(psi)
      R_wt = T @ Rz(psi)
      then w = R_wt z = T @ Rz(psi) z = T z       -- CONSTANT
"""
import os
import io
import math
import sys
import contextlib
import time

# The suites live in tests/ but import the modules under test from the
# repository root, so the root has to be on sys.path however this file
# is invoked -- "python3 tests/x.py", "python3 -m tests.x" or pytest.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import tilt_origin_check as TOC

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name} {detail}")
    else:
        FAIL += 1
        print(f"  FAIL  {name} {detail}")


# ---- small rotation helpers ------------------------------------------------
def matmul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def transpose(A):
    return tuple(tuple(A[j][i] for j in range(3)) for i in range(3))


def Rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def axis_angle(axis, deg):
    n = math.sqrt(sum(a * a for a in axis))
    x, y, z = (a / n for a in axis)
    t = math.radians(deg)
    c, s, C = math.cos(t), math.sin(t), 1 - math.cos(t)
    return ((c + x*x*C,   x*y*C - z*s, x*z*C + y*s),
            (y*x*C + z*s, c + y*y*C,   y*z*C - x*s),
            (z*x*C - y*s, z*y*C + x*s, c + z*z*C))


def R_to_quat(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2][1] - R[1][2]) / S
        qy = (R[0][2] - R[2][0]) / S
        qz = (R[1][0] - R[0][1]) / S
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        S = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        qw = (R[2][1] - R[1][2]) / S
        qx = 0.25 * S
        qy = (R[0][1] + R[1][0]) / S
        qz = (R[0][2] + R[2][0]) / S
    elif R[1][1] > R[2][2]:
        S = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        qw = (R[0][2] - R[2][0]) / S
        qx = (R[0][1] + R[1][0]) / S
        qy = 0.25 * S
        qz = (R[1][2] + R[2][1]) / S
    else:
        S = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
        qw = (R[1][0] - R[0][1]) / S
        qx = (R[0][2] + R[2][0]) / S
        qy = (R[1][2] + R[2][1]) / S
        qz = 0.25 * S
    return qx, qy, qz, qw


# ---- synthetic trace builder ----------------------------------------------
def build(headings, mode, tilt_deg, tilt_axis=(1.0, 0.4, 0.0),
          hz=240.0, still_s=4.0, move_s=2.0, noise=0.0):
    """Emit rows in the shape analyse() expects, with clear moves between
    stationary placements so the segmenter has something to cut on."""
    import random
    rng = random.Random(7)
    rows = []
    t = 0.0
    dt = 1.0 / hz
    D = axis_angle(tilt_axis, tilt_deg)

    for idx, psi in enumerate(headings):
        if mode == "A":
            R = matmul(Rz(psi), transpose(D))
        elif mode == "B":
            R = matmul(D, Rz(psi))
        elif mode == "clean":
            R = Rz(psi)
        else:
            raise ValueError(mode)
        qx, qy, qz, qw = R_to_quat(R)
        # stationary placement
        for _ in range(int(still_s * hz)):
            rows.append((t,
                         0.5 + idx * 0.30 + rng.gauss(0, noise),
                         -0.2 + idx * 0.10 + rng.gauss(0, noise),
                         0.015 + rng.gauss(0, noise),
                         qx, qy, qz, qw,
                         TOC.yaw_deg(qx, qy, qz, qw)))
            t += dt
        # transit: pick it up and carry it (breaks the still segment)
        if idx < len(headings) - 1:
            for k in range(int(move_s * hz)):
                f = k / (move_s * hz)
                rows.append((t, 0.5 + idx * 0.30 + 0.30 * f,
                             -0.2 + idx * 0.10 + 0.10 * f,
                             0.015 + 0.25 * math.sin(math.pi * f),
                             qx, qy, qz, qw,
                             TOC.yaw_deg(qx, qy, qz, qw)))
                t += dt
    return rows


def run(rows):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code, info = TOC.analyse(rows)
    return code, info, buf.getvalue()


HEADINGS = [0.0, 90.0, 200.0]

print("\n[A] template misalignment (body-fixed tilt) is named as CAUSE A")
for tilt in (3.0, 5.45, 12.0):
    code, info, _ = run(build(HEADINGS, "A", tilt))
    check(f"tilt {tilt} deg -> A_TEMPLATE", info["verdict"] == "A_TEMPLATE",
          f"(got {info['verdict']})")
    check(f"tilt {tilt} deg magnitude recovered", abs(info["tilt"] - tilt) < 0.3,
          f"(reported {info['tilt']:.2f})")

print("\n[B] non-level Vicon world (world-fixed tilt) is named as CAUSE B")
for tilt in (3.0, 5.45, 12.0):
    code, info, _ = run(build(HEADINGS, "B", tilt))
    check(f"tilt {tilt} deg -> B_WORLD", info["verdict"] == "B_WORLD",
          f"(got {info['verdict']})")
    check(f"tilt {tilt} deg magnitude recovered", abs(info["tilt"] - tilt) < 0.3,
          f"(reported {info['tilt']:.2f})")

print("\n[C] a healthy setup is not blamed for anything")
code, info, _ = run(build(HEADINGS, "clean", 0.0))
check("clean -> NO_TILT", info["verdict"] == "NO_TILT", f"(got {info['verdict']})")

print("\n[D] the two cases are genuinely distinguished, not coin-flipped")
_, ia, _ = run(build(HEADINGS, "A", 5.45))
_, ib, _ = run(build(HEADINGS, "B", 5.45))
check("A: u steady, w swings", ia["u_spread"] < 0.5 and ia["w_spread"] > 3.0,
      f"(u {ia['u_spread']:.2f}, w {ia['w_spread']:.2f})")
check("B: w steady, u swings", ib["w_spread"] < 0.5 and ib["u_spread"] > 3.0,
      f"(u {ib['u_spread']:.2f}, w {ib['w_spread']:.2f})")

print("\n[E] refuses to rule when the drone was barely rotated")
code, info, _ = run(build([0.0, 10.0, 20.0], "A", 5.45))
check("small rotation -> refuses", info["verdict"] == "TOO_LITTLE_ROTATION",
      f"(got {info['verdict']})")
check("refusal is a nonzero exit", code == 1)

print("\n[F] refuses to rule on a single placement")
code, info, _ = run(build([0.0], "A", 5.45))
check("one placement -> refuses", info["verdict"] == "TOO_FEW_PLACEMENTS",
      f"(got {info['verdict']})")

print("\n[G] segmenter finds exactly the placements that were injected")
for n in (2, 3, 5):
    hs = [i * 72.0 for i in range(n)]
    _, info, _ = run(build(hs, "A", 5.45))
    check(f"{n} placements found", len(info["placements"]) == n,
          f"(found {len(info['placements'])})")

print("\n[H] survives realistic measurement noise")
for noise in (0.0002, 0.0008):
    _, info, _ = run(build(HEADINGS, "A", 5.45, noise=noise))
    check(f"noise {noise*1000:.1f} mm -> still A", info["verdict"] == "A_TEMPLATE",
          f"(got {info['verdict']})")
    _, info, _ = run(build(HEADINGS, "B", 5.45, noise=noise))
    check(f"noise {noise*1000:.1f} mm -> still B", info["verdict"] == "B_WORLD",
          f"(got {info['verdict']})")

print("\n[I] tilt axis direction does not change the verdict")
for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.6, -0.8, 0.0), (0.3, 0.5, 0.1)):
    _, info, _ = run(build(HEADINGS, "A", 5.45, tilt_axis=axis))
    check(f"axis {axis} -> A", info["verdict"] == "A_TEMPLATE",
          f"(got {info['verdict']})")

print("\n[J] recovered vertical for case B points where it was injected")
tilt = 5.45
_, info, _ = run(build(HEADINGS, "B", tilt, tilt_axis=(1.0, 0.0, 0.0)))
D = axis_angle((1.0, 0.0, 0.0), tilt)
expect = (D[0][2], D[1][2], D[2][2])
got = info["w_mean"]
err = TOC.ang_between(expect, got)
check("w_mean matches injected world tilt", err < 0.3, f"(err {err:.3f} deg)")

print("\n[K] O(n) segmenter matches the obvious O(n^2) reference exactly")


def segment_reference(rows):
    """Deliberately naive: rescans the whole window per sample. Correct but
    quadratic. The shipped segmenter must agree with it sample for sample."""
    segs, cur = [], []

    def flush():
        if cur and cur[-1][0] - cur[0][0] >= TOC.MIN_STILL_S:
            segs.append(list(cur))

    for r in rows:
        if not cur:
            cur.append(r)
            continue
        xs = [c[1] for c in cur] + [r[1]]
        ys = [c[2] for c in cur] + [r[2]]
        zs = [c[3] for c in cur] + [r[3]]
        yw = [c[8] for c in cur] + [r[8]]
        pos_ok = max(max(xs)-min(xs), max(ys)-min(ys),
                     max(zs)-min(zs)) < TOC.STILL_POS_SPAN
        ref = yw[0]
        yaw_ok = max(abs((v - ref + 180.0) % 360.0 - 180.0)
                     for v in yw) < TOC.STILL_YAW_SPAN
        if pos_ok and yaw_ok:
            cur.append(r)
        else:
            flush()
            cur = [r]
    flush()
    return segs


for mode, hs, noise in (("A", [0.0, 90.0, 200.0], 0.0),
                        ("B", [0.0, 120.0, 240.0], 0.0),
                        ("A", [0.0, 60.0, 130.0, 250.0], 0.0008),
                        ("clean", [0.0, 90.0], 0.0005)):
    tr = build(hs, mode, 5.45, noise=noise)
    fast = TOC.segment(tr)
    slow = segment_reference(tr)
    same = (len(fast) == len(slow)
            and all(len(a) == len(b) and a[0][0] == b[0][0] and a[-1][0] == b[-1][0]
                    for a, b in zip(fast, slow)))
    check(f"{mode} {len(hs)} headings noise {noise}: identical segmentation", same,
          f"({len(fast)} vs {len(slow)} segments)")

print("\n[L] a full-length real capture analyses promptly (regression)")
# 100 s at 240 Hz with long still periods -- the shape that hung the quadratic
# version. Must finish in seconds, not minutes.
big = build([0.0, 90.0, 180.0, 270.0], "A", 5.45,
            still_s=18.0, move_s=2.0, noise=0.0005)
t0 = time.perf_counter()
_, info, _ = run(big)
dt = time.perf_counter() - t0
check(f"{len(big)} samples segmented+analysed", dt < 5.0, f"({dt:.2f} s)")
check("long capture still reaches the right verdict",
      info["verdict"] == "A_TEMPLATE", f"(got {info['verdict']})")
check("long capture finds all 4 placements", len(info["placements"]) == 4,
      f"(found {len(info['placements'])})")

def concat(*traces):
    """Glue traces end to end on a continuous clock."""
    out, t = [], 0.0
    for tr in traces:
        base = tr[0][0]
        for r in tr:
            out.append((t + (r[0] - base),) + tuple(r[1:]))
        t = out[-1][0] + 0.02
    return out


print("\n[M] repeats at one heading are merged, not counted as new headings")
# Heading 0 visited twice (as happened on the real run), plus one real second
# heading. Should collapse to 2 distinct headings, not 3.
dup = concat(build([0.0], "A", 5.45, still_s=8.0),
             build([90.0], "A", 5.45, still_s=8.0),
             build([2.0], "A", 5.45, still_s=4.0))
_, info, _ = run(dup)
check("3 placements detected", len(info["placements"]) == 3,
      f"(got {len(info['placements'])})")
check("collapsed to 2 distinct headings", len(info["selected"]) == 2,
      f"(got {len(info['selected'])})")
check("longest sample kept for the repeated heading",
      abs(info["selected"][0]["dur"] - 8.0) < 0.2
      or abs(info["selected"][1]["dur"] - 8.0) < 0.2)
check("still reaches the right verdict", info["verdict"] == "A_TEMPLATE",
      f"(got {info['verdict']})")

print("\n[N] a drone that rocks between headings is caught, not ruled on")
# Same cause, but the drone's own attitude changes between headings -- the
# exact contamination that made the first real run inconclusive.
rock = concat(build([0.0], "A", 5.45, still_s=8.0),
              build([90.0], "A", 3.20, still_s=8.0),
              build([200.0], "A", 4.60, still_s=8.0))
_, info, _ = run(rock)
check("tilt inconsistency measured", info["tilt_range"] > 1.0,
      f"(range {info['tilt_range']:.2f} deg)")
# With exactly 3 points the circle fit has zero spare degrees of freedom, so
# rocking is mathematically indistinguishable from a genuine combined defect.
# The tool must SAY that rather than quietly reporting a perfect fit.
check("3 points -> fit declared not falsifiable", info["falsifiable"] is False,
      f"(dof {info['dof']})")
check("residual is zero by construction, not agreement", info["residual"] < 1e-6,
      f"(residual {info['residual']:.2e})")

# A tilt that varies SMOOTHLY with heading is not evidence of rocking -- that
# is precisely the combined-defect signature, and the fit should absorb it.
smooth = concat(build([0.0], "A", 5.45, still_s=8.0),
                build([90.0], "A", 3.20, still_s=8.0),
                build([200.0], "A", 4.60, still_s=8.0),
                build([290.0], "A", 7.10, still_s=8.0))
_, info, _ = run(smooth)
check("smooth heading-dependent tilt is absorbed, not flagged",
      info["shaky"] is False, f"(residual {info['residual']:.2f} deg)")

# Irreducible contradiction: the SAME heading reported twice with different
# tilts. No fixed template+world defect can produce that, so it must be caught.
rock4 = concat(build([0.0], "A", 5.45, still_s=8.0),
               build([90.0], "A", 3.20, still_s=8.0),
               build([200.0], "A", 4.60, still_s=8.0),
               build([0.0], "A", 8.00, still_s=8.0))
_, info, _ = run(rock4)
check("4 points -> fit is falsifiable", info["falsifiable"] is True,
      f"(dof {info['dof']})")
check("4 points -> rocking now flagged", info["shaky"] is True,
      f"(residual {info['residual']:.2f} deg)")
check("4 points -> verdict marked unreliable",
      info["verdict"].startswith("UNRELIABLE_"), f"(got {info['verdict']})")

# ...and the same geometry with a steady attitude must still rule cleanly,
# so the gate is not simply refusing everything.
steady = concat(build([0.0], "A", 5.45, still_s=8.0),
                build([90.0], "A", 5.45, still_s=8.0),
                build([200.0], "A", 5.45, still_s=8.0))
_, info, _ = run(steady)
check("steady attitude is not flagged", info["shaky"] is False,
      f"(range {info['tilt_range']:.2f} deg)")
check("steady attitude rules cleanly", info["verdict"] == "A_TEMPLATE",
      f"(got {info['verdict']})")

print("\n[O] the real capture from 2026-08-12 is correctly refused")
# Placements as actually reported: two at ~the same heading with wildly
# different tilt. Whatever else happens, it must not emit a bare verdict.
real = concat(build([-3.17], "A", 5.12, still_s=14.1),
              build([-88.0], "A", 4.06, still_s=22.1),
              build([-0.61], "A", 0.72, still_s=3.2))
_, info, _ = run(real)
check("real capture collapses to 2 headings", len(info["selected"]) == 2,
      f"(got {len(info['selected'])})")
check("real capture is not ruled on",
      info["verdict"].startswith("UNRELIABLE_") or info["shaky"] is True,
      f"(got {info['verdict']})")

def build_both(headings, a_deg, b_deg, a_axis=(1.0, 0.4, 0.0),
               b_axis=(0.7, -0.3, 0.0), hz=240.0, still_s=6.0, move_s=2.0):
    """Both defects at once: R_wt = D . Rz(psi) . M^T.

    M is the template misalignment (a_deg), D the world tilt (b_deg). The
    reported tilt magnitude then sweeps between |a-b| and a+b as the drone
    turns -- the signature the plain spread test can only read as noise.
    """
    M = axis_angle(a_axis, a_deg)
    D = axis_angle(b_axis, b_deg)
    rows = []
    t = 0.0
    dt = 1.0 / hz
    for idx, psi in enumerate(headings):
        R = matmul(D, matmul(Rz(psi), transpose(M)))
        qx, qy, qz, qw = R_to_quat(R)
        for _ in range(int(still_s * hz)):
            rows.append((t, 0.5, -0.2, 0.015, qx, qy, qz, qw,
                         TOC.yaw_deg(qx, qy, qz, qw)))
            t += dt
        if idx < len(headings) - 1:
            for k in range(int(move_s * hz)):
                f = k / (move_s * hz)
                rows.append((t, 0.5 + 0.4 * f, -0.2, 0.015 + 0.25 * f,
                             qx, qy, qz, qw, TOC.yaw_deg(qx, qy, qz, qw)))
                t += dt
    return rows


HEAD4 = [0.0, 90.0, 180.0, 270.0]

print("\n[P] circle fit recovers a pure template misalignment")
for a in (3.0, 5.45, 12.0):
    _, info, _ = run(build(HEAD4, "A", a))
    check(f"a={a}: template recovered", abs(info["template_deg"] - a) < 0.15,
          f"(got {info['template_deg']:.2f})")
    check(f"a={a}: world reads ~0", info["world_deg"] < 0.15,
          f"(got {info['world_deg']:.2f})")
    check(f"a={a}: verdict A", info["verdict"] == "A_TEMPLATE",
          f"(got {info['verdict']})")

print("\n[Q] circle fit recovers a pure world tilt (degenerate circle)")
for b in (3.0, 5.45, 12.0):
    _, info, _ = run(build(HEAD4, "B", b))
    check(f"b={b}: world recovered", abs(info["world_deg"] - b) < 0.15,
          f"(got {info['world_deg']:.2f})")
    check(f"b={b}: template reads ~0", info["template_deg"] < 0.15,
          f"(got {info['template_deg']:.2f})")
    check(f"b={b}: verdict B", info["verdict"] == "B_WORLD",
          f"(got {info['verdict']})")

print("\n[R] both defects at once are separated, not confused")
for a, b in ((3.17, 0.79), (5.0, 3.0), (2.0, 6.0), (4.0, 4.0)):
    _, info, _ = run(build_both(HEAD4, a, b))
    check(f"a={a} b={b}: template recovered",
          abs(info["template_deg"] - a) < 0.2, f"(got {info['template_deg']:.2f})")
    check(f"a={a} b={b}: world recovered",
          abs(info["world_deg"] - b) < 0.2, f"(got {info['world_deg']:.2f})")
    check(f"a={a} b={b}: residual small", info["residual"] < 0.2,
          f"(got {info['residual']:.3f})")

print("\n[S] the dominant defect is the one named")
_, info, _ = run(build_both(HEAD4, 5.0, 0.5))
check("template-dominant -> A", info["verdict"] == "A_TEMPLATE",
      f"(got {info['verdict']})")
_, info, _ = run(build_both(HEAD4, 0.5, 5.0))
check("world-dominant -> B", info["verdict"] == "B_WORLD",
      f"(got {info['verdict']})")
_, info, _ = run(build_both(HEAD4, 4.0, 4.0))
check("comparable -> BOTH", info["verdict"] == "BOTH", f"(got {info['verdict']})")
_, info, _ = run(build_both(HEAD4, 0.4, 0.4))
check("both negligible -> NO_TILT", info["verdict"] == "NO_TILT",
      f"(got {info['verdict']})")

print("\n[T] heading-dependent tilt is EXPLAINED, not flagged as bad data")
# This is the case that made the real 2026-08-12 run inconclusive: a genuine
# combined defect makes the tilt magnitude swing, which the old tilt-range
# gate could only treat as contamination.
_, info, _ = run(build_both(HEAD4, 3.17, 0.79))
check("tilt magnitude does swing across headings", info["tilt_range"] > 1.0,
      f"(range {info['tilt_range']:.2f} deg)")
check("but the fit residual is tiny", info["residual"] < 0.2,
      f"(residual {info['residual']:.3f} deg)")
check("so it is NOT flagged shaky", info["shaky"] is False)
check("and it still rules", info["verdict"] == "A_TEMPLATE",
      f"(got {info['verdict']})")

print("\n[U] genuinely inconsistent data is still refused")
# Physical attitude actually changing between headings: scatter that no fixed
# template+world combination can produce.
noisy = concat(build([0.0], "A", 5.45, still_s=6.0),
               build([90.0], "A", 1.50, still_s=6.0),
               build([180.0], "A", 8.00, still_s=6.0),
               build([270.0], "A", 3.00, still_s=6.0))
_, info, _ = run(noisy)
check("residual exceeds the limit", info["residual"] > TOC.TILT_CONSISTENCY,
      f"(residual {info['residual']:.2f} deg)")
check("flagged shaky", info["shaky"] is True)
check("verdict marked unreliable", info["verdict"].startswith("UNRELIABLE_"),
      f"(got {info['verdict']})")

print("\n[V] the real 2026-08-12 capture, replayed end to end")
import os
CAP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "fixtures", "tilt_capture_1786563877.csv")
if os.path.exists(CAP):
    import csv as _csv
    rr = []
    with open(CAP) as f:
        rd = _csv.reader(f)
        next(rd)
        for row in rd:
            rr.append(tuple(float(v) for v in row))
    _, info, _ = run(rr)
    check("names CAUSE A", info["verdict"] in ("A_TEMPLATE", "UNRELIABLE_A_TEMPLATE"),
          f"(got {info['verdict']})")
    check("template misalignment ~3.2 deg",
          2.7 < info["template_deg"] < 3.7, f"(got {info['template_deg']:.2f})")
    check("world tilt is the smaller term",
          info["world_deg"] < info["template_deg"] / 2.0,
          f"(world {info['world_deg']:.2f} vs template {info['template_deg']:.2f})")
else:
    print(f"  SKIP  capture not present at {CAP}")

print("\n" + "=" * 60)
print(f"{PASS} passed, {FAIL} failed")
print("ALL CHECKS PASSED" if FAIL == 0 else "*** FAILURES ***")
sys.exit(1 if FAIL else 0)
