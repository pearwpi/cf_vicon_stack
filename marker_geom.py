#!/usr/bin/env python3
"""
marker_geom.py -- does this marker layout have a rotational ambiguity?

WHY THIS EXISTS
---------------
A 180 deg yaw flip family means the marker constellation maps onto itself under
a half turn: the solver has two equally good answers and picks arbitrarily.
That is a property of WHERE THE MARKERS ARE, not of vibration, not of the
controller, not of the flight.  So it can be checked on a laptop, before any
battery is spent and without touching the aircraft.

Feed it the marker positions of the rigid body and it enumerates every
near-symmetry of the constellation and reports the margin -- how much worse the
best WRONG answer is than the right one.  Small margin = ambiguous.

CALIBRATED AGAINST ONE MEASURED FAILURE
---------------------------------------
On 2026-09-08 the lab's `crazyflie2` object was recorded flipping in flight,
with ALL SEVEN markers visible and no data gap: single-frame attitude jumps of
86-95 deg and 158-160 deg, out and back within one 240 Hz frame.

This tool scores that same layout at 25.2 mm and, under its original 8 mm /
15 mm thresholds, called it PASS.  It also predicted both families correctly --
167 deg about a near-horizontal axis (the roll inversions) and 100.7 deg about
the vertical (the yaw jumps).  So the ANALYSIS was right and the THRESHOLDS
were wrong, and they are now set from that measurement: 25.2 mm demonstrably
flips, so anything at or below it must fail.

That is one labelled example, not a calibration curve.  A layout that passes
here is a layout worth building and then MEASURING with track_monitor.py; it is
not a layout proven good.

Input, best first:
    python3 marker_geom.py --vsk ~/vicon/objects/crazyflie2.vsk
    python3 marker_geom.py --csv markers.csv          # x,y,z per line
    python3 marker_geom.py --xyz "0,0,0 40,0,0 ..."   # mm, space separated

In Vicon Tracker the object file is a .vsk under the objects directory; using it
avoids transcribing coordinates by hand.  Otherwise read the marker positions
off the object's properties panel.
"""
import argparse
import itertools
import math
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

# A candidate rotation counts as a competing solution if every marker lands
# within this of another marker.  Vicon markers are matched by pairwise
# distance; the solver cannot separate two fits that agree this closely.
# Set from the 2026-09-08 measurement above: `crazyflie2` scores 25.2 mm and
# flips, so 25.2 mm is inside the failure region, not outside it.
AMBIGUOUS_MM = 30.0         # hard fail below this
MARGINAL_MM = 45.0          # warn below this
# Depth, as a fraction of the widest marker pair. A perfectly planar set maps
# onto itself under a 180 deg rotation about ANY in-plane axis through its
# centroid -- only the in-plane arrangement breaks that, and it breaks it
# weakly. Depth is what makes the flip geometrically impossible rather than
# merely numerically disfavoured. `crazyflie2` sits at 0.096 and flips.
FLAT_RATIO = 0.15           # hard fail below this
THIN_RATIO = 0.25           # warn below this
MIN_ROT_DEG = 15.0          # below this it is the identity, not a symmetry
RADIUS_TOL_MM = 12.0        # pairing tolerance when generating candidates


# --------------------------------------------------------------------- input
def from_vsk(path):
    """Vicon Tracker .vsk.  Two layouts exist and both appear in the wild:

    a) <Marker NAME=... POSITION="x y z"/>
    b) <Parameter NAME="<obj>_<marker>_x" VALUE="..."/>  -- Tracker 3.x writes
       this one, with the coordinates hoisted out into <Parameters> and the
       <Marker> elements carrying only name/radius/colour.
    """
    root = ET.parse(path).getroot()

    pts, names = [], []
    for el in root.iter():
        if not el.tag.endswith("Marker"):
            continue
        pos = el.get("POSITION") or el.get("Position") or el.get("position")
        if pos:
            v = [float(x) for x in re.split(r"[,\s]+", pos.strip()) if x]
            if len(v) >= 3:
                pts.append(v[:3])
                names.append(el.get("NAME") or el.get("Name") or "m%d" % len(pts))
    if pts:
        return np.array(pts, float), names

    axes = {}
    for el in root.iter():
        if not el.tag.endswith("Parameter"):
            continue
        nm, val = el.get("NAME") or "", el.get("VALUE")
        if val is None or len(nm) < 3 or nm[-2] != "_" or nm[-1] not in "xyz":
            continue
        axes.setdefault(nm[:-2], {})[nm[-1]] = float(val)
    for stem in sorted(axes):
        c = axes[stem]
        if set(c) == {"x", "y", "z"}:
            pts.append([c["x"], c["y"], c["z"]])
            names.append(stem.split("_")[-1] or stem)
    if not pts:
        sys.exit("no marker coordinates found in %s\n"
                 "Expected <Marker POSITION=...> or <Parameter NAME='..._x'>." % path)
    return np.array(pts, float), names


def from_text(text):
    nums = [float(x) for x in re.split(r"[,\s]+", text.strip()) if x]
    if len(nums) % 3:
        sys.exit("need a multiple of 3 numbers, got %d" % len(nums))
    p = np.array(nums, float).reshape(-1, 3)
    return p, ["m%d" % (i + 1) for i in range(len(p))]


# ---------------------------------------------------------------- geometry
def frame_from(a, b):
    """Orthonormal frame from two non-parallel vectors, or None."""
    na = np.linalg.norm(a)
    if na < 1e-6:
        return None
    u1 = a / na
    r = b - np.dot(b, u1) * u1
    nr = np.linalg.norm(r)
    if nr < 1e-3:                       # a and b parallel: frame undefined
        return None
    u2 = r / nr
    return np.column_stack([u1, u2, np.cross(u1, u2)])


def rot_angle(R):
    """Rotation angle only -- from the trace, no eigendecomposition.

    Called once per candidate correspondence, so it must stay cheap; the axis
    is worked out later for the handful of candidates that survive.
    """
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))))


def rot_angle_axis(R):
    ang = rot_angle(R)
    w, V = np.linalg.eig(R)
    axis = None
    for i in range(3):
        if abs(w[i].real - 1.0) < 1e-6 and abs(w[i].imag) < 1e-6:
            axis = V[:, i].real
            break
    if axis is None:
        axis = np.array([0.0, 0.0, 1.0])
    axis = axis / (np.linalg.norm(axis) or 1.0)
    if axis[2] < -1e-9:                 # canonical sign: prefer +Z
        axis, ang = -axis, ang
    return ang, axis


def residual(P, R):
    """Worst per-marker distance after rotating P by R and matching to P.

    Greedy nearest with exclusion.  Reports the WORST marker, not the RMS: one
    marker far from any partner is enough to break a symmetry, and averaging
    would hide exactly that.
    """
    Q = P @ R.T
    free = list(range(len(P)))
    worst = 0.0
    for q in Q:
        if not free:
            return float("inf")
        d = [np.linalg.norm(q - P[j]) for j in free]
        k = int(np.argmin(d))
        worst = max(worst, d[k])
        free.pop(k)
    return worst


def symmetries(P):
    """Enumerate candidate near-symmetries of the centred point set."""
    n = len(P)
    r = np.linalg.norm(P, axis=1)
    out = []
    for i, k in itertools.permutations(range(n), 2):
        Fs = frame_from(P[i], P[k])
        if Fs is None:
            continue
        for j, l in itertools.permutations(range(n), 2):
            if (i, k) == (j, l):
                continue
            if abs(r[i] - r[j]) > RADIUS_TOL_MM or abs(r[k] - r[l]) > RADIUS_TOL_MM:
                continue
            Ft = frame_from(P[j], P[l])
            if Ft is None:
                continue
            R = Ft @ Fs.T
            ang = rot_angle(R)
            if ang < MIN_ROT_DEG:
                continue
            out.append((residual(P, R), ang, R))
    out.sort(key=lambda t: t[0])
    out = [(res, ang, rot_angle_axis(R)[1]) for res, ang, R in out[:40]]
    keep = []                            # dedupe by angle+axis
    for res, ang, axis in out:
        if any(abs(ang - a) < 6.0 and np.dot(axis, ax) > 0.97 for _, a, ax in keep):
            continue
        keep.append((res, ang, axis))
    return keep[:8]


def axis_kind(axis):
    """What a rotation about this axis does to the airframe.

    The distinction matters for the FIX: a symmetry about the vertical axis is
    broken by moving a marker in the horizontal plane, and one about a
    horizontal axis is broken by moving a marker in Z. Advice that assumes the
    wrong one sends you to rebuild the wrong part of the aircraft.
    """
    tilt = math.degrees(math.asin(min(1.0, abs(float(axis[2])))))
    if tilt < 30.0:
        return "roll/pitch flip", "z"
    if tilt > 60.0:
        return "yaw flip", "xy"
    return "oblique flip", "z"


def planarity(P):
    """Out-of-plane depth as a fraction of the widest marker pair."""
    sv = np.linalg.svd(P, compute_uv=False)
    spread = max(np.linalg.norm(P[i] - P[j])
                 for i in range(len(P)) for j in range(i + 1, len(P)))
    return float(sv[2]), float(spread), float(sv[2] / spread) if spread else 0.0


def standoff_sweep(P, names, heights=(20.0, 30.0, 35.0)):
    """What raising each existing marker on a standoff would do.

    Raising an EXISTING marker, not adding a new one. Adding an eighth marker
    at the centroid was tried on `crazyflie2` and made it WORSE -- 25.2 mm down
    to 13.3 mm at a 30 mm height -- because a point on the axis of the
    candidate rotations is very nearly invariant under them, so it adds almost
    no discriminating information and brings new small-angle near-symmetries
    with it.
    """
    rows = []
    for i, n in enumerate(names):
        r = float(np.linalg.norm(P[i][:2]))
        out = []
        for h in heights:
            Q = P.copy()
            Q[i, 2] += h
            out.append(margin_of(Q - Q.mean(axis=0)))
        rows.append((n, r, out))
    return rows


def pair_distances(P):
    n = len(P)
    return sorted(((np.linalg.norm(P[i] - P[j]), i, j)
                   for i in range(n) for j in range(i + 1, n)))


def margin_of(P):
    sy = symmetries(P)
    return sy[0][0] if sy else float("inf")


def dropout_margin(P, names):
    """Worst margin with any ONE marker occluded.

    This is the number that matters in flight. A layout whose asymmetry rests
    on a single marker looks fine with everything visible and collapses the
    instant that marker is hidden -- which is exactly the moment the solver
    starts choosing between branches.
    """
    worst, culprit = float("inf"), None
    for i in range(len(P)):
        if len(P) - 1 < 4:
            break
        sub = np.delete(P, i, axis=0)
        sub = sub - sub.mean(axis=0)
        m = margin_of(sub)
        if m < worst:
            worst, culprit = m, names[i]
    return worst, culprit


def min_colinearity(P):
    """Smallest distance of any marker from the line through two others."""
    n = len(P)
    worst, trio = float("inf"), None
    for i, j, k in itertools.combinations(range(n), 3):
        d = P[j] - P[i]
        L = np.linalg.norm(d)
        if L < 1e-6:
            return 0.0, (i, j, k)
        h = np.linalg.norm(np.cross(P[k] - P[i], d)) / L
        if h < worst:
            worst, trio = h, (i, j, k)
    return worst, trio


# -------------------------------------------------------------------- report
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--vsk")
    g.add_argument("--csv")
    g.add_argument("--xyz")
    ap.add_argument("--metres", action="store_true", help="input is in m, not mm")
    ap.add_argument("--sweep", action="store_true",
                    help="also show what raising each existing marker on a "
                         "standoff would do, so a post can be placed where one "
                         "physically fits rather than where the maths prefers")
    a = ap.parse_args()

    if a.vsk:
        P, names = from_vsk(a.vsk)
    elif a.csv:
        P, names = from_text(open(a.csv).read())
    else:
        P, names = from_text(a.xyz)
    if a.metres:
        P = P * 1000.0
    P = P - P.mean(axis=0)

    print("\n  markers        : %d" % len(P))
    if len(P) < 4:
        sys.exit("  need at least 4 markers for a rigid body.")

    ds = pair_distances(P)
    print("  spread         : %.0f mm (closest pair %.0f mm, widest %.0f mm)"
          % (ds[-1][0], ds[0][0], ds[-1][0]))
    dup = [(d1, d2) for (d1, i1, j1), (d2, i2, j2) in zip(ds, ds[1:])
           if abs(d1 - d2) < 3.0]
    print("  duplicate gaps : %d pairs within 3 mm of another pair%s"
          % (len(dup), "  <-- solver has less to work with" if dup else ""))

    depth, spread, ratio = planarity(P)
    flat = ("   <-- FLAT PLATE, flips about any in-plane axis"
            if ratio < FLAT_RATIO else
            "   <-- thin" if ratio < THIN_RATIO else "")
    print("  out-of-plane   : %.0f mm of depth over %.0f mm = %.3f of the spread%s"
          % (depth, spread, ratio, flat))

    sy = symmetries(P)
    print("\n  competing solutions the solver could pick instead:")
    if not sy:
        print("    none found")
    for res, ang, axis in sy[:5]:
        tag = ""
        if res < AMBIGUOUS_MM:
            tag = "   <-- AMBIGUOUS"
        elif res < MARGINAL_MM:
            tag = "   <-- marginal"
        kind, _ = axis_kind(axis)
        print("    %6.1f deg about [%5.2f %5.2f %5.2f]  %-15s off by %6.1f mm%s"
              % (ang, axis[0], axis[1], axis[2], kind, res, tag))

    margin = sy[0][0] if sy else float("inf")

    col, trio = min_colinearity(P)
    if trio is not None:
        print("\n  colinearity    : %.1f mm  (%s)%s"
              % (col, ", ".join(names[i] for i in trio),
                 "   <-- three markers nearly on a line" if col < 8.0 else ""))
    drop, culprit = dropout_margin(P, names)
    if culprit is not None:
        print("  with %-9s occluded: margin falls to %.1f mm%s"
              % (culprit, drop, "   <-- SINGLE POINT OF FAILURE"
                 if drop < AMBIGUOUS_MM <= margin else ""))

    print("\n  " + "=" * 62)
    print("  MARGIN: %s mm   (need >= %.0f, comfortable >= %.0f)"
          % ("inf" if margin == float("inf") else "%.1f" % margin,
             AMBIGUOUS_MM, MARGINAL_MM))
    if drop < AMBIGUOUS_MM <= margin:
        print("  ...but only %.1f mm once %s is occluded. The asymmetry rests on"
              % (drop, culprit))
        print("  one marker, so it is gone the moment that marker is hidden.")
        print("  A layout needs TWO independent asymmetries, not one.")
        margin = drop
    if margin < AMBIGUOUS_MM or ratio < FLAT_RATIO:
        res, ang, axis = sy[0] if sy else (float("inf"), 180.0,
                                           np.array([1.0, 0.0, 0.0]))
        kind, move = axis_kind(axis)
        print("  VERDICT: FAIL -- ambiguous under a %.0f deg rotation (%s)."
              % (ang, kind))
        if ratio < FLAT_RATIO:
            print("  The set is %.3f of its spread deep. That is a flat plate, and a"
                  % ratio)
            print("  flat plate maps onto itself under a half turn about any in-plane")
            print("  axis. This flips with EVERY MARKER VISIBLE and a clean fit -- it")
            print("  is not an occlusion problem and no filtering reaches it.")
        if move == "z":
            print("\n  To break it: add DEPTH. Raise ONE EXISTING marker on a stiff")
            print("  standoff. Do NOT add an extra marker at the centre: a point on")
            print("  the axis of the competing rotations is nearly invariant under")
            print("  them, contributes almost nothing, and brings new small-angle")
            print("  near-symmetries with it. Measured on crazyflie2: adding an")
            print("  eighth centre marker at 30 mm took the margin from 25.2 mm")
            print("  DOWN to 13.3 mm. Re-run with --sweep for the per-marker table.")
        else:
            print("\n  To break it: move ONE marker in the HORIZONTAL plane -- radially")
            print("  in or out along an arm. Height does nothing against a rotation")
            print("  about the vertical axis: rotate about Z and Z is unchanged.")
    elif margin < MARGINAL_MM or ratio < THIN_RATIO:
        print("  VERDICT: MARGINAL -- it will hold when the fit is clean and slip")
        print("  when markers are partly occluded or the airframe flexes.")
    else:
        print("  VERDICT: PASS -- no competing solution within %.0f mm, and %.3f"
              % (MARGINAL_MM, ratio))
        print("  of the spread in depth. Geometry is clear.")
        print("  Two things this CANNOT see: marker FLEX -- press every marker by")
        print("  hand, any give is a defect -- and whether the layout actually")
        print("  stops flipping. Fly it and watch track_monitor.py; the thresholds")
        print("  here rest on a single measured failure, not a calibration curve.")

    if a.sweep:
        print("\n  raising ONE EXISTING marker on a standoff (margin, mm):")
        print("    %-12s %10s   %7s %7s %7s" % ("marker", "r from ctr",
                                                "+20 mm", "+30 mm", "+35 mm"))
        for n, r, vals in standoff_sweep(P, names):
            print("    %-12s %7.1f mm   %7.1f %7.1f %7.1f" % (n, r, *vals))
        print("    %-12s %7s    now %.1f mm" % ("(as built)", "", margin))
        print("\n  Prefer an option that IMPROVES MONOTONICALLY with height: a")
        print("  column that jumps around means a few millimetres of build error")
        print("  lands somewhere much worse. Prefer a small radius too -- mass at")
        print("  the rim costs inertia and sits closer to the props.")
    print()


if __name__ == "__main__":
    main()
