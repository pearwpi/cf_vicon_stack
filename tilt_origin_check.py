#!/usr/bin/env python3
"""Separate a misaligned rigid-body TEMPLATE from a non-level Vicon WORLD.

Same symptom, different fixes:
  A) TEMPLATE MISALIGNED -- axes baked in from a pose that was not level. The
     tilt is fixed in the DRONE's frame and rotates with it. FIX: delete and
     re-create the rigid body level. Implies a constant YAW offset too.
  B) WORLD NOT LEVEL -- the calibration L-frame sat on a non-level surface, so
     Vicon +Z is off true vertical. Fixed in the ROOM, does not rotate with the
     drone. FIX: re-run ground-plane calibration. Affects EVERY object and puts
     a real horizontal component of gravity into the controller's world.

WHY THIS IS DECISIVE. With R the reported rotation (world <- template), place
the drone flat at several yaw headings and form
    u = R^T @ z_hat   (true vertical in TEMPLATE coords)
    w = R   @ z_hat   (template Z in WORLD coords)
Case A: u is CONSTANT and w traces a cone. Case B: the reverse. Exactly one
holds still, assuming only that the quaternion is a proper rotation. Fitting
the small circle w(psi) = D . Rz(psi) . v recovers both at once: its angular
radius is the template misalignment, its axis is true vertical.

CAVEAT: a circle on a sphere has 3 free parameters, so 3 points fit it exactly
and the residual is zero regardless of the data. NEED_PLACEMENTS is 4 so every
run is falsifiable by construction.

    python3 tilt_origin_check.py [seconds]
Place the drone FLAT in at least 4 clearly different headings, ~4 s still each;
pick it up and move it between placements. Still periods are found automatically.
No radio, no motors.
"""
import math
import os
import sys
import time

from cf_core import yaw_deg_of as yaw_deg  # one definition, shared

TOPIC = os.environ.get("VICON_TOPIC", "/vicon/crazyflie1/crazyflie1")
STILL_POS_SPAN = 0.006     # m   -- max position span within a still segment
STILL_YAW_SPAN = 1.5       # deg -- max yaw span within a still segment
MIN_STILL_S = 3.0          # s   -- shortest accepted still segment
TILT_CONSISTENCY = 1.0     # deg -- max unexplained scatter (circle-fit residual)
SIGNIF_DEG = 1.5           # deg -- below this a defect is not worth acting on
MIN_YAW_SEP = 45.0         # deg -- placements must differ by at least this
NEED_PLACEMENTS = 4        # stop recording once this many headings are captured.
                           # Four, not three: a circle on a sphere has 3 free
                           # parameters, so 3 points fit it exactly and the
                           # residual is zero whatever the data. The 4th point
                           # is the first one that can disagree.


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-9:
        return None
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return (
        (1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)),
        (  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)),
        (  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)),
    )


def col_z(R):
    """R @ [0,0,1] -- template Z axis in world coords."""
    return (R[0][2], R[1][2], R[2][2])


def row_z(R):
    """R^T @ [0,0,1] -- world vertical in template coords."""
    return (R[2][0], R[2][1], R[2][2])


def mean_unit(vs):
    n = len(vs)
    m = [sum(v[i] for v in vs)/n for i in range(3)]
    L = math.sqrt(sum(c*c for c in m))
    return tuple(c/L for c in m) if L > 1e-9 else (0.0, 0.0, 1.0)


def ang_between(a, b):
    d = max(-1.0, min(1.0, sum(x*y for x, y in zip(a, b))))
    return math.degrees(math.acos(d))


def spread(vs):
    """Max angle of any sample from the mean direction, in degrees."""
    m = mean_unit(vs)
    return max(ang_between(v, m) for v in vs), m


def circle_fit(vecs):
    """Fit a small circle on the unit sphere to the template-Z vectors.

    Returns (axis, angular_radius, residual). The radius is the TEMPLATE
    misalignment and the axis is true vertical, so the axis's angle from Vicon
    +Z is the WORLD tilt: one fit recovers both defects. 3 parameters, so 3
    points fit exactly and prove nothing -- hence NEED_PLACEMENTS = 4.
    """
    import numpy as np

    V = np.array(vecs, dtype=float)
    m = V.mean(axis=0)
    norm = float(np.linalg.norm(m))
    if norm < 1e-9:
        return (0.0, 0.0, 1.0), 0.0, 0.0, 180.0
    mn = m / norm

    # Degenerate case: a pure world tilt puts every point on top of every
    # other, so the plane through them is undefined. Radius is zero and the
    # common direction IS the axis.
    if max(ang_between(tuple(v), tuple(mn)) for v in V) < 0.2:
        return tuple(mn), 0.0, ang_between(tuple(mn), (0.0, 0.0, 1.0)), 0.0

    S = (V - m).T @ (V - m)
    _, evec = np.linalg.eigh(S)
    n = evec[:, 0]                      # smallest eigenvalue -> plane normal
    if n[2] < 0:
        n = -n
    c = max(-1.0, min(1.0, float(n @ m)))
    a = math.degrees(math.acos(c))
    b = ang_between(tuple(n), (0.0, 0.0, 1.0))
    resid = max(abs(math.degrees(math.acos(
        max(-1.0, min(1.0, float(n @ v))))) - a) for v in V)
    return tuple(n), a, b, resid


def distinct_headings(placements):
    """Collapse placements that sit at effectively the same heading.

    Keeps the longest sample from each heading cluster. Repeats at one heading
    add no information -- both candidate causes are heading-invariant in tilt
    magnitude -- but they do perturb the spread statistics.
    """
    clusters = []
    for p in sorted(placements, key=lambda q: -q["dur"]):
        for c in clusters:
            if abs((p["yaw"] - c[0]["yaw"] + 180.0) % 360.0 - 180.0) < MIN_YAW_SEP:
                c.append(p)
                break
        else:
            clusters.append([p])
    return [c[0] for c in clusters]


def segment(rows):
    """Split into still segments. rows = (t, x, y, z, qx,qy,qz,qw, yaw)."""
    segs, cur = [], []

    def flush():
        if not cur:
            return
        if cur[-1][0] - cur[0][0] >= MIN_STILL_S:
            segs.append(list(cur))

    # Running extrema, not a rescan of the window per sample. A segment only
    # ever grows or resets, so incremental min/max is exactly equivalent to
    # recomputing over the whole window -- and it is O(n) instead of O(n^2).
    # The quadratic version worked fine on 4 s synthetic segments and then
    # took minutes on a real 100 s capture.
    lo = [0.0, 0.0, 0.0]
    hi = [0.0, 0.0, 0.0]
    yaw_ref = 0.0
    yaw_dev = 0.0

    def start(r):
        nonlocal lo, hi, yaw_ref, yaw_dev
        lo = [r[1], r[2], r[3]]
        hi = [r[1], r[2], r[3]]
        yaw_ref = r[8]
        yaw_dev = 0.0
        return [r]

    for r in rows:
        if not cur:
            cur = start(r)
            continue
        nlo = [min(lo[i], r[i + 1]) for i in range(3)]
        nhi = [max(hi[i], r[i + 1]) for i in range(3)]
        ndev = max(yaw_dev, abs((r[8] - yaw_ref + 180.0) % 360.0 - 180.0))
        pos_ok = max(nhi[i] - nlo[i] for i in range(3)) < STILL_POS_SPAN
        if pos_ok and ndev < STILL_YAW_SPAN:
            cur.append(r)
            lo, hi, yaw_dev = nlo, nhi, ndev
        else:
            flush()
            cur = start(r)
    flush()
    return segs


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

    rows = []

    class Probe(Node):
        def __init__(self):
            super().__init__("tilt_origin_check")
            self.create_subscription(PoseStamped, TOPIC, self.cb, 10)

        def cb(self, msg):
            p, o = msg.pose.position, msg.pose.orientation
            rows.append((time.time(), p.x, p.y, p.z,
                         o.x, o.y, o.z, o.w,
                         yaw_deg(o.x, o.y, o.z, o.w)))

    print("=" * 72)
    print("TILT ORIGIN CHECK -- passive Vicon, no radio, no motors")
    print("=" * 72)
    print(f"Recording up to {dur:.0f} s from {TOPIC}")
    print()
    print("  Leave the drone FLAT ON THE FLOOR, on ONE spot. Spin it in place")
    print("  about the vertical axis -- like turning a compass needle. Do not")
    print("  lift or tip it.")
    print()
    print(f"  {NEED_PLACEMENTS} headings, roughly 90 deg apart (N/E/S/W). Hold each")
    print(f"  still, hands OFF, for ~{MIN_STILL_S + 2:.0f} s. Then turn to the next.")
    print()
    print("  The live line below tells you what the tool currently sees, and")
    print("  it stops on its own as soon as it has enough. START TURNING NOW.")
    print()

    rclpy.init()
    node = Probe()
    t0 = time.time()
    next_status = 0.0
    done = False

    while time.time() - t0 < dur and not done:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.time() - t0
        if now < next_status or len(rows) < 20:
            continue
        next_status = now + 0.5

        # segment() is O(n) now, so re-running it every half second on the
        # whole capture is cheap -- and it guarantees the live readout is the
        # same computation as the final verdict, not a lookalike.
        snapshot = list(rows)
        segs = segment(snapshot)
        # Count DISTINCT headings, not raw segments: setting the drone back
        # down at a heading it already visited adds nothing, and stopping on
        # such a repeat is how the first real run ended up inconclusive.
        heads = [{"yaw": mean_ang([r[8] for r in s]),
                  "dur": s[-1][0] - s[0][0]} for s in segs]
        uniq = distinct_headings(heads) if heads else []
        ys = [h["yaw"] for h in uniq]
        sep = max((abs((a - b + 180.0) % 360.0 - 180.0)
                   for a in ys for b in ys), default=0.0)

        cur_still = 0.0
        if segs and snapshot[-1][0] - segs[-1][-1][0] < 0.15:
            cur_still = segs[-1][-1][0] - segs[-1][0][0]
        state = f"STILL {cur_still:4.1f}s" if cur_still > 0 else "MOVING    "
        print(f"  [{now:5.1f}s] yaw {snapshot[-1][8]:+7.1f}  {state}  "
              f"captured {len(uniq)}/{NEED_PLACEMENTS} distinct headings  "
              f"spread {sep:5.1f} deg", flush=True)

        if len(uniq) >= NEED_PLACEMENTS and sep >= MIN_YAW_SEP:
            print(f"\n  Enough data -- {len(uniq)} distinct headings, "
                  f"{sep:.0f} deg apart. Stopping early.")
            done = True

    node.destroy_node()
    rclpy.shutdown()

    # Always keep the raw capture. Re-analysis must never cost another run of
    # the operator's time.
    out = f"tilt_capture_{int(t0)}.csv"
    try:
        with open(out, "w") as f:
            f.write("t,x,y,z,qx,qy,qz,qw,yaw\n")
            for r in rows:
                f.write(",".join(repr(v) for v in r) + "\n")
        print(f"\n  raw capture saved to {out} ({len(rows)} samples)")
    except OSError as exc:
        print(f"\n  WARNING: could not save raw capture ({exc})")

    if len(rows) < 100:
        print(f"\nFAIL: only {len(rows)} samples. Is the rigid body tracked?")
        return 1

    return analyse(rows)[0]


def analyse(rows):
    """Segment, reduce to placements, and rule on the cause.

    Split out from main() so the verdict logic can be exercised against
    synthetic rigid bodies with a known injected defect -- an unvalidated
    diagnostic is worse than none, because it points confidently at the
    wrong subsystem. See test_tilt_origin_check.py.

    Returns (exit_code, info).
    """
    segs = segment(rows)
    print(f"\nCollected {len(rows)} samples; found {len(segs)} still placement(s).")

    placements = []
    for s in segs:
        us, ws, yaws = [], [], []
        for r in s:
            R = quat_to_R(r[4], r[5], r[6], r[7])
            if R is None:
                continue
            us.append(row_z(R))
            ws.append(col_z(R))
            yaws.append(r[8])
        if not us:
            continue
        placements.append({
            "dur": s[-1][0] - s[0][0],
            "yaw": mean_ang(yaws),
            "u": mean_unit(us),
            "w": mean_unit(ws),
            "tilt": ang_between(mean_unit(ws), (0.0, 0.0, 1.0)),
        })

    print()
    print("--- PLACEMENTS ---")
    for i, p in enumerate(placements):
        print(f"  {i+1}: {p['dur']:4.1f}s  yaw {p['yaw']:+8.2f} deg   "
              f"reported tilt from Vicon +Z {p['tilt']:5.2f} deg")

    if len(placements) < 2:
        print("\nNot enough still placements. Re-run and set the drone down,")
        print("hands off, in several different headings.")
        return 1, {"verdict": "TOO_FEW_PLACEMENTS", "placements": placements}

    # Two placements at the SAME heading carry no discriminating information,
    # but they do drag the spread statistics around. Merge them and keep the
    # longest sample of each distinct heading.
    sel = distinct_headings(placements)
    if len(sel) < len(placements):
        print(f"\n  merged {len(placements)} placements into {len(sel)} distinct "
              f"heading(s) (same-heading repeats keep only the longest)")

    yaws = [p["yaw"] for p in sel]
    sep = max(abs((a - b + 180.0) % 360.0 - 180.0) for a in yaws for b in yaws)
    print(f"\n  distinct headings: {len(sel)}   max separation: {sep:.1f} deg")
    if len(sel) < 2 or sep < MIN_YAW_SEP:
        print(f"  TOO SMALL -- need at least two headings {MIN_YAW_SEP:.0f} deg")
        print("  apart to tell the two causes apart. Re-run, rotating further.")
        return 1, {"verdict": "TOO_LITTLE_ROTATION", "placements": placements,
                   "selected": sel, "sep": sep}

    # Data-quality gate. Both candidate causes are heading-invariant in tilt
    # MAGNITUDE (Rz preserves the angle with z_hat), so if the reported tilt
    # changes as the drone is turned, the drone's own attitude changed --
    # it rocked on its feet, the surface is not flat, or it was still being
    # handled. That violates the assumption the whole test rests on, and it
    # corrupts u and w in different amounts, which is exactly how you get a
    # confident-looking but meaningless verdict.
    tilts = [p["tilt"] for p in sel]
    tilt_range = max(tilts) - min(tilts)

    u_spread, u_mean = spread([p["u"] for p in sel])
    w_spread, w_mean = spread([p["w"] for p in sel])
    tilt = sum(tilts) / len(tilts)

    # With three or more headings the two defects can be separated outright
    # rather than played off against each other.
    # Separating the causes needs 3+ DISTINCT headings, but the fit itself is
    # better served by EVERY placement: repeats at one heading are still points
    # on the same circle, and they are what give the residual something to
    # measure. A circle on a sphere has 3 free parameters (axis 2 + radius 1),
    # so with only 3 points the fit is exactly determined -- residual is
    # identically zero and the model cannot be contradicted by the data. That
    # is not agreement, it is an absence of evidence, and it must be said out
    # loud rather than reported as a clean fit.
    axis = None
    a_template = b_world = None
    resid = None
    dof = 0
    if len(sel) >= 3:
        fit_pts = [p["w"] for p in placements]
        axis, a_template, b_world, resid = circle_fit(fit_pts)
        dof = len(fit_pts) - 3

    print()
    print("--- DATA QUALITY ---")
    print(f"  reported tilt per heading : "
          f"{', '.join(f'{t:.2f}' for t in tilts)} deg")
    print(f"  spread across headings    : {tilt_range:.2f} deg")
    if resid is not None:
        # A heading-dependent tilt magnitude is NOT automatically bad data: it
        # is exactly what a combined defect produces, with the magnitude
        # sweeping between |a-b| and a+b. Judge the fit residual instead --
        # that is what cannot be explained by any combination of the two.
        shaky = dof > 0 and resid > TILT_CONSISTENCY
        lo, hi = abs(a_template - b_world), a_template + b_world
        print(f"  a combined defect predicts a range of {lo:.2f}-{hi:.2f} deg")
        print(f"  circle-fit residual       : {resid:.2f} deg over "
              f"{len(placements)} placements ({dof} spare degrees of freedom)")
        if dof <= 0:
            print("  NOT FALSIFIABLE -- 3 points define exactly one circle, so")
            print("  the residual is zero by construction, not by agreement.")
            print("  The numbers below are the best fit, but nothing in this")
            print("  run could have contradicted them. Add a 4th heading, or")
            print("  a repeat of one you already used, to make it testable.")
        elif shaky:
            print("  *** THE DRONE'S OWN ATTITUDE CHANGED BETWEEN HEADINGS ***")
            print("  The scatter is more than any template+world combination")
            print("  explains. It rocked, the surface is not flat, or it was")
            print("  still being handled. The verdict below is NOT trustworthy.")
        else:
            print("  OK -- the readings are consistent with a fixed defect.")
    else:
        shaky = tilt_range > TILT_CONSISTENCY
        print(f"  (only {len(sel)} headings -- cannot separate the two causes;")
        print("   a third heading would decompose them outright)")
        if shaky:
            print("  *** THE DRONE'S OWN ATTITUDE CHANGED BETWEEN HEADINGS ***")
            print("  The verdict below is NOT trustworthy.")

    print()
    print("--- DISCRIMINATOR ---")
    print(f"  mean reported tilt magnitude          : {tilt:5.2f} deg")
    print(f"  u = vertical in TEMPLATE coords, spread: {u_spread:5.2f} deg"
          f"   (small => body-fixed  => template misaligned)")
    print(f"  w = template Z in WORLD coords, spread : {w_spread:5.2f} deg"
          f"   (small => world-fixed => Vicon not level)")
    if a_template is not None:
        print()
        print("--- DECOMPOSITION (circle fit, 3+ headings) ---")
        print(f"  TEMPLATE misalignment (circle radius) : {a_template:5.2f} deg")
        print(f"  WORLD    tilt         (axis vs +Z)    : {b_world:5.2f} deg")
        print(f"  true vertical in Vicon coords         : "
              f"({axis[0]:+.4f}, {axis[1]:+.4f}, {axis[2]:+.4f})")

    print()
    print("=" * 72)
    print("VERDICT")
    print("=" * 72)
    if a_template is not None:
        a, b = a_template, b_world
        if max(a, b) < SIGNIF_DEG:
            verdict = "NO_TILT"
            print(f"  Both defects are under {SIGNIF_DEG:.1f} deg "
                  f"(template {a:.2f}, world {b:.2f}).")
            print("  Nothing here worth acting on.")
        elif a >= SIGNIF_DEG and b >= SIGNIF_DEG:
            verdict = "BOTH"
            print("  *** BOTH CAUSES ARE PRESENT ***")
            print(f"  template misalignment {a:.2f} deg, world tilt {b:.2f} deg.")
            print("  Fix the ground-plane calibration FIRST -- it moves every")
            print("  object in the lab, so re-creating rigid bodies before it")
            print("  just bakes the world error into each new template.")
        elif a > b:
            verdict = "A_TEMPLATE"
            print("  *** CAUSE A: RIGID-BODY TEMPLATE IS MISALIGNED ***")
            print(f"  Template misalignment {a:.2f} deg vs world tilt {b:.2f} deg.")
            print(f"  The world frame is level to within {b:.2f} deg, so this is")
            print("  the object's own axes, not the room's.")
            print()
            print(f"  The object's axes are baked in {a:.1f} deg off level.")
        else:
            verdict = "B_WORLD"
            print("  *** CAUSE B: THE VICON WORLD FRAME IS NOT LEVEL ***")
            print(f"  World tilt {b:.2f} deg vs template misalignment {a:.2f} deg.")
            print(f"     true vertical in Vicon coords ~ ({axis[0]:+.4f}, "
                  f"{axis[1]:+.4f}, {axis[2]:+.4f})")
            print()
            print("  FIX: re-run the Vicon ground-plane / L-frame calibration on")
            print("       a surface that is actually level.")
            print()
            print("  This one is worse than it looks. Every rigid body in the lab")
            print("  is affected, and the controller's 'horizontal' is tilted, so")
            print(f"  gravity gets a real {math.sin(math.radians(b))*9.81:.2f} m/s^2")
            print("  component along world-horizontal that the loop must fight.")
    elif tilt < SIGNIF_DEG:
        verdict = "NO_TILT"
        print("  Reported tilt is under 1.5 deg -- nothing to explain here.")
        print("  Whatever is wrong, it is not an orientation-template tilt.")
    elif u_spread < w_spread / 2.0:
        verdict = "A_TEMPLATE"
        print("  *** CAUSE A: RIGID-BODY TEMPLATE IS MISALIGNED ***")
        print(f"  The tilt is fixed in the DRONE's frame ({u_spread:.2f} deg spread)")
        print(f"  while its world direction swings by {w_spread:.2f} deg.")
        print()
        print(f"  The object's axes are baked in {tilt:.1f} deg off level.")
    elif w_spread < u_spread / 2.0:
        verdict = "B_WORLD"
        print("  *** CAUSE B: THE VICON WORLD FRAME IS NOT LEVEL ***")
        print(f"  The tilt is fixed in the WORLD ({w_spread:.2f} deg spread) while")
        print(f"  its body direction swings by {u_spread:.2f} deg.")
        print()
        print(f"  Vicon's +Z is about {tilt:.1f} deg off true vertical:")
        print(f"     true vertical in Vicon coords ~ ({w_mean[0]:+.3f}, "
              f"{w_mean[1]:+.3f}, {w_mean[2]:+.3f})")
        print()
        print("  FIX: re-run the Vicon ground-plane / L-frame calibration on a")
        print("       surface that is actually level.")
        print()
        print("  This one is worse than it looks. Every rigid body in the lab is")
        print("  affected, and the controller's 'horizontal' is tilted, so")
        print(f"  gravity gets a real {math.sin(math.radians(tilt))*9.81:.2f} m/s^2")
        print("  component along world-horizontal that the loop must fight.")
    else:
        verdict = "INCONCLUSIVE"
        print("  INCONCLUSIVE -- neither vector is clearly the steady one")
        print(f"  (u {u_spread:.2f} deg vs w {w_spread:.2f} deg).")
        print("  Most likely BOTH are present, or the drone was not actually")
        print("  flat on the floor for every placement. Re-run on a surface you")
        print("  have checked with a spirit level, using wider heading changes.")
    if verdict in ("A_TEMPLATE", "BOTH"):
        print()
        print("  FIX: in Tracker, put the drone flat and level, then DELETE and")
        print("       RE-CREATE the rigid body. Editing markers is not enough --")
        print("       the old template survives.")
        print()
        print("  IMPORTANT: the same act that baked in this tilt also bakes in")
        print("  the object's YAW zero. Expect a constant yaw offset too, which")
        print("  is the leading explanation for the outward spiral. After")
        print("  re-creating, confirm with --frame-test.")

    if shaky and verdict != "NO_TILT":
        verdict = "UNRELIABLE_" + verdict
        print()
        print("  ^^ DISREGARD THE ABOVE. The data-quality gate failed: the")
        print("     readings scatter more than any fixed template+world defect")
        print("     explains, so the drone's own attitude moved between")
        print("     headings. Re-run on a flat rigid surface, hands off.")
    return 0, {"verdict": verdict, "tilt": tilt, "u_spread": u_spread,
               "w_spread": w_spread, "u_mean": u_mean, "w_mean": w_mean,
               "sep": sep, "placements": placements, "selected": sel,
               "tilt_range": tilt_range, "shaky": shaky,
               "template_deg": a_template, "world_deg": b_world,
               "axis": axis, "residual": resid, "dof": dof,
               "falsifiable": dof > 0}


def mean_ang(vals):
    s = sum(math.sin(math.radians(v)) for v in vals)
    c = sum(math.cos(math.radians(v)) for v in vals)
    return math.degrees(math.atan2(s, c))


if __name__ == "__main__":
    sys.exit(main())
