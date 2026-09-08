#!/usr/bin/env python3
r"""Verify a mocap rigid body on the bench. No flying, no props.

Two defects wreck position control, both invisible in Tracker's viewport, both
fixable only in Tracker and never in flight code.

1. CONSTANT YAW OFFSET. Tracker defines the object's axes from the drone's
   orientation AT THE MOMENT you click create. If the nose was not on world +X,
   every reported yaw is wrong by a fixed t, the controller corrects in a
   rotated frame (de/dt = -k*R(t)*e), and the error spirals instead of
   converging; for |t| > 90 deg it diverges every flight.
   MEASURE: slide the drone along its own nose, compare the direction it
   actually travelled against the yaw it reported. The difference IS t.
   Convention-free: assumes nothing about quaternion order or handedness.

2. ORIGIN NOT AT THE DRONE'S CENTRE. With the origin off by d in the body
   frame, p_reported = p_true + R(yaw)*d, so as the drone yaws the reported
   position sweeps a circle of radius |d| and the controller faithfully chases
   a moving target -- an unfixable-looking slow orbit.
   MEASURE: rotate in place and regress reported position against yaw:
       x = cx + dx*cos(yaw) - dy*sin(yaw)
       y = cy + dx*sin(yaw) + dy*cos(yaw)
   Linear in (cx, cy, dx, dy), so a plain least-squares solve, and it tolerates
   the human drifting the centre around while turning.

Used by: python3 crazyflie_vicon_teleop.py --frame-test
"""

from __future__ import annotations

from cf_core import wrap180

import math

YAW_OFFSET_TOLERANCE_DEG = 12.0
ORIGIN_OFFSET_TOLERANCE_M = 0.020
MIN_SLIDE_M = 0.25
MAX_ROTATION_DURING_SLIDE_DEG = 25.0
MIN_YAW_SPAN_FOR_FIT_DEG = 90.0
# The drone is not rotated between the two slides, so the yaw it REPORTS must
# agree between them. More than this and the mocap solver changed solution
# between the slides. Well under MIN_FLIP_DEG (45) so it catches a flip, well
# over the few degrees of sloppy repositioning by hand.
SLIDE_YAW_AGREEMENT_DEG = 20.0


def circular_mean_deg(yaws) -> float:
    """Mean of angles. A plain arithmetic mean is wrong across the +-180 seam."""
    if not yaws:
        return 0.0
    sx = sum(math.cos(math.radians(y)) for y in yaws)
    sy = sum(math.sin(math.radians(y)) for y in yaws)
    return math.degrees(math.atan2(sy, sx))


def circular_span_deg(yaws) -> float:
    """Total angular range covered, seam-safe, via unwrapping."""
    if len(yaws) < 2:
        return 0.0
    unwrapped = [yaws[0]]
    for y in yaws[1:]:
        unwrapped.append(unwrapped[-1] + wrap180(y - unwrapped[-1]))
    return max(unwrapped) - min(unwrapped)


def analyze_slide(start, end, yaws, expected_offset_deg=0.0):
    """
    Compare the direction the drone actually moved with the yaw it reported.

    start/end: (x, y, z). yaws: reported yaw samples during the slide.
    expected_offset_deg: 0 for a nose-forward slide, +90 for a slide to its left.

    Returns a dict; 'yaw_error_deg' is the constant offset baked into the rigid
    body -- positive means the reported yaw reads LOWER than reality by that
    amount, i.e. the object's X axis is rotated that far from the nose.
    """
    dx, dy = end[0] - start[0], end[1] - start[1]
    dist = math.hypot(dx, dy)
    out = {
        "distance_m": dist,
        "travel_heading_deg": math.degrees(math.atan2(dy, dx)),
        "mean_yaw_deg": circular_mean_deg(yaws),
        "yaw_wobble_deg": circular_span_deg(yaws),
        "dz_m": end[2] - start[2],
    }
    if dist < MIN_SLIDE_M:
        out["error"] = (f"only moved {dist*100:.0f} cm; need "
                        f"{MIN_SLIDE_M*100:.0f} cm for a reliable heading")
        return out
    if out["yaw_wobble_deg"] > MAX_ROTATION_DURING_SLIDE_DEG:
        out["error"] = (f"the drone rotated {out['yaw_wobble_deg']:.0f} deg "
                        f"during the slide -- keep it pointing the same way "
                        f"and repeat")
        return out
    out["yaw_error_deg"] = wrap180(
        out["travel_heading_deg"] - out["mean_yaw_deg"] - expected_offset_deg)
    out["ok"] = abs(out["yaw_error_deg"]) <= YAW_OFFSET_TOLERANCE_DEG
    return out


def _solve(A, b):
    """Gauss-Jordan with partial pivoting. n is 4 here, so this is plenty."""
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-12:
            return None
        M[c], M[piv] = M[piv], M[c]
        d = M[c][c]
        M[c] = [v / d for v in M[c]]
        for r in range(n):
            if r != c and M[r][c] != 0.0:
                f = M[r][c]
                M[r] = [a - f * bb for a, bb in zip(M[r], M[c])]
    return [M[i][n] for i in range(n)]


def fit_origin_offset(samples):
    """
    samples: iterable of (x, y, yaw_deg) captured while rotating in place.

    Solves for the rigid-body origin offset d in BODY coordinates:
        x = cx + dx*cos(yaw) - dy*sin(yaw)
        y = cy + dx*sin(yaw) + dy*cos(yaw)
    """
    pts = list(samples)
    if len(pts) < 20:
        return {"error": f"only {len(pts)} samples; need >= 20"}
    span = circular_span_deg([p[2] for p in pts])
    if span < MIN_YAW_SPAN_FOR_FIT_DEG:
        return {"error": f"only rotated {span:.0f} deg; need >= "
                         f"{MIN_YAW_SPAN_FOR_FIT_DEG:.0f} deg to separate the "
                         f"offset from the centre"}

    ATA = [[0.0] * 4 for _ in range(4)]
    ATb = [0.0] * 4
    for x, y, yaw in pts:
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        for row, target in (([1.0, 0.0, c, -s], x), ([0.0, 1.0, s, c], y)):
            for i in range(4):
                ATb[i] += row[i] * target
                for j in range(4):
                    ATA[i][j] += row[i] * row[j]
    p = _solve(ATA, ATb)
    if p is None:
        return {"error": "fit is singular -- rotate through a wider angle"}
    cx, cy, dx, dy = p

    resid = []
    for x, y, yaw in pts:
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        resid.append(math.hypot(x - (cx + dx * c - dy * s),
                                y - (cy + dx * s + dy * c)))
    rms = math.sqrt(sum(r * r for r in resid) / len(resid))
    return {
        "center": (cx, cy),
        "offset_body": (dx, dy),
        "offset_mag": math.hypot(dx, dy),
        "residual_rms_m": rms,
        "yaw_span_deg": span,
        "n": len(pts),
        "ok": math.hypot(dx, dy) <= ORIGIN_OFFSET_TOLERANCE_M,
    }


def report(nose, left, lift_dz, origin, flips, yaw_jitter_still):
    L = ["", "=" * 72, "RIGID BODY FRAME CHECK", "=" * 72]
    problems = []

    # -- orientation stability ------------------------------------------------
    L.append(f"  stationary yaw : {yaw_jitter_still:.1f} deg of movement while "
             f"the drone was NOT moving")
    L.append(f"  solver flips   : {flips}")
    if flips:
        problems.append((
            "PRIMARY",
            f"{flips} impossible yaw jumps -- the marker layout is still "
            f"rotationally ambiguous.",
            "Tracker still has two valid solutions. Add asymmetry: 4-5 markers "
            "at DIFFERENT HEIGHTS, irregular spacing, then DELETE and re-create "
            "the rigid body (editing markers without re-creating keeps the old "
            "template)."))
    elif yaw_jitter_still > 5.0:
        problems.append((
            "LIKELY",
            f"{yaw_jitter_still:.0f} deg of yaw movement on a stationary drone.",
            "The orientation solution is poorly conditioned -- markers are close "
            "to symmetric or too close together. Spread them out."))

    # -- the key measurement --------------------------------------------------
    for name, res, expect in (("nose-forward", nose, 0.0), ("to-its-left", left, 90.0)):
        if res is None:
            L.append(f"  {name:14s}: skipped")
            continue
        if "error" in res:
            L.append(f"  {name:14s}: UNUSABLE -- {res['error']}")
            continue
        L.append(f"  {name:14s}: moved {res['distance_m']*100:.0f} cm at "
                 f"{res['travel_heading_deg']:+.1f} deg, reported yaw "
                 f"{res['mean_yaw_deg']:+.1f} deg  =>  "
                 f"offset {res['yaw_error_deg']:+.1f} deg")

    # CROSS-BRANCH GUARD. Nothing rotates the drone between the two slides, so
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
        usable = []          # refuse to derive an offset from mixed branches
    if usable:
        errs = [r["yaw_error_deg"] for r in usable]
        mean_err = circular_mean_deg(errs)
        if abs(mean_err) > YAW_OFFSET_TOLERANCE_DEG:
            sev = "PRIMARY" if abs(mean_err) > 25 else "LIKELY"
            unstable = " This alone makes the position loop DIVERGE." \
                if abs(mean_err) > 90 else ""
            problems.append((
                sev,
                f"CONSTANT YAW OFFSET of {mean_err:+.0f} deg: the rigid body's "
                f"X axis is not the drone's nose.{unstable}",
                f"In Tracker, rotate the object's local frame by "
                f"{-mean_err:+.0f} deg about Z, or -- cleaner -- physically "
                f"align the nose with world +X, then DELETE and re-create the "
                f"rigid body. Do not correct this in flight code; the estimator "
                f"and the controller both need the true heading."))
        if len(usable) == 2 and abs(wrap180(errs[0] - errs[1])) > 25.0:
            problems.append((
                "PRIMARY",
                f"The two slides disagree by "
                f"{abs(wrap180(errs[0] - errs[1])):.0f} deg -- the axes are not "
                f"a consistent rotation of the body frame.",
                "That means a MIRRORED or swapped axis, not a rotation. Check "
                "for a left-handed frame / negated axis in the publisher, and "
                "re-create the rigid body."))

    # -- z sign ---------------------------------------------------------------
    if lift_dz is not None:
        L.append(f"  lift test      : dz = {lift_dz:+.3f} m")
        if lift_dz < 0.05:
            problems.append((
                "PRIMARY",
                f"Lifting the drone changed z by {lift_dz:+.3f} m -- Z is "
                f"inverted or not vertical.",
                "The world frame is not Z-up. Fix the Vicon world calibration; "
                "everything downstream assumes Z-up, right-handed."))

    # -- origin ---------------------------------------------------------------
    if origin is None:
        L.append("  origin test    : skipped")
    elif "error" in origin:
        L.append(f"  origin test    : UNUSABLE -- {origin['error']}")
    else:
        L.append(f"  origin test    : offset {origin['offset_mag']*100:.1f} cm "
                 f"at body ({origin['offset_body'][0]*100:+.1f}, "
                 f"{origin['offset_body'][1]*100:+.1f}) cm, "
                 f"fit residual {origin['residual_rms_m']*100:.1f} cm over "
                 f"{origin['yaw_span_deg']:.0f} deg")
        if not origin["ok"]:
            problems.append((
                "LIKELY",
                f"RIGID BODY ORIGIN is {origin['offset_mag']*100:.0f} cm off "
                f"the drone's centre of rotation.",
                "Reported position then sweeps a circle of that radius as the "
                "drone yaws, and the controller dutifully chases it -- a slow "
                "orbit you cannot tune out. Set the object's origin to the "
                "drone's centre in Tracker."))

    L += ["", "-" * 72, "VERDICT", "-" * 72]
    if not problems:
        L += ["  PASS -- the rigid body is sane. Frame geometry is not your bug.",
              "  Next: python3 crazyflie_vicon_teleop.py --diagnose 30"]
    else:
        for i, (sev, what, fix) in enumerate(problems, 1):
            L.append(f"{i}. [{sev}] {what}")
            for line in fix.splitlines():
                L.append(f"      {line}")
            L.append("")
        L.append("  Fix these in Vicon Tracker before flying again.")
        L.append("  To fly meanwhile, --pos-only ignores mocap orientation "
                 "entirely.")
    L.append("=" * 72)
    return "\n".join(L)


# ==============================================================================
# CLOSED-LOOP SIGN TEST (props OFF, motors spinning, drone in your hand)
# ==============================================================================
# Everything above measures the mocap system. This measures the WHOLE LOOP --
# Vicon -> EKF -> position controller -> velocity controller -> attitude -> motor
# mix -- and answers one question: when the drone is displaced from its
# setpoint, does it try to tilt TOWARD the setpoint or somewhere else?
#
# It is made convention-free by a single piece of setup: the drone is held with
# its NOSE ALONG WORLD +X and kept that way. Physical yaw is then 0, so the body
# frame equals the world frame and no quaternion / handedness / firmware sign
# assumption is needed anywhere.
#
# Motor layout, Crazyflie 2.x in X configuration, viewed from above:
#       M4  M1          M1 front-right   M2 rear-right
#         \/            M3 rear-left     M4 front-left
#         /\
#       M3  M2
# More thrust at the rear pitches the nose down and accelerates FORWARD (+x).
# More thrust on the right rolls left and accelerates LEFT (+y, ENU right-handed).

MOTOR_TEST_MIN_ERR_M = 0.10
LOOP_ANGLE_TOLERANCE_DEG = 30.0


def tilt_from_motors(m1, m2, m3, m4):
    """
    Direction the drone is being accelerated, in the body frame (x fwd, y left),
    inferred from the motor mix. Returns an unnormalised (ax, ay).
    """
    front, rear = m1 + m4, m2 + m3
    right, left = m1 + m2, m3 + m4
    return (rear - front), (right - left)


def analyze_motor_response(samples):
    """
    samples: (err_x, err_y, m1, m2, m3, m4) where err = setpoint - position,
    in WORLD coordinates, captured with the drone's nose held along world +X.

    A correct loop accelerates along +err (toward the setpoint), so the angle
    between the motor-inferred acceleration and err should be ~0.
        ~0   deg -> loop sign correct
        ~180 deg -> inverted: corrections push AWAY from the setpoint
        ~+-90 deg -> the yaw the EKF is using is 90 deg off the truth
    """
    used, angles = 0, []
    for ex, ey, m1, m2, m3, m4 in samples:
        mag = math.hypot(ex, ey)
        if mag < MOTOR_TEST_MIN_ERR_M:
            continue
        ax, ay = tilt_from_motors(m1, m2, m3, m4)
        if math.hypot(ax, ay) < 1.0:
            continue          # motors essentially level; no information
        used += 1
        angles.append(wrap180(math.degrees(math.atan2(ay, ax))
                              - math.degrees(math.atan2(ey, ex))))
    if used < 15:
        return {"error": f"only {used} usable samples (need >= 15 with the "
                         f"drone held at least "
                         f"{MOTOR_TEST_MIN_ERR_M*100:.0f} cm off the setpoint)"}
    mean = circular_mean_deg(angles)
    spread = math.sqrt(sum(wrap180(a - mean) ** 2 for a in angles) / len(angles))
    return {
        "n": used,
        "angle_error_deg": mean,
        "angle_spread_deg": spread,
        "ok": abs(mean) <= LOOP_ANGLE_TOLERANCE_DEG,
    }


def report_motor_test(r):
    L = ["", "=" * 72, "CLOSED-LOOP SIGN TEST", "=" * 72]
    if "error" in r:
        return "\n".join(L + [f"  UNUSABLE: {r['error']}", "=" * 72])
    a, sp = r["angle_error_deg"], r["angle_spread_deg"]
    L += [f"  samples      : {r['n']}",
          f"  angle between commanded acceleration and the direction to the",
          f"  setpoint     : {a:+.1f} deg  (spread {sp:.1f} deg)",
          ""]
    if abs(a) <= LOOP_ANGLE_TOLERANCE_DEG:
        L += ["  PASS -- the loop pushes the drone toward its setpoint.",
              "  The control frame is correct. Look elsewhere: thrust/weight",
              "  margin, battery sag, prop damage, or controller gains."]
    elif abs(a) > 150.0:
        L += ["  FAIL -- INVERTED. Corrections push the drone AWAY from the",
              "  setpoint, so it diverges every flight regardless of gains.",
              "",
              "  The yaw the EKF is using is ~180 deg from the truth. Either:",
              "    * the rigid body's X axis points out the drone's TAIL, or",
              "    * you are in --pos-only and the drone was not physically",
              "      aligned with world +X when the estimator was reset.",
              "  Fix: re-create the rigid body with the nose along world +X."]
    elif 60.0 < abs(a) < 120.0:
        L += [f"  FAIL -- the loop is rotated {a:+.0f} deg, i.e. roughly a",
              "  quarter turn. The rigid body's axes are swapped relative to",
              "  the drone (X axis out of a side rather than the nose).",
              "  Fix: re-create the rigid body with the nose along world +X."]
    else:
        L += [f"  FAIL -- the loop is rotated {a:+.0f} deg. Apply the opposite",
              f"  correction to the rigid body's local frame in Tracker",
              f"  ({-a:+.0f} deg about Z), or re-create it with the nose along",
              f"  world +X."]
    if sp > 45.0:
        L += ["",
              f"  NOTE: {sp:.0f} deg of spread is high. If it is not a steady",
              "  rotation, suspect an intermittent solution (marker ambiguity)",
              "  rather than a fixed offset."]
    L.append("=" * 72)
    return "\n".join(L)
