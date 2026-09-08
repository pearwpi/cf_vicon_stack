"""
Validate frame_check against synthetic rigid bodies with KNOWN defects.

A bench test that reports a confident wrong number is worse than no test: it
sends you to change the Vicon world calibration when the real fault was a
40 degree object rotation. So every defect below is injected on purpose and the
recovered value is checked against ground truth.
"""
import math
import random
import sys

import frame_check as F

fails = []


def check(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name} {detail}")
    if not cond:
        fails.append(name)


def slide(travel_heading_deg, reported_yaw_deg, dist=0.40, n=60, noise=0.002,
          wobble=0.0, seed=1):
    """
    Simulate sliding the drone in a straight line.

    The drone physically travels along `travel_heading_deg`. The rigid body
    reports `reported_yaw_deg`. A correct rigid body has these equal; a constant
    object rotation makes them differ, and that difference is what we recover.
    """
    rnd = random.Random(seed)
    h = math.radians(travel_heading_deg)
    start = (0.30, -0.20, 0.02)
    end = (start[0] + dist * math.cos(h),
           start[1] + dist * math.sin(h),
           start[2])
    s = (start[0] + rnd.gauss(0, noise), start[1] + rnd.gauss(0, noise), start[2])
    e = (end[0] + rnd.gauss(0, noise), end[1] + rnd.gauss(0, noise), end[2])
    yaws = [reported_yaw_deg + rnd.gauss(0, 0.5) + wobble * (i / n)
            for i in range(n)]
    return s, e, yaws


print("\n[A] a correctly built rigid body reports no offset")
for h in (0.0, 45.0, 130.0, -170.0):
    s, e, y = slide(h, h)
    r = F.analyze_slide(s, e, y)
    check(f"heading {h:+.0f}: offset ~0", abs(r["yaw_error_deg"]) < 2.0,
          f"{r['yaw_error_deg']:+.2f} deg")

print("\n[B] recover a KNOWN constant yaw offset")
for truth in (10.0, -25.0, 40.0, 95.0, -120.0, 179.0):
    # Drone physically travels at 60 deg; rigid body under-reports by `truth`
    s, e, y = slide(60.0, 60.0 - truth)
    r = F.analyze_slide(s, e, y)
    check(f"offset {truth:+.0f} deg recovered",
          abs(F.wrap180(r["yaw_error_deg"] - truth)) < 3.0,
          f"est={r['yaw_error_deg']:+.1f}")

print("\n[C] the +-180 seam does not break the estimate")
s, e, y = slide(175.0, -175.0)          # 10 deg apart across the seam
r = F.analyze_slide(s, e, y)
check("seam handled", abs(F.wrap180(r["yaw_error_deg"] - (-10.0))) < 3.0,
      f"est={r['yaw_error_deg']:+.1f} (truth -10)")
check("circular mean is seam-safe",
      abs(F.wrap180(F.circular_mean_deg([179.0, -179.0]) - 180.0)) < 1.0,
      f"{F.circular_mean_deg([179.0, -179.0]):+.1f}")

print("\n[D] unusable slides are rejected, not silently averaged")
s, e, y = slide(0.0, 0.0, dist=0.05)     # barely moved
check("short slide rejected", "error" in F.analyze_slide(s, e, y),
      F.analyze_slide(s, e, y).get("error", "")[:40])
s, e, y = slide(0.0, 0.0, wobble=70.0)   # rotated a lot mid-slide
check("rotating slide rejected", "error" in F.analyze_slide(s, e, y),
      F.analyze_slide(s, e, y).get("error", "")[:40])

print("\n[E] the left-slide expectation is applied correctly")
# Correct body: sliding to its own left travels at yaw+90
s, e, y = slide(90.0, 0.0)
r = F.analyze_slide(s, e, y, expected_offset_deg=90.0)
check("correct left slide -> ~0 offset", abs(r["yaw_error_deg"]) < 2.0,
      f"{r['yaw_error_deg']:+.2f}")
# A MIRRORED frame sends 'left' to yaw-90 instead
s, e, y = slide(-90.0, 0.0)
r = F.analyze_slide(s, e, y, expected_offset_deg=90.0)
check("mirrored frame shows a large offset", abs(r["yaw_error_deg"]) > 150.0,
      f"{r['yaw_error_deg']:+.1f}")


def rotate_in_place(dx, dy, center=(0.4, -0.3), n=200, span=360.0,
                    noise=0.002, drift=0.0, seed=2):
    """
    Simulate turning the drone on the spot when the object's origin is offset
    from the true centre of rotation by (dx, dy) in BODY coordinates.
        p_reported = centre + R(yaw) * d
    `drift` translates the true centre during the turn, as a human would.
    """
    rnd = random.Random(seed)
    out = []
    for i in range(n):
        yaw = F.wrap180(-180.0 + span * i / n)
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        cxi = center[0] + drift * (i / n)
        cyi = center[1] - 0.5 * drift * (i / n)
        out.append((cxi + dx * c - dy * s + rnd.gauss(0, noise),
                    cyi + dx * s + dy * c + rnd.gauss(0, noise),
                    yaw))
    return out


print("\n[F] recover a KNOWN rigid-body origin offset")
for truth in ((0.0, 0.0), (0.04, 0.0), (0.0, -0.03), (0.05, 0.05)):
    r = F.fit_origin_offset(rotate_in_place(*truth))
    got = r["offset_body"]
    err = math.dist(got, truth)
    check(f"offset {truth} recovered", err < 0.006,
          f"got ({got[0]:+.3f},{got[1]:+.3f}), err {err*1000:.1f} mm")

print("\n[G] a centred origin passes; an offset one fails")
check("centred passes", F.fit_origin_offset(rotate_in_place(0.002, 0.0))["ok"])
check("40 mm offset fails", not F.fit_origin_offset(rotate_in_place(0.04, 0.0))["ok"])

print("\n[H] the fit survives the human drifting the drone while turning")
r = F.fit_origin_offset(rotate_in_place(0.04, 0.0, drift=0.06))
check("offset still recovered with 6 cm of drift",
      abs(r["offset_body"][0] - 0.04) < 0.012,
      f"got {r['offset_body'][0]:+.3f} (truth +0.040)")

print("\n[I] an under-rotated capture is refused, not guessed")
r = F.fit_origin_offset(rotate_in_place(0.04, 0.0, span=40.0))
check("small rotation refused", "error" in r, r.get("error", "")[:50])
check("too few samples refused",
      "error" in F.fit_origin_offset(rotate_in_place(0.04, 0.0, n=5)))

print("\n[J] end-to-end report: correct body -> PASS")
s1, e1, y1 = slide(30.0, 30.0)
s2, e2, y2 = slide(120.0, 30.0)
rep = F.report(F.analyze_slide(s1, e1, y1),
               F.analyze_slide(s2, e2, y2, expected_offset_deg=90.0),
               0.35, F.fit_origin_offset(rotate_in_place(0.001, 0.0)), 0, 0.6)
check("reports PASS", "PASS --" in rep)
check("no yaw offset claimed", "CONSTANT YAW OFFSET" not in rep)

print("\n[K] end-to-end: the failure Ronit actually has")
# Markers rebuilt so the flips are gone, but the object was created with the
# nose 100 deg away from world +X -> constant offset -> guaranteed divergence.
TRUTH = 100.0
s1, e1, y1 = slide(30.0, 30.0 - TRUTH)
s2, e2, y2 = slide(120.0, 30.0 - TRUTH)
rep = F.report(F.analyze_slide(s1, e1, y1),
               F.analyze_slide(s2, e2, y2, expected_offset_deg=90.0),
               0.35, F.fit_origin_offset(rotate_in_place(0.002, 0.0)), 0, 0.8)
check("constant yaw offset flagged", "CONSTANT YAW OFFSET" in rep)
check("magnitude reported correctly", "+100 deg" in rep or "+99 deg" in rep
      or "+101 deg" in rep, [l for l in rep.splitlines() if "OFFSET of" in l])
check("divergence called out", "DIVERGE" in rep)
check("gives the Tracker correction", "-100 deg" in rep or "-99 deg" in rep
      or "-101 deg" in rep)
check("does NOT blame the world calibration", "world calibration" not in rep)

print("\n[L] end-to-end: still-flipping markers are called out above all else")
rep = F.report(F.analyze_slide(*slide(30.0, 30.0)), None, 0.35, None, 14, 178.0)
check("flips ranked first", rep.split("1. [")[1].startswith("PRIMARY"))
check("names the ambiguity", "ambiguous" in rep.lower())
check("says to re-create the body", "re-create" in rep)

print("\n[M] inverted Z is caught")
rep = F.report(F.analyze_slide(*slide(0.0, 0.0)), None, -0.34, None, 0, 0.5)
check("Z inversion flagged", "Z is" in rep and "inverted" in rep)

print("\n[N] closed-loop sign test: motor mix -> commanded direction")
def motor_samples(loop_angle_deg, n=60, hover=32000, gain=6000, noise=200,
                  err_headings=(0.0, 45.0, 90.0, 135.0, 180.0, -90.0), seed=9):
    """
    Simulate the props-off bench test. The drone is displaced by `err` from the
    setpoint; the loop commands an acceleration rotated by `loop_angle_deg` from
    the ideal (which points straight at the setpoint). Convert that commanded
    acceleration back into the motor mix the firmware would produce.
    """
    rnd = random.Random(seed)
    out = []
    for i in range(n):
        h = math.radians(err_headings[i % len(err_headings)])
        mag = 0.15 + 0.1 * ((i % 3) / 2.0)
        ex, ey = mag * math.cos(h), mag * math.sin(h)
        # commanded acceleration = direction to setpoint, rotated by the fault
        ca = math.atan2(ey, ex) + math.radians(loop_angle_deg)
        ax, ay = math.cos(ca), math.sin(ca)
        # invert tilt_from_motors: ax = rear-front, ay = right-left
        front = hover - gain * ax / 2; rear = hover + gain * ax / 2
        right = gain * ay / 2;         left = -gain * ay / 2
        m1 = front / 2 + right / 2 + rnd.gauss(0, noise)   # front-right
        m4 = front / 2 + left / 2 + rnd.gauss(0, noise)    # front-left
        m2 = rear / 2 + right / 2 + rnd.gauss(0, noise)    # rear-right
        m3 = rear / 2 + left / 2 + rnd.gauss(0, noise)     # rear-left
        out.append((ex, ey, m1, m2, m3, m4))
    return out

for truth in (0.0, 30.0, 90.0, 180.0, -90.0, -45.0):
    r = F.analyze_motor_response(motor_samples(truth))
    check(f"loop angle {truth:+.0f} recovered",
          abs(F.wrap180(r["angle_error_deg"] - truth)) < 12.0,
          f"est={r['angle_error_deg']:+.1f} spread={r['angle_spread_deg']:.0f}")

print("\n[O] verdicts name the right fault")
check("correct loop passes", F.analyze_motor_response(motor_samples(0.0))["ok"])
rep = F.report_motor_test(F.analyze_motor_response(motor_samples(180.0)))
check("inverted loop -> INVERTED", "INVERTED" in rep)
check("inverted loop -> names tail/pos-only cause",
      "TAIL" in rep and "pos-only" in rep)
rep = F.report_motor_test(F.analyze_motor_response(motor_samples(90.0)))
check("quarter-turn -> named as such", "quarter turn" in rep)
rep = F.report_motor_test(F.analyze_motor_response(motor_samples(0.0)))
check("passing loop points elsewhere", "Look elsewhere" in rep)
check("unusable input refused",
      "error" in F.analyze_motor_response([(0.01, 0.0, 1, 1, 1, 1)] * 40))

print("\n[P] tilt_from_motors direction sanity")
# more thrust at the rear (m2, m3) -> nose down -> accelerate FORWARD (+x)
ax, ay = F.tilt_from_motors(1000, 2000, 2000, 1000)
check("rear-heavy -> +x", ax > 0 and abs(ay) < 1e-9, f"({ax},{ay})")
# more thrust on the right (m1, m2) -> rolls left -> accelerate LEFT (+y)
ax, ay = F.tilt_from_motors(2000, 2000, 1000, 1000)
check("right-heavy -> +y (left)", ay > 0 and abs(ax) < 1e-9, f"({ax},{ay})")

print("\n" + "=" * 60)
print("FAILED:" if fails else "ALL CHECKS PASSED", fails if fails else "")
sys.exit(1 if fails else 0)
