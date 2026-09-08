#!/usr/bin/env python3
"""Make track_monitor able to answer its own question.

The first run reported 88 yaw flips, 79 with all 7 markers visible, and
concluded "two problems". Not supported: the tool measures only YAW, and yaw
is a PARAMETER of an orientation. Near pitch = +-90 deg it is ill-conditioned
-- nudging the body 0.4 deg through pitch = 90 swings yaw a full 180 deg while
the body barely moves. A genuine branch flip also shows d_yaw = 180. The run
was hand-held through steep attitudes, exactly where the artefact lives.

    nudge through pitch=90    d_yaw = 180.0   d_att =   0.4
    genuine 180 branch flip   d_yaw = 180.0   d_att = 180.0
"""
import os, shutil, sys, time

F = "track_monitor.py"
if not os.path.exists(F):
    sys.exit("not found: %s -- run from the cf_vicon_stack root." % F)
src = open(F).read()
if "_att_change_deg" in src:
    sys.exit("Already applied; nothing to do.")

EDITS = []
EDITS.append(('''def _yaw_ref(qx, qy, qz, qw):
    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                                   1.0 - 2.0 * (qy * qy + qz * qz)))''',
'''def _yaw_ref(qx, qy, qz, qw):
    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                                   1.0 - 2.0 * (qy * qy + qz * qz)))


def _roll_pitch(qx, qy, qz, qw):
    """ZYX roll and pitch, the same convention as the yaw above."""
    s = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    return (math.degrees(math.atan2(2.0 * (qw * qx + qy * qz),
                                    1.0 - 2.0 * (qx * qx + qy * qy))),
            math.degrees(math.asin(s)))


def _att_change_deg(q1, q2):
    """Geodesic angle between two orientations, in degrees.

    This separates the two things a yaw jump can mean. Yaw is a PARAMETER of
    an orientation and is ill-conditioned near pitch = +-90 deg: nudging the
    body 0.4 deg through pitch = 90 swings the reported yaw a full 180 deg.
    The angle between the quaternions themselves has no singularity.

        nudge through pitch=90    d_yaw = 180.0   d_att =   0.4
        genuine 180 branch flip   d_yaw = 180.0   d_att = 180.0

    Judge TRACKING on d_att. Judge the CONTROLLER on yaw, because yaw is what
    it flies on -- a parameterisation blow-up is still a real problem for a
    yaw-rate controller, just not evidence of the tracker losing the body.
    """
    d = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, d))))''',
"helpers: roll/pitch and the geodesic attitude change"))

EDITS.append(('''        self.prev = None                  # (t, yaw)
        self.worst_rate = 0.0
        self.csv = open(csv_path, "w") if csv_path else None
        if self.csv:
            self.csv.write("t,yaw_deg,d_yaw,visible,expected,quality,flip\\n")''',
'''        self.prev = None                  # (t, yaw, quat)
        self.worst_rate = 0.0
        # Every flip is classified twice: did the markers go dark, and did the
        # BODY actually move (see _att_change_deg). Only flips where the body
        # moved say anything about tracking.
        self.flips_real = 0
        self.flips_param = 0
        self.flips_real_loss = 0
        self.flips_real_clean = 0
        self.worst_pitch_param = 0.0
        self.csv = open(csv_path, "w") if csv_path else None
        if self.csv:
            self.csv.write("t,yaw_deg,d_yaw,roll_deg,pitch_deg,d_att_deg,"
                           "visible,expected,quality,flip\\n")''',
"__init__: counters and the wider CSV header"))

EDITS.append(('''        now = time.monotonic()
        yaw = self.yaw_fn((o.x, o.y, o.z, o.w))
        self.n += 1
        d = 0.0
        flip = 0
        if self.prev is not None:
            dt = now - self.prev[0]
            d = wrap180(yaw - self.prev[1])
            if 0 < dt <= FLIP_GAP_CAP_S:
                self.worst_rate = max(self.worst_rate, abs(d) / dt)
                if abs(d) >= MIN_FLIP_DEG:
                    flip = 1
                    self.flips += 1
                    # was the marker set incomplete within the last 200 ms?
                    if now - self.last_partial_t < 0.2:
                        self.flips_with_loss += 1
                    else:
                        self.flips_clean += 1
        self.prev = (now, yaw)
        if self.csv:
            self.csv.write("%.4f,%.2f,%.2f,%s,%s,%.3f,%d\\n"
                           % (now - self.t0, yaw, d,
                              self.visible if self.visible is not None else "",
                              self.expected if self.expected is not None else "",
                              self.quality, flip))''',
'''        now = time.monotonic()
        q = (o.x, o.y, o.z, o.w)
        yaw = self.yaw_fn(q)
        roll, pitch = _roll_pitch(*q)
        self.n += 1
        d = 0.0
        d_att = 0.0
        flip = 0
        if self.prev is not None:
            dt = now - self.prev[0]
            d = wrap180(yaw - self.prev[1])
            d_att = _att_change_deg(q, self.prev[2])
            if 0 < dt <= FLIP_GAP_CAP_S:
                self.worst_rate = max(self.worst_rate, abs(d) / dt)
                if abs(d) >= MIN_FLIP_DEG:
                    flip = 1
                    self.flips += 1
                    # was the marker set incomplete within the last 200 ms?
                    lost = (now - self.last_partial_t) < 0.2
                    if lost:
                        self.flips_with_loss += 1
                    else:
                        self.flips_clean += 1
                    # did the BODY move, or only the angle convention?
                    if d_att >= MIN_FLIP_DEG:
                        self.flips_real += 1
                        if lost:
                            self.flips_real_loss += 1
                        else:
                            self.flips_real_clean += 1
                    else:
                        self.flips_param += 1
                        self.worst_pitch_param = max(self.worst_pitch_param,
                                                     abs(pitch))
        self.prev = (now, yaw, q)
        if self.csv:
            self.csv.write("%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s,%.3f,%d\\n"
                           % (now - self.t0, yaw, d, roll, pitch, d_att,
                              self.visible if self.visible is not None else "",
                              self.expected if self.expected is not None else "",
                              self.quality, flip))''',
"_pose_cb: compute and classify"))

EDITS.append(('''        else:
            print("    with marker loss within 200 ms : %d" % self.flips_with_loss)
            print("    with a complete marker set     : %d" % self.flips_clean)
            print()
            if self.flips_with_loss and not self.flips_clean:
                print("  Every flip coincided with markers going dark. That is "
                      "Findings 1.7\\n  confirmed directly: the fix is marker "
                      "GEOMETRY and placement, not\\n  filtering, not the "
                      "controller, not the radio.")
            elif self.flips_clean and not self.flips_with_loss:
                print("  No flip coincided with marker loss, so occlusion is NOT "
                      "the cause\\n  here. Look at template quality and marker "
                      "labelling in Tracker.")
            else:
                print("  Mixed: %d with loss, %d without. Occlusion explains part "
                      "of it and\\n  something else explains the rest -- treat "
                      "them as two problems."
                      % (self.flips_with_loss, self.flips_clean))''',
'''        else:
            print("    with marker loss within 200 ms : %d" % self.flips_with_loss)
            print("    with a complete marker set     : %d" % self.flips_clean)
            print()
            print("  Did the BODY move, or only the yaw parameter?")
            print("    real  (orientation moved >= %.0f deg) : %d"
                  % (MIN_FLIP_DEG, self.flips_real))
            print("    param (orientation barely moved)      : %d"
                  "    worst |pitch| at one: %.0f deg"
                  % (self.flips_param, self.worst_pitch_param))
            print()
            if not self.flips_real:
                print("  NOT ONE of these was a real reorientation. Yaw is a\\n"
                      "  parameter, ill-conditioned near pitch = +-90 deg, and\\n"
                      "  yaw is what swung -- the body did not. This is the angle\\n"
                      "  convention, not the tracker. It is still a problem for a\\n"
                      "  yaw-rate controller, but it is not evidence about markers.")
                return
            if self.flips_param:
                print("  %d of the %d were parameterisation artefacts and say\\n"
                      "  nothing about tracking. The %d real ones split:"
                      % (self.flips_param, self.flips, self.flips_real))
                print()
            print("    real, with marker loss    : %d" % self.flips_real_loss)
            print("    real, complete marker set : %d" % self.flips_real_clean)
            print()
            if self.flips_real_loss and not self.flips_real_clean:
                print("  Every real flip coincided with markers going dark. That "
                      "is Findings 1.7\\n  confirmed directly: the fix is marker "
                      "GEOMETRY and placement, not\\n  filtering, not the "
                      "controller, not the radio.")
            elif self.flips_real_clean and not self.flips_real_loss:
                print("  No real flip coincided with marker loss, so occlusion is "
                      "NOT the\\n  cause here. The template itself is rotationally "
                      "ambiguous: run\\n  marker_geom.py on the FULL marker set "
                      "and read the best\\n  non-identity competitor.")
            else:
                print("  Mixed: %d real flips with loss, %d without. Occlusion "
                      "explains part\\n  of it; template ambiguity is the "
                      "candidate for the rest."
                      % (self.flips_real_loss, self.flips_real_clean))''',
"report: attribute occlusion only against real flips"))

print("checking anchors in %s ..." % F)
for old, _, label in EDITS:
    n = src.count(old)
    if n != 1:
        sys.exit("\nANCHOR %s for '%s' (found %d). Nothing written."
                 % ("NOT FOUND" if n == 0 else "NOT UNIQUE", label, n))
    print("  ok    %s" % label)
if "--apply" not in sys.argv:
    sys.exit("\nreport only. re-run with --apply to write.")
bak = "_backup_attitude_" + time.strftime("%Y%m%d-%H%M%S")
os.makedirs(bak, exist_ok=True)
shutil.copy2(F, os.path.join(bak, F))
for old, new, _ in EDITS:
    src = src.replace(old, new, 1)
open(F, "w").write(src)
print("\n  backed up to %s/ and wrote %s" % (bak, F))
