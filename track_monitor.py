#!/usr/bin/env python3
"""
track_monitor.py -- watch marker visibility and yaw flips together, live.

Run this in a second terminal during a flight. It answers, directly and at the
moment it happens, the question five sessions of debugging had to infer
backwards from a yaw histogram:

    when the reported yaw jumps, had markers just gone dark?

Findings 1.7 predicts yes. This measures it.

    python3 track_monitor.py --topic /vicon/crazyflie2/crazyflie2
    python3 track_monitor.py --topic ... --csv track_log.csv

Needs a bridge new enough to publish the companion topic <topic>/quality.
Without it the marker columns read "--" and only the yaw side works;
rebuild the vicon_receiver image if you see that.

Touches nothing the drone flies on. Read-only, in its own process.
"""
import argparse
import math
import sys
import time

try:
    from cf_core import MIN_FLIP_DEG, FLIP_GAP_CAP_S, wrap180
    import cf_core as _core
except Exception as exc:                                    # pragma: no cover
    sys.exit("cannot import cf_core (%s)\nRun from inside cf_vicon_stack." % exc)


def _yaw_ref(qx, qy, qz, qw):
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
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, d))))


def _bind_yaw():
    """Use cf_core's yaw function so flips are counted the same way as in flight."""
    fn = getattr(_core, "yaw_deg_of", None)
    if fn is None:
        sys.exit("cf_core has no yaw_deg_of -- stack version mismatch.")
    probes = [(0.0, 0.0, 0.3826834, 0.9238795),
              (0.0, 0.0, 0.7071068, 0.7071068),
              (0.1, -0.2, 0.3, 0.9273618)]
    for name, call in (("4-arg", lambda q: fn(*q)), ("tuple", lambda q: fn(q))):
        try:
            if all(abs(wrap180(call(q) - _yaw_ref(*q))) < 1e-6 for q in probes):
                return call, name
        except Exception:
            continue
    sys.exit("cf_core.yaw_deg_of does not match the reference convention; "
             "refusing to count flips differently from the flight code.")


class Monitor:
    def __init__(self, topic, yaw_fn, csv_path):
        import rclpy
        from rclpy.node import Node
        from rosidl_runtime_py.utilities import get_message
        self._rclpy = rclpy
        rclpy.init()
        self.node = Node("track_monitor")
        self.yaw_fn = yaw_fn
        self.t0 = time.monotonic()

        mtype = None
        for _ in range(60):
            for name, types in self.node.get_topic_names_and_types():
                if name == topic and types:
                    mtype = types[0]
                    break
            if mtype:
                break
            time.sleep(0.1)
        if not mtype:
            sys.exit("topic %s not found -- is the bridge running?" % topic)
        self.node.create_subscription(get_message(mtype), topic, self._pose_cb, 20)
        print("  pose    %s  [%s]" % (topic, mtype))

        qtopic = topic.rstrip("/") + "/quality"
        self.have_quality = False
        for name, types in self.node.get_topic_names_and_types():
            if name == qtopic and types:
                self.node.create_subscription(get_message(types[0]), qtopic,
                                              self._quality_cb, 20)
                self.have_quality = True
                print("  quality %s  [%s]" % (qtopic, types[0]))
        if not self.have_quality:
            print("  quality %s  NOT PUBLISHED -- rebuild the "
                  "vicon_receiver image" % qtopic)

        self.visible = self.expected = None
        self.quality = -1.0
        self.last_partial_t = -1e9        # when markers were last incomplete
        self.n = self.flips = 0
        self.flips_with_loss = 0
        self.flips_clean = 0
        self.min_visible = None
        self.partial_frames = 0
        self.prev = None                  # (t, yaw, quat)
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
                           "visible,expected,quality,flip\n")

    # -- callbacks ---------------------------------------------------------
    def _quality_cb(self, msg):
        v = getattr(msg, "vector", None)
        if v is None:
            return
        self.visible, self.expected, self.quality = int(v.x), int(v.y), float(v.z)
        if self.expected and self.visible < self.expected:
            self.partial_frames += 1
            self.last_partial_t = time.monotonic()
        if self.min_visible is None or self.visible < self.min_visible:
            self.min_visible = self.visible

    def _pose_cb(self, msg):
        p = getattr(msg, "pose", None)
        if p is not None and hasattr(p, "orientation"):
            o = p.orientation
        else:
            t = getattr(msg, "transform", None)
            if t is None:
                return
            o = t.rotation
        now = time.monotonic()
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
            self.csv.write("%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s,%.3f,%d\n"
                           % (now - self.t0, yaw, d, roll, pitch, d_att,
                              self.visible if self.visible is not None else "",
                              self.expected if self.expected is not None else "",
                              self.quality, flip))

    def line(self):
        mk = ("%d/%d" % (self.visible, self.expected)
              if self.expected else "--")
        q = "%.2f" % self.quality if self.quality >= 0 else " -- "
        warn = ""
        if self.expected and self.visible is not None and self.visible < self.expected:
            warn = "  <-- MARKERS LOST"
        return ("  %6.1fs  n=%-6d  markers %-6s  q=%s  yaw=%7.1f  "
                "FLIPS=%-4d (%d w/ marker loss, %d clean)%s"
                % (time.monotonic() - self.t0, self.n, mk, q,
                   self.prev[1] if self.prev else 0.0,
                   self.flips, self.flips_with_loss, self.flips_clean, warn))

    def spin(self):
        try:
            last = 0.0
            while self._rclpy.ok():
                self._rclpy.spin_once(self.node, timeout_sec=0.05)
                if time.monotonic() - last > 0.25:
                    last = time.monotonic()
                    sys.stdout.write("\r" + self.line() + " " * 6)
                    sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        finally:
            print("\n")
            self.report()
            if self.csv:
                self.csv.close()
            try:
                self.node.destroy_node()
                self._rclpy.shutdown()
            except Exception:
                pass

    def report(self):
        print("  " + "=" * 66)
        print("  samples          : %d over %.1f s" % (self.n, time.monotonic() - self.t0))
        if self.expected:
            print("  markers          : %d expected, fewest seen %d"
                  % (self.expected, self.min_visible if self.min_visible is not None else -1))
            pct = 100.0 * self.partial_frames / max(1, self.n)
            print("  partial frames   : %d (%.1f%% of the flight tracked on an "
                  "incomplete set)" % (self.partial_frames, pct))
        print("  yaw flips >%.0f deg: %d   peak rate %.0f deg/s"
              % (MIN_FLIP_DEG, self.flips, self.worst_rate))
        if not self.flips:
            print("\n  No flips. If markers also stayed complete, that is the "
                  "acceptance test passed.")
        elif not self.have_quality:
            print("\n  Cannot attribute these: the quality topic was not "
                  "published.\n  Rebuild the vicon_receiver image.")
        else:
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
                print("  NOT ONE of these was a real reorientation. Yaw is a\n"
                      "  parameter, ill-conditioned near pitch = +-90 deg, and\n"
                      "  yaw is what swung -- the body did not. This is the angle\n"
                      "  convention, not the tracker. It is still a problem for a\n"
                      "  yaw-rate controller, but it is not evidence about markers.")
                return
            if self.flips_param:
                print("  %d of the %d were parameterisation artefacts and say\n"
                      "  nothing about tracking. The %d real ones split:"
                      % (self.flips_param, self.flips, self.flips_real))
                print()
            print("    real, with marker loss    : %d" % self.flips_real_loss)
            print("    real, complete marker set : %d" % self.flips_real_clean)
            print()
            if self.flips_real_loss and not self.flips_real_clean:
                print("  Every real flip coincided with markers going dark. That "
                      "is Findings 1.7\n  confirmed directly: the fix is marker "
                      "GEOMETRY and placement, not\n  filtering, not the "
                      "controller, not the radio.")
            elif self.flips_real_clean and not self.flips_real_loss:
                print("  No real flip coincided with marker loss, so occlusion is "
                      "NOT the\n  cause here. The template itself is rotationally "
                      "ambiguous: run\n  marker_geom.py on the FULL marker set "
                      "and read the best\n  non-identity competitor.")
            else:
                print("  Mixed: %d real flips with loss, %d without. Occlusion "
                      "explains part\n  of it; template ambiguity is the "
                      "candidate for the rest."
                      % (self.flips_real_loss, self.flips_real_clean))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", required=True)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    yaw_fn, shape = _bind_yaw()
    print("\n  yaw convention: cf_core.yaw_deg_of (%s) matches reference\n" % shape)
    Monitor(a.topic, yaw_fn, a.csv).spin()


if __name__ == "__main__":
    main()
