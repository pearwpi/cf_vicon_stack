#!/usr/bin/env python3
"""
spin_test.py -- bench vibration test for mocap rigid-body ambiguity.

WHY THIS EXISTS
---------------
Yaw flips on this airframe appear only under thrust.  Hand provocation cannot
reproduce them (a hand-rotated drone does not vibrate), so until now the only
test was a flight -- one battery and one crash risk per attempt.

This spins the motors WITH THE PROPELLERS REMOVED while watching the Vicon
yaw, and reports the throttle at which the rigid body starts flipping.  Zero
thrust, zero flight risk, ~60 seconds.

HONEST LIMITATION: props-off vibration is weaker than props-on (no aero load,
no blade imbalance).  A FAIL here is conclusive.  A PASS is encouraging but
does not fully clear the body -- confirm with one hover watching YAWFLIP.

    PROPELLERS MUST BE OFF.  The script sets raw motor power and bypasses the
    commander.  With props on this WILL take off unrestrained.

Usage:
    python3 spin_test.py --topic /vicon/crazyflie2/crazyflie2
    python3 spin_test.py --topic ... --dry-run      # Vicon only, no radio
    python3 spin_test.py --topic ... --max-pct 60   # default 45
"""
import argparse
import atexit
import math
import signal
import sys
import threading
import time

# --- one source of truth for the flip thresholds -----------------------------
try:
    from cf_core import MIN_FLIP_DEG, FLIP_GAP_CAP_S, wrap180
    import cf_core as _core
except Exception as exc:                                    # pragma: no cover
    sys.exit("cannot import cf_core (%s)\nRun this from inside cf_vicon_stack." % exc)

MOTORS = ("m1", "m2", "m3", "m4")
PWM_MAX = 65535


def _yaw_local(qx, qy, qz, qw):
    """Reference yaw.  Must agree with cf_core; verified at startup."""
    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                                   1.0 - 2.0 * (qy * qy + qz * qz)))


def _bind_yaw():
    """Use cf_core's yaw function, whatever its call shape, or abort loudly."""
    fn = getattr(_core, "yaw_deg_of", None)
    if fn is None:
        sys.exit("cf_core has no yaw_deg_of -- stack version mismatch.")
    probes = [(0.0, 0.0, 0.3826834, 0.9238795),      # +45 deg
              (0.0, 0.0, 0.7071068, 0.7071068),      # +90 deg
              (0.1, -0.2, 0.3, 0.9273618)]           # arbitrary
    shapes = [
        ("4-arg", lambda q: fn(q[0], q[1], q[2], q[3])),
        ("tuple", lambda q: fn(q)),
    ]
    for name, call in shapes:
        try:
            if all(abs(wrap180(call(q) - _yaw_local(*q))) < 1e-6 for q in probes):
                return call, name
        except Exception:
            continue
    sys.exit("cf_core.yaw_deg_of does not match the reference convention.\n"
             "Refusing to run: this test would count flips differently from flight code.")


# --- Vicon listener ----------------------------------------------------------
def _extract(msg):
    p = getattr(msg, "pose", None)
    if p is not None and hasattr(p, "position"):
        return (p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    t = getattr(msg, "transform", None)
    if t is not None:
        return (t.translation.x, t.translation.y, t.translation.z,
                t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    if hasattr(msg, "x_trans"):
        return (msg.x_trans / 1000.0, msg.y_trans / 1000.0, msg.z_trans / 1000.0,
                msg.x_rot, msg.y_rot, msg.z_rot, msg.w_rot)
    raise TypeError("unrecognised pose message: %s" % type(msg).__name__)


class Listener:
    """Timestamped yaw samples off the Vicon topic, in a background thread."""

    def __init__(self, topic, yaw_fn):
        import rclpy
        from rclpy.node import Node
        from rosidl_runtime_py.utilities import get_message
        self._rclpy = rclpy
        rclpy.init()
        self.node = Node("spin_test")
        self.samples = []                       # (t, yaw_deg, occluded)
        self._lock = threading.Lock()
        self._yaw = yaw_fn

        mtype = None
        for _ in range(50):                     # topic may not be up yet
            for name, types in self.node.get_topic_names_and_types():
                if name == topic and types:
                    mtype = types[0]
                    break
            if mtype:
                break
            time.sleep(0.1)
        if not mtype:
            sys.exit("topic %s not found -- is the bridge running?" % topic)
        print("  topic %s  type %s" % (topic, mtype))
        self.node.create_subscription(get_message(mtype), topic, self._cb, 10)

        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._spin, daemon=True)
        self._thr.start()

    def _cb(self, msg):
        x, y, z, qx, qy, qz, qw = _extract(msg)
        occ = _core.occluded_sentinel(x, y, z, qx, qy, qz, qw) \
            if hasattr(_core, "occluded_sentinel") else False
        with self._lock:
            self.samples.append((time.monotonic(), self._yaw((qx, qy, qz, qw)), occ))

    def _spin(self):
        while not self._stop.is_set():
            self._rclpy.spin_once(self.node, timeout_sec=0.05)

    def window(self, t0, t1):
        with self._lock:
            return [s for s in self.samples if t0 <= s[0] <= t1]

    def close(self):
        self._stop.set()
        self._thr.join(timeout=1.0)
        try:
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


def score(win):
    """flips, peak deg/s, yaw sd, visibility %  for one dwell window."""
    good = [(t, y) for t, y, occ in win if not occ]
    vis = 100.0 * len(good) / max(1, len(win))
    flips, peak = 0, 0.0
    for (t0, y0), (t1, y1) in zip(good, good[1:]):
        dt = t1 - t0
        if dt <= 0 or dt > FLIP_GAP_CAP_S:
            continue
        d = abs(wrap180(y1 - y0))
        peak = max(peak, d / dt)
        if d >= MIN_FLIP_DEG:
            flips += 1
    ys = [y for _, y in good]
    if len(ys) > 1:
        m = sum(ys) / len(ys)
        sd = math.sqrt(sum((y - m) ** 2 for y in ys) / (len(ys) - 1))
    else:
        sd = 0.0
    return len(win), flips, peak, sd, vis


# --- motor control -----------------------------------------------------------
class Motors:
    def __init__(self, uri, dry):
        self.dry, self.cf = dry, None
        if dry:
            return
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        cflib.crtp.init_drivers()
        self.scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache"))
        self.scf.open_link()
        self.cf = self.scf.cf
        try:                                    # newer firmware gates on arming
            self.cf.platform.send_arming_request(True)
            time.sleep(0.3)
        except Exception:
            pass
        self.cf.param.set_value("motorPowerSet.enable", "1")
        time.sleep(0.1)

    def set(self, pct):
        if self.dry:
            return
        v = int(round(PWM_MAX * pct / 100.0))
        for m in MOTORS:
            self.cf.param.set_value("motorPowerSet." + m, str(v))

    def off(self):
        if self.dry or self.cf is None:
            return
        try:
            for m in MOTORS:
                self.cf.param.set_value("motorPowerSet." + m, "0")
            self.cf.param.set_value("motorPowerSet.enable", "0")
            time.sleep(0.1)
            self.cf.platform.send_arming_request(False)
        except Exception:
            pass

    def close(self):
        self.off()
        if not self.dry and self.cf is not None:
            try:
                self.scf.close_link()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", required=True)
    ap.add_argument("--uri", default="radio://0/80/2M/E7E7E7E7E7")
    ap.add_argument("--max-pct", type=float, default=45.0)
    ap.add_argument("--dwell", type=float, default=4.0)
    ap.add_argument("--steps", type=int, default=6)
    ap.add_argument("--dry-run", action="store_true",
                    help="Vicon only; never touches the radio or the motors")
    a = ap.parse_args()

    yaw_fn, shape = _bind_yaw()
    print("\n  yaw convention: cf_core.yaw_deg_of (%s) matches reference" % shape)

    if not a.dry_run:
        print("\n" + "=" * 68)
        print("  REMOVE THE PROPELLERS.  This sets raw motor power directly.")
        print("  Put the drone on the bench in its normal upright orientation,")
        print("  markers visible to Vicon, resting on its own feet -- do not")
        print("  clamp it, the vibration is the point.")
        print("=" * 68)
        if input("\n  Type PROPS OFF to continue: ").strip().upper() != "PROPS OFF":
            sys.exit("aborted.")

    lis = Listener(a.topic, yaw_fn)
    mot = Motors(a.uri, a.dry_run)
    atexit.register(mot.off)
    signal.signal(signal.SIGINT, lambda *_: (mot.off(), sys.exit("\n  stopped.")))

    levels = [0.0] + [a.max_pct * (i + 1) / a.steps for i in range(a.steps)]
    rows = []
    print("\n   pct     pwm   samples  flips   peak deg/s   yaw sd    vis%")
    print("  " + "-" * 60)
    try:
        for pct in levels:
            mot.set(pct)
            t0 = time.monotonic()
            time.sleep(0.4)                     # let the level settle
            time.sleep(a.dwell)
            t1 = time.monotonic()
            n, flips, peak, sd, vis = score(lis.window(t0 + 0.4, t1))
            rows.append((pct, flips, peak, sd, vis, n))
            print("  %5.1f  %6d   %7d  %5d   %10.1f  %7.2f  %6.1f%s"
                  % (pct, int(PWM_MAX * pct / 100.0), n, flips, peak, sd, vis,
                     "   <-- FLIPPING" if flips else ""))
    finally:
        mot.close()
        lis.close()

    print("\n  " + "=" * 60)
    bad = [r for r in rows if r[1] > 0]
    starved = [r for r in rows if r[5] < 10]
    if starved:
        print("  INCONCLUSIVE: no Vicon samples at some levels. Check the bridge")
        print("  and that the drone is inside the capture volume.")
    elif bad:
        print("  FAIL: rigid body flips from %.1f%% throttle upward." % bad[0][0])
        print("        %d flips, peak %.0f deg/s." % (bad[0][1], bad[0][2]))
        print("  The marker constellation is ambiguous under vibration. Fix the")
        print("  layout before flying -- this is not a controller problem.")
    else:
        span = max(r[3] for r in rows)
        print("  PASS at the bench: 0 flips to %.1f%% throttle, worst yaw sd %.2f deg."
              % (a.max_pct, span))
        print("  Props-off vibration is weaker than props-on. Confirm with one")
        print("  hover and watch YAWFLIP during the climb.")
    print()


if __name__ == "__main__":
    main()
