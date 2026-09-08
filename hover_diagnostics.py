#!/usr/bin/env python3
r"""Work out WHY a mocap-fed Crazyflie will not hold position, from a flight CSV.

Gains are the LAST thing to suspect and the most expensive to change, because
altering them invalidates every other observation.

KEY IDEA. A position loop drives de/dt = -k*e. If estimator YAW is wrong by t,
the correction is applied in a rotated frame, de/dt = -k*R(t)*e. Writing the
horizontal error as z = ex + i*ey:

    z(T) = z0 * exp(-k*cos(t)*T) * exp(-i*k*sin(t)*T)
               \___decay___/         \___rotation___/

so a yaw calibration error makes the error ROTATE while it decays: the drone
spirals rather than converging, which to the eye is "it drifts around a lot".
Fit lambda to the recorded error and read it off:
    a = -Re(lambda) = k*cos(t)      omega = Im(lambda) = -k*sin(t)
    t = atan2(-omega, a)            <- the yaw error, in degrees
|t| > ~90 deg makes cos(t) < 0, the decay rate negative, and the loop unstable.
30-40 deg gives a slow persistent orbit that looks exactly like bad tuning.

ALSO SEPARATES: constant offset (trim / CG / rigid-body origin off centre);
fast oscillation (velocity estimate, latency, or genuinely hot gains -- the ONLY
case where touching gains is right); EKF-vs-Vicon gap (estimator not tracking,
fix injection first); small RMS (nothing wrong, 1-3 cm is normal mocap hover).

    python3 crazyflie_vicon_teleop.py --diagnose 20   # fly it, writes CSV
    python3 hover_diagnostics.py hover_log.csv        # or analyse later
"""

from __future__ import annotations

import cmath
import csv
import math
import statistics
import sys

# What counts as acceptable for a mocap Crazyflie holding position.
GOOD_RMS_M = 0.030        # <=3 cm RMS horizontal is a healthy hover
BIAS_SIGNIFICANT_M = 0.050
YAW_SIGNIFICANT_DEG = 12.0
ROTATION_CONSISTENCY = 0.35   # 0 = random walk, 1 = perfectly one-directional
EKF_GAP_SIGNIFICANT_M = 0.050
FAST_OSC_HZ = 1.0

IMPOSSIBLE_YAW_DPS = 720.0   # 2 rev/s -- far above anything a CF does in flight

COLUMNS = ["t", "sp_x", "sp_y", "sp_z", "x", "y", "z",
           "ex", "ey", "ez", "yaw_deg", "latency_ms"]


# ---------------------------------------------------------------------------
def _wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def load_csv(path):
    out = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                out.append({k: float(row[k]) for k in COLUMNS if k in row})
            except (TypeError, ValueError):
                continue
    return out


def _median_complex(vals):
    if not vals:
        return complex(0, 0)
    return complex(statistics.median([v.real for v in vals]),
                   statistics.median([v.imag for v in vals]))


def _rms(vals):
    if not vals:
        return 0.0
    return math.sqrt(sum(abs(v) ** 2 for v in vals) / len(vals))


def _zero_cross_hz(series, dt):
    """Crude but robust dominant-frequency estimate; no numpy dependency."""
    if len(series) < 4 or dt <= 0:
        return 0.0
    m = statistics.fmean(series)
    crossings = sum(1 for i in range(len(series) - 1)
                    if (series[i] - m) * (series[i + 1] - m) < 0)
    span = dt * (len(series) - 1)
    return (crossings / 2.0) / span if span > 0 else 0.0


def _fit_lambda(z, dt, lag_span_s=0.25):
    """Fit a complex lambda to the error signal z = ex + i*ey.

    Least-squares over lags up to lag_span_s. -Re(lambda) is the decay rate
    k*cos(t) and Im(lambda) the rotation rate -k*sin(t), so t = atan2(-Im, -Re)
    recovers the yaw misalignment. See the module docstring for the derivation.
    """
    n = len(z)
    if n < 30 or dt <= 0:
        return None
    L1 = 1
    M = max(1, int(round(lag_span_s / dt)))
    while L1 + M >= n - 10 and M > 1:
        M //= 2
    L2 = L1 + M
    m = n - L2
    if m < 10:
        return None
    r1 = sum(z[i].conjugate() * z[i + L1] for i in range(m))
    r2 = sum(z[i].conjugate() * z[i + L2] for i in range(m))
    if abs(r1) < 1e-15 or abs(r2) < 1e-15:
        return None
    ratio = r2 / r1
    # Phase must stay inside +-pi over the lag span or the rotation aliases.
    if abs(cmath.phase(ratio)) > 0.9 * math.pi:
        return None
    return cmath.log(ratio) / (M * dt)


def _windowed_omegas(z, dt, n_windows=6):
    """Per-window rotation rate, for measuring how persistent the orbit is."""
    n = len(z)
    w = n // n_windows
    if w < 40:
        return 0, []
    out = []
    for k in range(n_windows):
        seg = z[k * w:(k + 1) * w]
        lam = _fit_lambda(seg, dt)
        if lam is not None:
            out.append(lam.imag)
    return len(out), out


def analyze(samples):
    """Returns a dict of measured quantities. No opinions -- see verdicts()."""
    if len(samples) < 20:
        return {"error": f"only {len(samples)} samples; need >= 20"}

    ts = [s["t"] for s in samples]
    dts = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    if not dts:
        return {"error": "timestamps are not monotonic"}
    dt = statistics.median(dts)

    # Horizontal error as a complex signal
    exy = [complex(s["sp_x"] - s["x"], s["sp_y"] - s["y"]) for s in samples]
    bias = _median_complex(exy)
    fluct = [e - bias for e in exy]
    rms = _rms(fluct)

    # ---- noise floor, estimated from the data ------------------------------
    # The phase of a vector shorter than the measurement noise is meaningless.
    # Estimate noise from consecutive differences: at 50+ Hz the true motion
    # between two samples is negligible, so |z[i+1]-z[i]| is essentially noise.
    # For Gaussian noise of std sigma per axis, median|dz| = 1.665*sigma.
    diffs = [abs(fluct[i + 1] - fluct[i]) for i in range(len(fluct) - 1)]
    sigma = (statistics.median(diffs) / 1.665) if diffs else 0.0
    floor = max(0.004, 8.0 * sigma)

    lam = _fit_lambda(fluct, dt)
    n_win, win_omegas = _windowed_omegas(fluct, dt)

    res = {
        "n": len(samples),
        "duration_s": ts[-1] - ts[0],
        "dt_s": dt,
        "sample_hz": 1.0 / dt if dt > 0 else 0.0,
        "bias_x": bias.real,
        "bias_y": bias.imag,
        "bias_mag": abs(bias),
        "rms_xy": rms,
        "peak_xy": max((abs(v) for v in fluct), default=0.0),
        "noise_sigma_m": sigma,
        "fit_floor_m": floor,
        "fit_samples": len(fluct),
    }

    if lam is not None and rms > floor * 0.5:
        a = -lam.real            # decay rate, >0 means converging
        omega = lam.imag         # rotation rate, rad/s
        res["decay_rate"] = a
        res["rotation_rad_s"] = omega
        res["orbit_hz"] = abs(omega) / (2 * math.pi)
        res["yaw_error_deg"] = math.degrees(math.atan2(-omega, a))

        # Directional consistency = does the orbit keep turning the SAME way
        # across the whole record? Measured by splitting into windows and
        # checking sign agreement of the per-window rotation rate.
        #   0.0 = direction is random, 1.0 = every window agrees
        # Per-STEP phase agreement (the obvious choice) does not work here: for
        # a disturbance-driven hover the step-to-step phase is dominated by the
        # disturbance, so a real orbit reads as noise.
        if win_omegas:
            sign = 1.0 if omega > 0 else -1.0
            agree = sum(1 for w in win_omegas if (w > 0) == (sign > 0)) / len(win_omegas)
            res["rotation_consistency"] = max(0.0, 2.0 * agree - 1.0)
            res["n_windows"] = len(win_omegas)
        else:
            res["rotation_consistency"] = 0.0
        res["rotation_sign"] = "CCW" if omega > 0 else "CW"
    else:
        res["fit_failed"] = True

    # ---- orientation sanity -------------------------------------------------
    # A 180 deg jump in the reported yaw is not motion -- no Crazyflie yaws that
    # fast, and certainly not while its position is unchanged. It means the
    # mocap system has TWO valid solutions for the rigid body and is switching
    # between them, which happens when the marker layout is rotationally
    # symmetric. This poisons every attitude update the EKF receives and
    # inverts the frame the position controller works in.
    if all("yaw_deg" in s_ for s_ in samples):
        yaws = [s_["yaw_deg"] for s_ in samples]
        flips, impossible, rates = 0, 0, []
        for i in range(len(yaws) - 1):
            d = _wrap180(yaws[i + 1] - yaws[i])
            step = ts[i + 1] - ts[i]
            if step <= 0:
                continue
            rate = abs(d) / step
            rates.append(rate)
            if abs(d) > 150.0:
                flips += 1
            if rate > IMPOSSIBLE_YAW_DPS:
                impossible += 1
        res["yaw_flips"] = flips
        res["yaw_impossible"] = impossible
        res["yaw_max_rate_dps"] = max(rates) if rates else 0.0
        res["yaw_span_deg"] = max(yaws) - min(yaws)
        # Yaw movement while the drone is essentially stationary is the cleanest
        # possible evidence: the geometry is bad, independent of any flying.
        still = [i for i in range(len(samples) - 1)
                 if math.dist((samples[i]["x"], samples[i]["y"], samples[i]["z"]),
                              (samples[i+1]["x"], samples[i+1]["y"],
                               samples[i+1]["z"])) < 0.002]
        if len(still) > 10:
            res["yaw_jitter_while_still_deg"] = max(
                abs(_wrap180(yaws[i + 1] - yaws[i])) for i in still)

    # ---- vertical -----------------------------------------------------------
    ez = [s["sp_z"] - s["z"] for s in samples]
    res["bias_z"] = statistics.median(ez)
    res["rms_z"] = math.sqrt(statistics.fmean(
        [(e - res["bias_z"]) ** 2 for e in ez]))

    # ---- oscillation frequency ---------------------------------------------
    res["osc_hz_x"] = _zero_cross_hz([v.real for v in fluct], dt)
    res["osc_hz_y"] = _zero_cross_hz([v.imag for v in fluct], dt)
    res["osc_hz_z"] = _zero_cross_hz(ez, dt)

    # ---- estimator agreement ------------------------------------------------
    if all("ex" in s for s in samples):
        gaps = [math.dist((s["ex"], s["ey"], s["ez"]), (s["x"], s["y"], s["z"]))
                for s in samples]
        res["ekf_gap_mean"] = statistics.fmean(gaps)
        res["ekf_gap_max"] = max(gaps)

    if all("latency_ms" in s for s in samples):
        lat = [s["latency_ms"] for s in samples if s["latency_ms"] > 0]
        if lat:
            res["latency_med_ms"] = statistics.median(lat)
            res["latency_p95_ms"] = sorted(lat)[int(0.95 * len(lat)) - 1]

    # ---- how much is it actually wandering ---------------------------------
    speeds = []
    for i in range(len(samples) - 1):
        step = ts[i + 1] - ts[i]
        if step > 0:
            speeds.append(math.dist(
                (samples[i + 1]["x"], samples[i + 1]["y"]),
                (samples[i]["x"], samples[i]["y"])) / step)
    if speeds:
        res["mean_speed_ms"] = statistics.fmean(speeds)

    return res


# ---------------------------------------------------------------------------
def verdicts(r):
    """Ordered hypotheses: cheapest and most likely first."""
    if "error" in r:
        return [("ERROR", r["error"], "")]
    out = []

    if r.get("yaw_flips", 0) > 0 or r.get("yaw_impossible", 0) > 0:
        extra = ""
        if "yaw_jitter_while_still_deg" in r:
            extra = (f" While the drone was STATIONARY the yaw moved "
                     f"{r['yaw_jitter_while_still_deg']:.0f} deg -- that cannot "
                     f"be real motion.")
        out.append((
            "PRIMARY",
            f"ORIENTATION TRACKING FAILURE: {r['yaw_flips']} yaw jumps >150 deg, "
            f"peak rate {r.get('yaw_max_rate_dps', 0):.0f} deg/s.{extra}",
            "Your Vicon rigid body has an AMBIGUOUS (rotationally symmetric) "
            "marker layout, so Tracker has two valid solutions and keeps "
            "swapping between them. Each swap inverts the frame the position "
            "controller corrects in, which makes the loop push the wrong way.\n"
            "FIX: re-place the markers so no rotation maps the set onto itself "
            "-- 4-5 markers at DIFFERENT HEIGHTS with irregular spacing -- then "
            "delete and re-create the rigid body in Tracker.\n"
            "FLY TODAY: --pos-only ignores the quaternion entirely. Expect the "
            "spiral to disappear. Do not paper over this with a yaw offset; an "
            "offset cannot fix a solution that is jumping."))

    yaw = r.get("yaw_error_deg")
    cons = r.get("rotation_consistency", 0.0)
    if (yaw is not None and abs(yaw) > YAW_SIGNIFICANT_DEG
            and cons > ROTATION_CONSISTENCY):
        sev = "PRIMARY" if abs(yaw) > 25 else "LIKELY"
        out.append((
            sev,
            f"YAW / FRAME MISALIGNMENT: estimated {yaw:+.0f} deg. The position "
            f"error orbits {r['rotation_sign']} at {r['orbit_hz']:.2f} Hz with "
            f"{cons*100:.0f}% directional consistency.",
            "The rigid-body X axis in Vicon Tracker is not the drone's nose, or "
            "the quaternion convention differs. Fix it in Tracker -- NOT in the "
            "flight code. Verify with: --check, drone in hand.\n"
            "Fast discriminator: re-fly with --pos-only. If the orbit vanishes, "
            "it is the injected quaternion."))

    if r.get("decay_rate", 1.0) <= 0 and r.get("fit_samples", 0) >= 10:
        out.append((
            "PRIMARY",
            f"UNSTABLE POSITION LOOP: measured decay rate "
            f"{r['decay_rate']:+.2f} 1/s (negative = error grows).",
            "Error is being amplified, not corrected. A yaw misalignment beyond "
            "90 deg does exactly this. Check frame alignment before gains."))

    if r["bias_mag"] > BIAS_SIGNIFICANT_M:
        out.append((
            "LIKELY",
            f"CONSTANT OFFSET of {r['bias_mag']*100:.0f} cm toward "
            f"({r['bias_x']:+.2f}, {r['bias_y']:+.2f}).",
            "A steady offset in one direction is not a tuning problem. Check, in "
            "this order: (1) the Vicon rigid-body origin is at the drone's "
            "centre, not on a marker; (2) props undamaged and correctly "
            "oriented; (3) the marker deck has not shifted the CG."))

    gap = r.get("ekf_gap_mean")
    if gap is not None and gap > EKF_GAP_SIGNIFICANT_M:
        out.append((
            "LIKELY",
            f"ESTIMATOR NOT TRACKING: onboard estimate sits {gap*100:.0f} cm "
            f"from Vicon on average (peak {r['ekf_gap_max']*100:.0f} cm).",
            "Fix injection before anything else: raise --extpose-hz (you have "
            "240 Hz of Vicon available), and check locSrv.extPosStdDev."))

    fast = max(r.get("osc_hz_x", 0), r.get("osc_hz_y", 0))
    if fast > FAST_OSC_HZ and r["rms_xy"] > GOOD_RMS_M:
        out.append((
            "POSSIBLE",
            f"OSCILLATION at {fast:.1f} Hz, {r['rms_xy']*100:.1f} cm RMS.",
            "This is the one pattern where gains are a fair suspect -- but try "
            "cheaper causes first: (1) raise --extpose-hz to 100 so the EKF's "
            "VELOCITY estimate improves (the velocity PID is the second cascade "
            "level and is fed by a derived quantity); (2) check latency below; "
            "(3) only then lower velCtlPid gains."))

    lat = r.get("latency_med_ms")
    if lat is not None and lat > 30:
        out.append((
            "POSSIBLE",
            f"LATENCY {lat:.0f} ms median, {r.get('latency_p95_ms', 0):.0f} ms p95.",
            "Phase lag in the position loop causes exactly this kind of "
            "wandering. If the Vicon PC and this machine are the same box, this "
            "is real delay; if not, suspect clock skew before believing it."))

    if not out:
        if r["rms_xy"] <= GOOD_RMS_M:
            out.append((
                "OK",
                f"{r['rms_xy']*100:.1f} cm RMS horizontal is a NORMAL mocap "
                f"hover. Nothing here needs fixing.",
                "A Crazyflie always moves a little. If this still looks like too "
                "much drift, the expectation is the thing to adjust, not the "
                "gains -- 1-3 cm RMS is what good looks like."))
        else:
            out.append((
                "UNCLEAR",
                f"{r['rms_xy']*100:.1f} cm RMS with no single dominant "
                f"signature.",
                "Re-fly for longer (--diagnose 30), and try --pos-only as a "
                "discriminator on the quaternion path."))
    return out


def report(r) -> str:
    L = ["", "=" * 72, "HOVER DIAGNOSTICS", "=" * 72]
    if "error" in r:
        return "\n".join(L + [f"  {r['error']}", "=" * 72])

    L += [
        f"  samples      : {r['n']} over {r['duration_s']:.1f} s "
        f"@ {r['sample_hz']:.0f} Hz",
        f"  horizontal   : RMS {r['rms_xy']*100:5.1f} cm   "
        f"peak {r['peak_xy']*100:5.1f} cm   "
        f"(healthy is <= {GOOD_RMS_M*100:.0f} cm RMS)",
        f"  offset       : {r['bias_mag']*100:5.1f} cm toward "
        f"({r['bias_x']:+.3f}, {r['bias_y']:+.3f}) m",
        f"  vertical     : offset {r['bias_z']*100:+.1f} cm, "
        f"RMS {r['rms_z']*100:.1f} cm",
        f"  wander speed : {r.get('mean_speed_ms', 0)*100:.1f} cm/s mean",
    ]
    if "yaw_error_deg" in r:
        L += [
            f"  loop fit     : decay {r['decay_rate']:+.2f} 1/s, "
            f"rotation {r['rotation_rad_s']:+.2f} rad/s "
            f"({r['orbit_hz']:.2f} Hz {r['rotation_sign']})",
            f"  => YAW ERROR : {r['yaw_error_deg']:+.1f} deg  "
            f"(consistency {r['rotation_consistency']*100:.0f}%, "
            f"{r['fit_samples']} pts)",
        ]
    else:
        L += ["  loop fit     : not enough motion above the noise floor"]
    if "yaw_flips" in r:
        L += [f"  yaw sanity   : {r['yaw_flips']} flips >150deg, "
              f"{r['yaw_impossible']} impossible rates, "
              f"peak {r['yaw_max_rate_dps']:.0f} deg/s, "
              f"span {r['yaw_span_deg']:.0f} deg"]
        if "yaw_jitter_while_still_deg" in r:
            L += [f"                 {r['yaw_jitter_while_still_deg']:.0f} deg of "
                  f"yaw movement while STATIONARY"]
    L += [f"  oscillation  : x {r.get('osc_hz_x', 0):.1f} Hz  "
          f"y {r.get('osc_hz_y', 0):.1f} Hz  z {r.get('osc_hz_z', 0):.1f} Hz"]
    if "ekf_gap_mean" in r:
        L += [f"  EKF vs Vicon : mean {r['ekf_gap_mean']*100:.1f} cm, "
              f"max {r['ekf_gap_max']*100:.1f} cm"]
    if "latency_med_ms" in r:
        L += [f"  latency      : med {r['latency_med_ms']:.0f} ms, "
              f"p95 {r['latency_p95_ms']:.0f} ms"]

    L += ["", "-" * 72, "DIAGNOSIS (work these top-down)", "-" * 72]
    for i, (sev, what, fix) in enumerate(verdicts(r), 1):
        L.append(f"{i}. [{sev}] {what}")
        for line in fix.splitlines():
            if line.strip():
                L.append(f"      {line}")
        L.append("")
    L.append("=" * 72)
    return "\n".join(L)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    samples = load_csv(sys.argv[1])
    print(report(analyze(samples)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
