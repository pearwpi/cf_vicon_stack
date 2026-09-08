#!/usr/bin/env python3
"""Follow-up to fix_docker.py, from what the first working build measured.

Run from the cf_vicon_stack root:   python3 fix_docker2.py
Requires fix_docker.py to have been applied first.

1. UNPIN cflib again.
   fix_docker.py defaulted to cflib==0.1.28 on the theory that cflib 0.1.29+
   (numpy~=2.2) could not work with Humble's NumPy-1.x rosidl extensions. That
   was measured on 2026-09-01 and it is FALSE for ros:humble-ros-base: with
   numpy 2.2.6 installed, a float64[9] field round-trips through
   serialize/deserialize as a correct ndarray. Pinning to an old cflib on a
   fear that has been disproved costs real fixes for no benefit.

2. Make verify.py's guard non-vacuous, which is what let the theory stand so
   long. `import geometry_msgs.msg` does NOT touch NumPy: the ABI break only
   shows up on messages with NUMERIC ARRAY fields, and neither geometry_msgs'
   plain Pose nor std_srvs has one. The check passed under both NumPy 1 and
   NumPy 2 while proving nothing either way. Round-trip a float64[36] field so
   the guard actually exercises the path it claims to protect.
"""
import ast
import os
import shutil
import sys
import time

DF, VF = "docker/Dockerfile", "docker/verify.py"
for f in (DF, VF):
    if not os.path.exists(f):
        sys.exit("not found: %s\nRun from the cf_vicon_stack root." % f)
src = {f: open(f).read() for f in (DF, VF)}
edits = []

D_OLD = '''# cflib 0.1.29 moved to numpy~=2.2, which Humble's NumPy-1.x rosidl extensions
# cannot load. 0.1.28 (numpy~=1.26) is the newest that works on Humble.
# Override only if you know the pair is compatible.
ARG CFLIB_VERSION="cflib==0.1.28"'''
D_NEW = '''# NumPy 2 was expected to break Humble's rosidl extensions with "_ARRAY_API
# not found". Measured 2026-09-01 on ros:humble-ros-base: it does not. cflib
# 0.1.33 pulls numpy 2.2.6, and a float64[36] covariance field round-trips
# through serialize/deserialize as a correct ndarray. Left UNPINNED on purpose
# -- verify.py now exercises that exact path, so a future incompatible pair
# fails the build instead of shipping.
ARG CFLIB_VERSION="cflib"'''
edits.append((DF, D_OLD, D_NEW, "unpin cflib (measured, not assumed)",
              'ARG CFLIB_VERSION="cflib"\n'))

V_OLD = '''    try:
        import cflib.crtp                  # noqa: F401
        import cflib.crazyflie             # noqa: F401
    except Exception as exc:
        sys.exit("\\n*** BUILD ABORTED ***\\n"
                 f"cflib will not import: {type(exc).__name__}: {exc}\\n")'''
V_NEW = '''    # Importing a message module does NOT touch NumPy. The ABI break only
    # appears on NUMERIC ARRAY fields, which neither geometry_msgs' Pose nor
    # std_srvs has -- so the imports above pass identically under NumPy 1 and
    # NumPy 2 and prove nothing. Round-trip a float64[36] field, which is
    # numpy-backed, so this guard is worth the line it occupies.
    try:
        from rclpy.serialization import serialize_message, deserialize_message
        from geometry_msgs.msg import PoseWithCovariance
        _m = PoseWithCovariance()
        _m.covariance = [0.5] * 36
        _r = deserialize_message(serialize_message(_m), PoseWithCovariance)
        assert len(_r.covariance) == 36, "covariance length %d" % len(_r.covariance)
        assert abs(float(_r.covariance[7]) - 0.5) < 1e-12, _r.covariance[7]
    except Exception as exc:
        sys.exit(
            "\\n*** BUILD ABORTED ***\\n"
            f"{type(exc).__name__}: {exc}\\n\\n"
            f"NumPy {numpy.__version__} cannot round-trip a numeric array field\\n"
            "through the ROS message extensions. This is the real NumPy ABI\\n"
            "clash, not a missing import. Pin cflib to the last release that\\n"
            "predates its NumPy 2 move:\\n"
            "    --build-arg CFLIB_VERSION='cflib==0.1.28'\\n"
            "and set the same default in docker/docker-compose.yml, which\\n"
            "overrides the Dockerfile ARG.\\n")

    try:
        import cflib.crtp                  # noqa: F401
        import cflib.crazyflie             # noqa: F401
    except Exception as exc:
        sys.exit("\\n*** BUILD ABORTED ***\\n"
                 f"cflib will not import: {type(exc).__name__}: {exc}\\n")'''
edits.append((VF, V_OLD, V_NEW, "verify.py: round-trip a numpy-backed field",
              "PoseWithCovariance"))

todo, done = [], []
for f, old, new, label, marker in edits:
    if marker in src[f]:
        done.append(label)
    elif old in src[f]:
        todo.append((f, old, new, label))
    else:
        sys.exit("ANCHOR NOT FOUND for '%s' in %s.\nNothing written. Has "
                 "fix_docker.py been applied?" % (label, f))
for l in done:
    print("  already applied: %s" % l)
if not todo:
    print("\nNothing to do.")
    sys.exit(0)

bak = "_backup_docker2_" + time.strftime("%Y%m%d-%H%M%S")
os.makedirs(bak, exist_ok=True)
for f in {t[0] for t in todo}:
    shutil.copy2(f, os.path.join(bak, os.path.basename(f)))
print("\nbacked up to %s/" % bak)
out = dict(src)
for f, old, new, label in todo:
    out[f] = out[f].replace(old, new, 1)
    print("  %s: %s" % (f, label))
ast.parse(out[VF])
for f in {t[0] for t in todo}:
    open(f, "w").write(out[f])
print("\nwrote %s" % ", ".join(sorted({t[0] for t in todo})))
print("\nRebuild to prove the new guard runs:")
print("  cd docker && docker compose build")
