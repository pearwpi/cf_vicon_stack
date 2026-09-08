#!/usr/bin/env python3
"""Fix the three Docker build defects found by the first real build (2026-09-01).

Run from the cf_vicon_stack root:   python3 fix_docker.py

1. docker/Dockerfile -- the pip/verify RUN never sources ROS.
   A RUN line does not go through the image entrypoint, so rclpy is not on the
   Python path there. `verify.py imports` therefore failed with
   "ModuleNotFoundError: No module named 'rclpy'" on EVERY build, whatever the
   NumPy situation. Two later RUN lines source ROS correctly; this one was
   missed.

2. docker/verify.py -- it blamed NumPy for that.
   A bare `except Exception` attributed a missing ROS path to a NumPy ABI
   clash, and then suggested a fix that cannot work. Split the cases.

3. docker/Dockerfile -- default CFLIB_VERSION cannot build on Humble.
   cflib 0.1.29+ requires numpy~=2.2. Humble's rosidl extensions are built
   against NumPy 1.x. cflib 0.1.28 is the last release requiring numpy~=1.26,
   so it is the newest that can work. Default to it rather than shipping an
   image that cannot build.

   Note `PIP_EXTRA='numpy<2'` -- suggested by the old error text -- is not a
   fix: with cflib unpinned pip sees numpy~=2.2 AND numpy<2 and fails to
   resolve at all.

Idempotent and all-or-nothing: every anchor is checked before anything is
written, originals go to _backup_docker_<timestamp>/.
"""
import os
import shutil
import sys
import time

DF = "docker/Dockerfile"
VF = "docker/verify.py"
for f in (DF, VF):
    if not os.path.exists(f):
        sys.exit("not found: %s\nRun this from the cf_vicon_stack root." % f)

src = {f: open(f).read() for f in (DF, VF)}
edits = []      # (file, old, new, label, marker present only AFTER applying)

# --- 1 + 3: Dockerfile -------------------------------------------------------
D1_OLD = '''ARG CFLIB_VERSION="cflib"
ARG PIP_EXTRA=""'''
D1_NEW = '''# cflib 0.1.29 moved to numpy~=2.2, which Humble's NumPy-1.x rosidl extensions
# cannot load. 0.1.28 (numpy~=1.26) is the newest that works on Humble.
# Override only if you know the pair is compatible.
ARG CFLIB_VERSION="cflib==0.1.28"
ARG PIP_EXTRA=""'''
edits.append((DF, D1_OLD, D1_NEW, "default cflib -> 0.1.28",
              'ARG CFLIB_VERSION="cflib==0.1.28"'))

D2_OLD = '''RUN python3 -m pip install --no-cache-dir --upgrade pip \\
    && python3 -m pip install --no-cache-dir ${CFLIB_VERSION} ${PIP_EXTRA} \\
    && python3 /usr/local/bin/verify.py imports'''
D2_NEW = '''RUN source /opt/ros/${ROS_DISTRO}/setup.bash \\
    && python3 -m pip install --no-cache-dir --upgrade pip \\
    && python3 -m pip install --no-cache-dir ${CFLIB_VERSION} ${PIP_EXTRA} \\
    && python3 /usr/local/bin/verify.py imports'''
edits.append((DF, D2_OLD, D2_NEW, "source ROS before verify.py imports",
              "RUN source /opt/ros/${ROS_DISTRO}/setup.bash \\\n    && python3 -m pip"))

# --- 2: verify.py ------------------------------------------------------------
V_OLD = '''    try:
        import numpy
        import rclpy                       # noqa: F401
        import geometry_msgs.msg           # noqa: F401
        import std_srvs.srv                # noqa: F401
        import cflib.crtp                  # noqa: F401
        import cflib.crazyflie             # noqa: F401
    except Exception as exc:
        sys.exit(
            "\\n*** BUILD ABORTED ***\\n"
            f"{type(exc).__name__}: {exc}\\n\\n"
            "The installed NumPy and the ROS message extensions disagree.\\n"
            "Rebuild with one of:\\n"
            "    --build-arg PIP_EXTRA='numpy<2'\\n"
            "    --build-arg CFLIB_VERSION='cflib==0.1.28'\\n")'''
V_NEW = '''    try:
        import numpy
    except Exception as exc:
        sys.exit("\\n*** BUILD ABORTED ***\\n"
                 f"numpy will not import at all: {type(exc).__name__}: {exc}\\n")

    # Distinguish two failures that look alike from the outside. A missing ROS
    # module means ROS was never sourced in this build step, which is a
    # Dockerfile bug; anything else from these imports is the NumPy ABI clash.
    try:
        import rclpy                       # noqa: F401
        import geometry_msgs.msg           # noqa: F401
        import std_srvs.srv                # noqa: F401
    except ModuleNotFoundError as exc:
        if (getattr(exc, "name", "") or "").split(".")[0] in (
                "rclpy", "geometry_msgs", "std_srvs"):
            sys.exit(
                "\\n*** BUILD ABORTED ***\\n"
                f"{type(exc).__name__}: {exc}\\n\\n"
                "ROS is not on the Python path in this build step.\\n"
                "This is a Dockerfile bug, NOT a NumPy problem: a RUN line does\\n"
                "not go through the image entrypoint, so it must source ROS:\\n"
                "    RUN source /opt/ros/${ROS_DISTRO}/setup.bash \\\\\\n"
                "        && python3 ...\\n")
        raise
    except Exception as exc:
        sys.exit(
            "\\n*** BUILD ABORTED ***\\n"
            f"{type(exc).__name__}: {exc}\\n\\n"
            f"NumPy {numpy.__version__} and the ROS message extensions disagree.\\n"
            "Humble's rosidl extensions are built against NumPy 1.x, and cflib\\n"
            "0.1.29+ requires numpy~=2.2. Rebuild with the last cflib that\\n"
            "predates that move:\\n"
            "    --build-arg CFLIB_VERSION='cflib==0.1.28'\\n\\n"
            "PIP_EXTRA='numpy<2' does NOT work here: against an unpinned cflib\\n"
            "pip sees numpy~=2.2 and numpy<2 together and fails to resolve.\\n")

    try:
        import cflib.crtp                  # noqa: F401
        import cflib.crazyflie             # noqa: F401
    except Exception as exc:
        sys.exit("\\n*** BUILD ABORTED ***\\n"
                 f"cflib will not import: {type(exc).__name__}: {exc}\\n")'''
edits.append((VF, V_OLD, V_NEW, "verify.py: separate ROS-path from NumPy ABI",
              'ROS is not on the Python path in this build step'))

todo, done = [], []
for f, old, new, label, marker in edits:
    if marker in src[f]:
        done.append(label)
    elif old in src[f]:
        todo.append((f, old, new, label))
    else:
        sys.exit("ANCHOR NOT FOUND for '%s' in %s.\nNothing written. The file "
                 "differs from the version this patch was written against."
                 % (label, f))

for l in done:
    print("  already applied: %s" % l)
if not todo:
    print("\nNothing to do.")
    sys.exit(0)

bak = "_backup_docker_" + time.strftime("%Y%m%d-%H%M%S")
os.makedirs(bak, exist_ok=True)
for f in {t[0] for t in todo}:
    shutil.copy2(f, os.path.join(bak, os.path.basename(f)))
print("\nbacked up to %s/" % bak)

out = dict(src)
for f, old, new, label in todo:
    out[f] = out[f].replace(old, new, 1)
    print("  %s: %s" % (f, label))

import ast
ast.parse(out[VF])                        # never write a verify.py that will not parse
for f in {t[0] for t in todo}:
    open(f, "w").write(out[f])
print("\nwrote %s" % ", ".join(sorted({t[0] for t in todo})))
print("\nNow rebuild:  cd docker && docker compose build")
