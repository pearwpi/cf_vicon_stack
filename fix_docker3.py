#!/usr/bin/env python3
"""Reconcile the Dockerfile guard with fix_symlinks.py.

fix_docker.py (Sep 1 16:46) asserted cf_core.py is a SYMLINK; fix_symlinks.py
(Sep 1 17:42) made it a REAL FILE. Every build since has failed at that guard.
Keep the guard, change what it asserts: identity, not link resolution -- and
cover cf_keyboard.py too, which DUPLICATES.json lists and the old guard missed.
"""
import os, shutil, sys, time

DF = "docker/Dockerfile"
if not os.path.exists(DF):
    sys.exit("not found: %s -- run from the cf_vicon_stack root." % DF)
src = open(DF).read()
if "sha256sum < ${CFV_ROOT}" in src:
    sys.exit("Already applied; nothing to do.")

OLD = """RUN test "$(readlink -f ${CFV_ROOT}/src/crazyflie_ros/crazyflie_ros/cf_core.py)" \\
       = "${CFV_ROOT}/cf_core.py" \\
    || { echo "*** cf_core.py symlink does not resolve to the stack root."; \\
         echo "    Re-create it on the host with:"; \\
         echo "    ln -sf ../../../cf_core.py src/crazyflie_ros/crazyflie_ros/cf_core.py"; \\
         exit 1; }"""

NEW = """RUN for f in cf_core.py cf_keyboard.py; do \\
      test "$(sha256sum < ${CFV_ROOT}/$f)" \\
         = "$(sha256sum < ${CFV_ROOT}/src/crazyflie_ros/crazyflie_ros/$f)" \\
      || { echo "*** $f differs between the stack root and the ROS package."; \\
           echo "    Fix on the host with:  python3 fix_symlinks.py --sync"; \\
           exit 1; }; \\
    done"""

if src.count(OLD) != 1:
    sys.exit("anchor not found or not unique (%d) -- nothing written. Send this to Claude."
             % src.count(OLD))
print("  ok  anchor matched")
if "--apply" not in sys.argv:
    sys.exit("report only. re-run with --apply.")

bak = "_backup_docker3_" + time.strftime("%Y%m%d-%H%M%S")
os.makedirs(bak, exist_ok=True)
shutil.copy2(DF, bak + "/Dockerfile")
open(DF, "w").write(src.replace(OLD, NEW, 1))
print("  backed up to %s/ and wrote %s" % (bak, DF))
print("\nNow:  docker compose -f docker/docker-compose.yml build")
