#!/usr/bin/env python3
"""Build-time self-check. Run inside the image; a non-zero exit fails the build.

Kept as a file rather than a RUN heredoc so the Dockerfile does not depend on
BuildKit heredoc support, and so the check is readable on its own.

    python3 verify.py imports     inside the image, after installing cflib
    python3 verify.py packages    inside the image, after colcon build
    python3 verify.py context     on the HOST, from the stack root, before building

The `context` mode is the one that catches the mistakes you cannot see until a
build has already burned ten minutes: a COPY source that does not exist, a
.dockerignore that quietly drops the test fixture, a symlink that would dangle
because of COPY ordering, or a bind mount that would shadow the compiled
workspace.
"""
import sys

MODE = sys.argv[1] if len(sys.argv) > 1 else "imports"

if MODE == "imports":
    # cflib 0.1.32 wants numpy~=2.2; Humble's rosidl-generated message
    # extensions are built against NumPy 1.x and can fail to import under 2.x
    # with "_ARRAY_API not found". If that is the case here, fail NOW with an
    # actionable message rather than shipping an image that dies at the drone.
    try:
        import numpy
    except Exception as exc:
        sys.exit("\n*** BUILD ABORTED ***\n"
                 f"numpy will not import at all: {type(exc).__name__}: {exc}\n")

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
                "\n*** BUILD ABORTED ***\n"
                f"{type(exc).__name__}: {exc}\n\n"
                "ROS is not on the Python path in this build step.\n"
                "This is a Dockerfile bug, NOT a NumPy problem: a RUN line does\n"
                "not go through the image entrypoint, so it must source ROS:\n"
                "    RUN source /opt/ros/${ROS_DISTRO}/setup.bash \\\n"
                "        && python3 ...\n")
        raise
    except Exception as exc:
        sys.exit(
            "\n*** BUILD ABORTED ***\n"
            f"{type(exc).__name__}: {exc}\n\n"
            f"NumPy {numpy.__version__} and the ROS message extensions disagree.\n"
            "Humble's rosidl extensions are built against NumPy 1.x, and cflib\n"
            "0.1.29+ requires numpy~=2.2. Rebuild with the last cflib that\n"
            "predates that move:\n"
            "    --build-arg CFLIB_VERSION='cflib==0.1.28'\n\n"
            "PIP_EXTRA='numpy<2' does NOT work here: against an unpinned cflib\n"
            "pip sees numpy~=2.2 and numpy<2 together and fails to resolve.\n")

    # Importing a message module does NOT touch NumPy. The ABI break only
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
            "\n*** BUILD ABORTED ***\n"
            f"{type(exc).__name__}: {exc}\n\n"
            f"NumPy {numpy.__version__} cannot round-trip a numeric array field\n"
            "through the ROS message extensions. This is the real NumPy ABI\n"
            "clash, not a missing import. Pin cflib to the last release that\n"
            "predates its NumPy 2 move:\n"
            "    --build-arg CFLIB_VERSION='cflib==0.1.28'\n"
            "and set the same default in docker/docker-compose.yml, which\n"
            "overrides the Dockerfile ARG.\n")

    try:
        import cflib.crtp                  # noqa: F401
        import cflib.crazyflie             # noqa: F401
    except Exception as exc:
        sys.exit("\n*** BUILD ABORTED ***\n"
                 f"cflib will not import: {type(exc).__name__}: {exc}\n")
    print(f"OK  numpy {numpy.__version__}; rclpy, geometry_msgs, std_srvs "
          f"and cflib all import together")

elif MODE == "packages":
    import crazyflie_interfaces.msg as m
    missing = [n for n in ("Position", "Hover", "FullState", "VelocityWorld",
                           "Status", "LogDataGeneric") if not hasattr(m, n)]
    if missing:
        sys.exit(f"*** crazyflie_interfaces is missing: {missing}")
    import crazyflie_interfaces.srv as s
    missing = [n for n in ("Takeoff", "Land", "GoTo", "Arm", "Stop",
                           "NotifySetpointsStop") if not hasattr(s, n)]
    if missing:
        sys.exit(f"*** crazyflie_interfaces srv is missing: {missing}")
    print("OK  crazyflie_interfaces messages and services all generated")

elif MODE == "context":
    import fnmatch
    import os
    import re
    ok = True

    def chk(name, cond, detail=""):
        global ok
        print(("  PASS  " if cond else "  FAIL  ") + name
              + (("  " + detail) if detail and not cond else ""))
        ok = ok and cond

    root = os.getcwd()
    if not os.path.exists("docker/Dockerfile"):
        sys.exit("run this from the stack root (the build context)")
    df = open("docker/Dockerfile").read()

    srcs = []
    for m in re.finditer(r"^COPY\s+(?:--\S+\s+)*(.+)$", df, re.M):
        srcs += m.group(1).split()[:-1]
    for s_ in sorted(set(srcs)):
        chk(f"COPY source exists: {s_}",
            os.path.exists(os.path.join(root, s_.rstrip("/"))))

    ign = [l.strip() for l in open(".dockerignore")
           if l.strip() and not l.startswith("#")]
    chk("no trailing comments in .dockerignore",
        not any("#" in l for l in ign),
        str([l for l in ign if "#" in l]))
    neg = [p_[1:] for p_ in ign if p_.startswith("!")]
    pos = [p_ for p_ in ign if not p_.startswith("!")]

    def excluded(path):
        hit = any(fnmatch.fnmatch(path, q) or fnmatch.fnmatch(path, q.rstrip("/"))
                  for q in pos)
        return hit and not any(fnmatch.fnmatch(path, n) for n in neg)

    for need in ("tilt_capture_1786563877.csv", "cf_core.py", "cf_keyboard.py",
                 "crazyflie_vicon_teleop.py", "test_crazyflie_ros.py",
                 "docker/verify.py", "docker/entrypoint.sh"):
        chk(f"survives .dockerignore: {need}", not excluded(need))

    def _identical(a, b):
        try:
            with open(a, "rb") as fa, open(b, "rb") as fb:
                return fa.read() == fb.read()
        except OSError:
            return False

    # Real files now, not symlinks (see fix_symlinks.py and DUPLICATES.json), so
    # the context check is byte identity rather than link resolution. It passes
    # either way, so it survives a change back.
    for rel, tgt in (("src/crazyflie_ros/crazyflie_ros/cf_core.py", "cf_core.py"),
                     ("src/crazyflie_ros/crazyflie_ros/cf_keyboard.py", "cf_keyboard.py")):
        chk(f"copies identical in context: {os.path.basename(rel)}",
            _identical(rel, tgt), "fix with: python3 fix_symlinks.py --sync")

    chk("cf_core.py copied before src/ (the order the build-time guard assumes)",
        df.index("COPY cf_core.py") < df.index("COPY src/"))
    chk("both copied before colcon build",
        max(df.index("COPY cf_core.py"), df.index("COPY src/")) < df.index("colcon build"))
    chk("no RUN heredocs (they would require BuildKit + syntax=1.4)",
        "<<'PY'" not in df and "<<EOF" not in df)

    try:
        import yaml
        c = yaml.safe_load(open("docker/docker-compose.yml"))["services"]["cf"]
        mounts = [v.split(":")[1] for v in c["volumes"]]
        chk("no bind mount shadows the build prefix /opt/cfvicon",
            not any(m == "/opt/cfvicon" or m.startswith("/opt/cfvicon/") for m in mounts),
            str(mounts))
        chk("compose build context is the stack root", c["build"]["context"] == "..")
        chk("USB is bind-mounted, not `devices:` (hot-plug)",
            any(v.startswith("/dev/bus/usb:") for v in c["volumes"]) and "devices" not in c)
        chk("device_cgroup_rules grants USB char major 189",
            any("189" in r for r in c.get("device_cgroup_rules", [])))
        declared = set(re.findall(r"^ARG\s+([A-Z_]+)", df, re.M))
        for a in c["build"]["args"]:
            chk(f"ARG declared in Dockerfile: {a}", a in declared)
    except ImportError:
        print("  SKIP  compose checks (pyyaml not installed)")

    print("\n  " + ("ALL CONTEXT CHECKS PASSED" if ok else "FAILURES ABOVE"))
    sys.exit(0 if ok else 1)

else:
    sys.exit(f"unknown mode {MODE!r}")
