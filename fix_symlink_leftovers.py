#!/usr/bin/env python3
"""Finish what fix_symlinks.py started.

cf_core.py and cf_keyboard.py used to be ONE file reached by two import paths:
the package copy was a symlink to the stack root. Symlinks do not survive most
transfers -- the USB copy to pear-2 dropped them -- so fix_symlinks.py made them
real files and moved the anti-drift guarantee to DUPLICATES.json +
test_duplicates.py. Five places still assert the OLD invariant. Two are
executable and one has been failing the Docker build. The failure mode being
guarded has NOT changed, so every check is kept and only its assertion changes.
"""
import os, shutil, sys, time

TEST, VERIFY = "test_crazyflie_ros.py", "docker/verify.py"
README, DOCKERMD, COMPOSE = "README.md", "docker/DOCKER.md", "docker/docker-compose.yml"
EDITS = []

EDITS.append((TEST, """# NOT an identity check. The standalone imports `cf_core`; the node imports
# `crazyflie_ros.cf_core`. Python builds a separate module object per import
# NAME even when both names resolve to one file, so `T.Pose is C.Pose` is
# false here -- and irrelevant, because the two programs never share an
# interpreter. The invariant that actually prevents drift is that both names
# resolve to the SAME FILE ON DISK, which is what the symlink buys and what is
# asserted below, followed by behavioural equivalence on the guards themselves.
check("both import paths resolve to one file on disk",
      os.path.realpath(TopCore.__file__) == os.path.realpath(C.__file__),
      f"{os.path.realpath(TopCore.__file__)} vs {os.path.realpath(C.__file__)}")
check("the standalone's names come from that file",
      os.path.realpath(sys.modules[T.Pose.__module__].__file__)
      == os.path.realpath(C.__file__))""",
"""# NOT an identity check. The standalone imports `cf_core`; the node imports
# `crazyflie_ros.cf_core`. Python builds a separate module object per import
# NAME even when both names resolve to one file, so `T.Pose is C.Pose` is
# false here -- and irrelevant, because the two programs never share an
# interpreter.
#
# These were once ONE file: the package copy was a symlink to the stack root,
# and the invariant asserted here was same-inode. fix_symlinks.py made them
# real files, because symlinks do not survive a transfer -- that is exactly how
# the copy to pear-2 lost them. The invariant is therefore BYTE IDENTITY now,
# held on the host by DUPLICATES.json + test_duplicates.py. What is being
# guarded has not changed: two diverging copies of the safety core, with the
# node and the standalone enforcing different limits and nothing complaining.
# Content equality also passes if they are ever symlinked again, so this check
# does not have to be revisited either way.
def _sha_of(path):
    import hashlib
    with open(os.path.realpath(path), "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()

check("both import paths load byte-identical files",
      _sha_of(TopCore.__file__) == _sha_of(C.__file__),
      f"{TopCore.__file__} vs {C.__file__}"
      "  -- fix with: python3 fix_symlinks.py --sync")
check("the standalone's names come from that same content",
      _sha_of(sys.modules[T.Pose.__module__].__file__) == _sha_of(C.__file__))""",
"test: realpath equality -> byte identity"))

EDITS.append((VERIFY, """    for rel, tgt in (("src/crazyflie_ros/crazyflie_ros/cf_core.py", "cf_core.py"),
                     ("src/crazyflie_ros/crazyflie_ros/cf_keyboard.py", "cf_keyboard.py")):
        chk(f"symlink resolves in context: {os.path.basename(rel)}",
            os.path.realpath(rel) == os.path.realpath(tgt))

    chk("cf_core.py copied before src/ (else the symlink dangles at build time)",
        df.index("COPY cf_core.py") < df.index("COPY src/"))""",
"""    def _identical(a, b):
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
        df.index("COPY cf_core.py") < df.index("COPY src/"))""",
"verify.py: context check -> byte identity"))

EDITS.append((README,
"""|       |-- crazyflie_ros/cf_core.py            -> symlink to ../../../cf_core.py
|       |-- crazyflie_ros/cf_keyboard.py        -> symlink to ../../../cf_keyboard.py""",
"""|       |-- crazyflie_ros/cf_core.py            copy of ../../../cf_core.py, kept identical
|       |-- crazyflie_ros/cf_keyboard.py        copy of ../../../cf_keyboard.py, kept identical""",
"README: tree annotation"))

EDITS.append((README,
"""**`cf_core.py` and `cf_keyboard.py` inside the ROS package are symlinks** to the
copies at the root. One file on disk, two import paths, so the driver and the
standalone cannot drift apart. `colcon build` follows the symlink and installs a
real copy. If you ever move this folder with a tool that does not preserve
symlinks (some zip/rsync defaults), re-create them:

    ln -sf ../../../cf_core.py     src/crazyflie_ros/crazyflie_ros/cf_core.py
    ln -sf ../../../cf_keyboard.py src/crazyflie_ros/crazyflie_ros/cf_keyboard.py""",
"""**`cf_core.py` and `cf_keyboard.py` exist twice**: at the root, and inside the
ROS package. The root copy is canonical. They used to be symlinks -- one file
on disk, two import paths -- but symlinks do not survive most transfers, and the
USB copy to the lab PC dropped them and left the suite dying on ImportError.
They are real files in both places now.

Drift is prevented by a test instead of by the filesystem. `DUPLICATES.json`
records the pairs and `test_duplicates.py` fails the moment two copies differ,
so a divergence shows up as a red test rather than as the driver and the
standalone silently enforcing different safety limits. After editing a
canonical copy, push it out and check:

    python3 fix_symlinks.py --sync
    python3 -m pytest test_duplicates.py -q""",
"README: the symlink paragraph"))

EDITS.append((DOCKERMD,
"""The image runs all 238 checks from the four suites during `docker build` and
fails if any of them do, so a student image is never one that failed its own
tests. It also verifies that `cf_core.py` inside the ROS package still resolves
to the stack root — if a context ever flattens that symlink you would get two
silently diverging copies of the safety core, which is the one failure mode that
would be genuinely dangerous.""",
"""The image runs every check from the four suites during `docker build` and fails
if any of them do, so a student image is never one that failed its own tests. It
also verifies that the two copies of `cf_core.py` — the stack root and the ROS
package — are byte-identical. They were symlinks once, and the guard checked
that the link resolved; symlinks do not survive a transfer, so they are real
files now and the guard checks content. The failure mode is unchanged: two
silently diverging copies of the safety core is the one thing here that would be
genuinely dangerous.""",
"DOCKER.md: the build-tests-itself paragraph"))

EDITS.append((COMPOSE,
"""      # Context is the STACK ROOT so the cf_core.py symlinks resolve.""",
"""      # Context is the STACK ROOT: the Dockerfile copies cf_core.py from there
      # and compares it against the copy inside src/.""",
"compose: stale comment"))

FILES = sorted({e[0] for e in EDITS})
missing = [f for f in FILES if not os.path.exists(f)]
if missing:
    sys.exit("not found: %s\nRun from the cf_vicon_stack root." % ", ".join(missing))
cur = {f: open(f, encoding="utf-8").read() for f in FILES}

print("checking anchors...")
todo = []
for f, old, new, label in EDITS:
    if old not in cur[f] and new.strip().splitlines()[0].strip() in cur[f]:
        print("  SKIP  %s (already applied)" % label); continue
    n = cur[f].count(old)
    if n != 1:
        sys.exit("\nANCHOR %s for '%s' in %s (found %d).\nNothing written. Send this to Claude."
                 % ("NOT FOUND" if n == 0 else "NOT UNIQUE", label, f, n))
    print("  ok    %s" % label)
    todo.append((f, old, new))
if not todo:
    sys.exit("\nAll already applied; nothing to do.")
if "--apply" not in sys.argv:
    sys.exit("\nreport only. re-run with --apply to write.")

bak = "_backup_leftovers_" + time.strftime("%Y%m%d-%H%M%S")
for f in sorted({t[0] for t in todo}):
    os.makedirs(os.path.join(bak, os.path.dirname(f)) or bak, exist_ok=True)
    shutil.copy2(f, os.path.join(bak, f))
print("\nbacked up to %s/" % bak)
for f, old, new in todo:
    cur[f] = cur[f].replace(old, new, 1)
for f in sorted({t[0] for t in todo}):
    open(f, "w", encoding="utf-8").write(cur[f]); print("  wrote %s" % f)
print("\nNow:\n  python3 test_crazyflie_ros.py | tail -3"
      "\n  docker compose -f docker/docker-compose.yml build")
